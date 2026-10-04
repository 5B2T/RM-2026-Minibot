#include "power/ina219.h"
#include <math.h>
#include <stddef.h>

enum {PH_RESET,PH_RESET_WAIT,PH_CONFIG,PH_CALIBRATE,PH_CHECK_CONFIG,PH_CHECK_CAL,
    PH_TRIGGER,PH_WAIT,PH_SHUNT,PH_CURRENT,PH_FINAL_CAL,PH_FAULT};
#define CONFIG_IDLE 0x3e60U // BRNG=32V, PGA=/8, BADC/SADC=16 samples, power-down.
#define CONFIG_TRIGGER (CONFIG_IDLE|3U)
static INA219_WriteFn Send;
static INA219_ReadFn Read;
static uint8_t DeviceAddress,Phase=PH_FAULT;
static uint16_t Calibration,BusRaw,ShuntRaw,CurrentRaw;
static float EffectiveLSB;
static uint32_t StartTick,Timeout;

static uint8_t Write16(uint8_t Register,uint16_t Value)
{
    uint8_t Data[]={Register,(uint8_t)(Value>>8),(uint8_t)Value};
    return Send(DeviceAddress,Data,sizeof(Data));
}
static uint8_t Read16(uint8_t Register,uint16_t *Value)
{
    uint8_t Data[2];
    if(!Read(DeviceAddress,Register,Data,sizeof(Data)))return 0;
    *Value=((uint16_t)Data[0]<<8)|Data[1];
    return 1;
}
static int32_t Signed16(uint16_t Value)
{return Value<0x8000U?(int32_t)Value:(int32_t)Value-65536;}
uint8_t INA219_Init(INA219_WriteFn Write,INA219_ReadFn Reader,uint8_t Address,
    float ShuntOhm,float CurrentLSB,uint32_t ConversionTimeoutMs)
{
    Phase=PH_FAULT;
    if(Write==NULL || Reader==NULL || Address<0x40 || Address>0x4f ||
       !isfinite(ShuntOhm) || ShuntOhm<=0 || !isfinite(CurrentLSB) || CurrentLSB<=0 ||
       ConversionTimeoutMs<20 || ConversionTimeoutMs>1000)return 0;
    float Cal=0.04096f/(ShuntOhm*CurrentLSB);
    if(!isfinite(Cal) || Cal<2 || Cal>=65536)return 0;
    Calibration=(uint16_t)((uint32_t)Cal&0xfffeU); // FS0 is unused and reads zero.
    EffectiveLSB=0.04096f/((float)Calibration*ShuntOhm);
    if(!isfinite(EffectiveLSB) || EffectiveLSB*32767.0f<0.32f/ShuntOhm)return 0;
    Send=Write;Read=Reader;DeviceAddress=Address;Timeout=ConversionTimeoutMs;
    Phase=PH_RESET;
    return 1;
}
INA219_Result INA219_Step(uint32_t NowMs,INA219_Sample *Output)
{
    uint16_t Value;
    INA219_Result Error=INA219_IO_ERROR;
    if(Output==NULL)return INA219_INVALID_SAMPLE;
    switch(Phase)
    {
    case PH_RESET:
        if(!Write16(0,0x8000))goto Failure;
        StartTick=NowMs;Phase=PH_RESET_WAIT;break;
    case PH_RESET_WAIT:
        if((uint32_t)(NowMs-StartTick)>=2)Phase=PH_CONFIG;
        break;
    case PH_CONFIG:
        if(!Write16(0,CONFIG_IDLE))goto Failure;
        Phase=PH_CALIBRATE;break;
    case PH_CALIBRATE:
        if(!Write16(5,Calibration))goto Failure;
        Phase=PH_CHECK_CONFIG;break;
    case PH_CHECK_CONFIG:
        if(!Read16(0,&Value))goto Failure;
        if(Value!=CONFIG_IDLE) {Error=INA219_CONFIG_ERROR;goto Failure;}
        Phase=PH_CHECK_CAL;break;
    case PH_CHECK_CAL:
        if(!Read16(5,&Value))goto Failure;
        if(Value!=Calibration) {Error=INA219_CONFIG_ERROR;goto Failure;}
        Phase=PH_TRIGGER;break;
    case PH_TRIGGER:
        if(!Write16(0,CONFIG_TRIGGER))goto Failure;
        StartTick=NowMs;Phase=PH_WAIT;break;
    case PH_WAIT:
        if((uint32_t)(NowMs-StartTick)>=Timeout) {Error=INA219_TIMEOUT;goto Failure;}
        if(!Read16(2,&BusRaw))goto Failure;
        if(BusRaw&1U) {Error=INA219_OVERFLOW;goto Failure;}
        if(BusRaw&2U)Phase=PH_SHUNT;
        break;
    case PH_SHUNT:
        if(!Read16(1,&ShuntRaw))goto Failure;
        Phase=PH_CURRENT;break;
    case PH_CURRENT:
        if(!Read16(4,&CurrentRaw))goto Failure;
        Phase=PH_FINAL_CAL;break;
    case PH_FINAL_CAL:
        if(!Read16(5,&Value))goto Failure;
        if(Value!=Calibration) {Error=INA219_CONFIG_ERROR;goto Failure;}
        // One-shot measurements remain stable until the next trigger.
        int32_t Shunt=Signed16(ShuntRaw);
        INA219_Sample Sample;
        Sample.VoltageV=(float)(BusRaw>>3)*0.004f+(float)Shunt*0.00001f;
        Sample.CurrentA=(float)Signed16(CurrentRaw)*EffectiveLSB;
        Sample.PowerW=Sample.VoltageV*Sample.CurrentA; // Source power includes shunt loss.
        if(Shunt<=-32000 || Shunt>=32000 || Signed16(CurrentRaw)==-32768 ||
           Signed16(CurrentRaw)==32767) {Error=INA219_OVERFLOW;goto Failure;}
        if(!isfinite(Sample.VoltageV) || Sample.VoltageV<0 || Sample.VoltageV>26 ||
           !isfinite(Sample.CurrentA) || !isfinite(Sample.PowerW))
        {Error=INA219_INVALID_SAMPLE;goto Failure;}
        *Output=Sample;
        Phase=PH_TRIGGER;
        return INA219_SAMPLE;
    default:return INA219_CONFIG_ERROR;
    }
    return INA219_PENDING;
Failure:
    Phase=PH_FAULT;
    return Error;
}
