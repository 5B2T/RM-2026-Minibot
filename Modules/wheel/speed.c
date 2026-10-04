#include "wheel/speed.h"
#include "encoder/bsp_encoder.h"
#include <math.h>
static Speed_HandleTypeDef Speed[WHEEL_COUNT];
_Static_assert(WHEEL_COUNT==BSP_ENCODER_COUNT, "Encoder mapping must match wheel count");
uint8_t Speed_Init(float WheelDiameterM, float CountsPerRevolution)
{
    if(!isfinite(WheelDiameterM) || WheelDiameterM<=0 ||
       !isfinite(CountsPerRevolution) || CountsPerRevolution<=0)return 0;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
        Speed[i]=(Speed_HandleTypeDef){.Speed_EncoderCPR=CountsPerRevolution,
                                      .Speed_WheelDia=WheelDiameterM};
    Speed_Reset();
    return 1;
}
void Speed_Reset(void)
{
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
    {
        BSP_Encoder_Sync(i);
        Speed[i].Speed_SampleTime=0;
        Speed[i].Speed_Value=0;
        Speed[i].Speed_Delta=0;
    }
}
uint8_t Speed_Update(float SampleTimeS)
{
    if(!isfinite(SampleTimeS) || SampleTimeS<=0)return 0;
    uint8_t Valid=1;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
    {
        Speed[i].Speed_SampleTime=SampleTimeS;
        Speed[i].Speed_Delta=BSP_Encoder_GetDelta(i);
        Speed[i].Speed_Value=((float)Speed[i].Speed_Delta*3.1415926f*Speed[i].Speed_WheelDia)
                           /(Speed[i].Speed_EncoderCPR*SampleTimeS);
        if(!isfinite(Speed[i].Speed_Value))Valid=0;
    }
    return Valid;
}
void Speed_SetEncoderCPR(float CountsPerRevolution)
{
    if(!isfinite(CountsPerRevolution) || CountsPerRevolution<=0)return;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)Speed[i].Speed_EncoderCPR=CountsPerRevolution;
}
float Speed_Get(uint8_t Wheel)
{
    return Wheel<WHEEL_COUNT?Speed[Wheel].Speed_Value:0;
}
int16_t Speed_GetDelta(uint8_t Wheel)
{
    return Wheel<WHEEL_COUNT?Speed[Wheel].Speed_Delta:0;
}
Speed_HandleTypeDef Speed_GetState(uint8_t Wheel)
{
    return Wheel<WHEEL_COUNT?Speed[Wheel]:(Speed_HandleTypeDef){0};
}
