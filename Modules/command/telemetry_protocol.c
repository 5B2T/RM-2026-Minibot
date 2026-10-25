#include "command/telemetry_protocol.h"
#include <stddef.h>
#include <string.h>
static uint16_t ReadU16(const uint8_t *Data)
{
    return (uint16_t)Data[0]|((uint16_t)Data[1]<<8);
}
static uint16_t CRC16(const uint8_t *Data, uint8_t Length)
{
    uint16_t CRC=0xffffU;
    for(uint8_t i=0;i<Length;i++)
    {
        CRC^=Data[i];
        for(uint8_t Bit=0;Bit<8;Bit++)CRC=(CRC&1U)?(CRC>>1)^0xa001U:CRC>>1;
    }
    return CRC;
}
uint8_t RemoteTelemetry_Decode(const uint8_t *Data, uint8_t Length,
                               RemoteTelemetry_FrameTypeDef *Frame)
{
    if(Data==NULL || Frame==NULL || Length!=REMOTE_TELEMETRY_FRAME_SIZE)return 0;
    if(Data[0]!=0xa5U || Data[1]!=REMOTE_TELEMETRY_VERSION)return 0;
    if(Data[3]&~(REMOTE_TELEMETRY_FLAG_POTS|REMOTE_TELEMETRY_FLAG_POT_MODE))return 0;
    if((Data[3]&REMOTE_TELEMETRY_FLAG_POT_MODE) &&
       !(Data[3]&REMOTE_TELEMETRY_FLAG_POTS))return 0;
    if(CRC16(Data, REMOTE_TELEMETRY_FRAME_SIZE-2U)!=ReadU16(Data+30))return 0;
    RemoteTelemetry_FrameTypeDef Decoded={
        .Header=Data[0],.Version=Data[1],.Sequence=Data[2],.Flags=Data[3],
        .Frame_CRC=ReadU16(Data+30)};
    for(unsigned i=0;i<REMOTE_TELEMETRY_CHANNELS;i++)
    {
        Decoded.AngleCdeg[i]=ReadU16(Data+4+i*2);
        Decoded.Raw[i]=ReadU16(Data+14+i*2);
        if(Decoded.AngleCdeg[i]>REMOTE_TELEMETRY_ANGLE_MAX)return 0;
        if(Decoded.Raw[i]>REMOTE_TELEMETRY_ADC_MAX)return 0;
    }
    memcpy(Decoded.Reserved, Data+24, sizeof(Decoded.Reserved));
    for(unsigned i=0;i<sizeof(Decoded.Reserved);i++)if(Decoded.Reserved[i])return 0;
    *Frame=Decoded;
    return 1;
}
uint8_t RemoteTelemetry_Encode(const RemoteTelemetry_FrameTypeDef *Frame,
                               uint8_t Output[REMOTE_TELEMETRY_FRAME_SIZE])
{
    if(Frame==NULL || Output==NULL)return 0;
    uint8_t Data[REMOTE_TELEMETRY_FRAME_SIZE]={0};
    Data[0]=Frame->Header;Data[1]=Frame->Version;
    Data[2]=Frame->Sequence;Data[3]=Frame->Flags;
    for(uint8_t i=0;i<REMOTE_TELEMETRY_CHANNELS;i++)
    {
        Data[4+i*2]=(uint8_t)Frame->AngleCdeg[i];Data[5+i*2]=(uint8_t)(Frame->AngleCdeg[i]>>8);
        Data[14+i*2]=(uint8_t)Frame->Raw[i];Data[15+i*2]=(uint8_t)(Frame->Raw[i]>>8);
    }
    memcpy(Data+24,Frame->Reserved,sizeof(Frame->Reserved));
    uint16_t CRC=CRC16(Data,REMOTE_TELEMETRY_FRAME_SIZE-2U);
    Data[30]=(uint8_t)CRC;Data[31]=(uint8_t)(CRC>>8);
    RemoteTelemetry_FrameTypeDef Check;
    if(!RemoteTelemetry_Decode(Data,sizeof(Data),&Check))return 0;
    memcpy(Output,Data,sizeof(Data));
    return 1;
}
