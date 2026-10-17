#include "command/command_protocol.h"
#include <stddef.h>
#include <string.h>
static uint16_t ReadU16(const uint8_t *Data)
{
    return (uint16_t)Data[0]|((uint16_t)Data[1]<<8);
}
static int16_t ReadI16(const uint8_t *Data)
{
    int32_t Value=ReadU16(Data);
    if(Value>=0x8000L)Value-=0x10000L;
    return (int16_t)Value;
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
uint8_t CommandProtocol_Decode(const uint8_t *Data, uint8_t Length,
                               RemoteControl_FrameTypeDef *Frame)
{
    if(Data==NULL || Frame==NULL || Length!=REMOTE_CONTROL_FRAME_SIZE)return 0;
    if(Data[0]!=0xa5U || (Data[1]!=REMOTE_CONTROL_VERSION_WHEELS &&
                         Data[1]!=REMOTE_CONTROL_VERSION_VELOCITY &&
                         Data[1]!=REMOTE_CONTROL_VERSION_ROBOT))return 0;
    if(Data[3]&~(REMOTE_CONTROL_FLAG_STOP|REMOTE_CONTROL_FLAG_RESUME))return 0;
    if(CRC16(Data, REMOTE_CONTROL_FRAME_SIZE-2U)!=ReadU16(Data+30))return 0;
    RemoteControl_FrameTypeDef Decoded={
        .Header=Data[0],.Version=Data[1],.Sequence=Data[2],.Flags=Data[3],
        .M1=ReadI16(Data+4),.M2=ReadI16(Data+6),
        .M3=ReadI16(Data+8),.M4=ReadI16(Data+10),.Frame_CRC=ReadU16(Data+30)};
    memcpy(Decoded.Reserved, Data+12, sizeof(Decoded.Reserved));
    if(Decoded.Version==REMOTE_CONTROL_VERSION_ROBOT)
    {
        int16_t Rate=ReadI16(Decoded.Reserved+1);
        if(Decoded.Reserved[0]>3 || Rate<-1000 || Rate>1000 ||
           Decoded.M4<-1000 || Decoded.M4>1000)return 0;
        for(unsigned i=5;i<sizeof(Decoded.Reserved);i++)if(Decoded.Reserved[i])return 0;
        if(!(Decoded.Flags&REMOTE_CONTROL_FLAG_STOP) &&
           (Decoded.Flags&REMOTE_CONTROL_FLAG_RESUME) && Rate)return 0;
    }
    if(!(Decoded.Flags&REMOTE_CONTROL_FLAG_STOP) &&
       (Decoded.Flags&REMOTE_CONTROL_FLAG_RESUME) &&
       (Decoded.M1 || Decoded.M2 || Decoded.M3 || Decoded.M4))return 0;
    if(!(Decoded.Flags&REMOTE_CONTROL_FLAG_STOP) &&
       Decoded.Version==REMOTE_CONTROL_VERSION_VELOCITY && Decoded.M4!=0)return 0;
    *Frame=Decoded;
    return 1;
}
uint8_t CommandProtocol_Encode(const RemoteControl_FrameTypeDef *Frame,
                               uint8_t Output[REMOTE_CONTROL_FRAME_SIZE])
{
    if(Frame==NULL || Output==NULL)return 0;
    uint8_t Data[REMOTE_CONTROL_FRAME_SIZE]={0};
    Data[0]=Frame->Header;Data[1]=Frame->Version;
    Data[2]=Frame->Sequence;Data[3]=Frame->Flags;
    const int16_t Values[]={Frame->M1,Frame->M2,Frame->M3,Frame->M4};
    for(uint8_t i=0;i<4;i++)
    {
        uint16_t Value=(uint16_t)Values[i];
        Data[4+i*2]=(uint8_t)Value;Data[5+i*2]=(uint8_t)(Value>>8);
    }
    memcpy(Data+12,Frame->Reserved,sizeof(Frame->Reserved));
    uint16_t CRC=CRC16(Data,30);
    Data[30]=(uint8_t)CRC;Data[31]=(uint8_t)(CRC>>8);
    RemoteControl_FrameTypeDef Check;
    if(!CommandProtocol_Decode(Data,sizeof(Data),&Check))return 0;
    memcpy(Output,Data,sizeof(Data));
    return 1;
}
