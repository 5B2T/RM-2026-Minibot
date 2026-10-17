#ifndef INPUT_LINK_H
#define INPUT_LINK_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define INPUT_LINK_ADDRESS 0x32U
#define INPUT_LINK_SIZE 20U
#define INPUT_LINK_PERIOD_MS 20U
#define INPUT_LINK_STALE_MS 30U
#define INPUT_LINK_RESYNC_MS 100U
typedef struct
{
 uint16_t Keys,ADC[5];
 uint8_t Sequence,Valid;
} InputLink_Frame;
typedef struct
{
 InputLink_Frame Last;
 uint32_t TickMs;
 uint8_t HasFrame;
} InputLink_Receiver;
static inline uint16_t InputLink_CRC(const uint8_t *Data,uint8_t Length)
{
 uint16_t Check=0xffff;
 for(uint8_t t=0;t<Length;t++)
 {
  Check^=Data[t];
  for(uint8_t b=0;b<8;b++)Check=Check&1?(Check>>1)^0xa001:Check>>1;
 }
 return Check;
}
static inline void InputLink_Put(uint8_t *Data,uint16_t Value)
{Data[0]=(uint8_t)Value;Data[1]=(uint8_t)(Value>>8);}
static inline uint16_t InputLink_Get(const uint8_t *Data)
{return (uint16_t)(Data[0]|((uint16_t)Data[1]<<8));}
static inline uint8_t InputLink_Decode(const uint8_t *Data,uint8_t Length,InputLink_Frame *Frame)
{
 if(!Data || !Frame || Length!=INPUT_LINK_SIZE || Data[0]!=0xa6 || Data[1]!=1 ||
    Data[3]>1 || Data[16] || Data[17] || (InputLink_Get(Data+4)&0xf000U) ||
    InputLink_Get(Data+18)!=InputLink_CRC(Data,18))return 0;
 InputLink_Frame Next={.Keys=InputLink_Get(Data+4),.Sequence=Data[2],.Valid=Data[3]};
 for(unsigned i=0;i<5;i++){Next.ADC[i]=InputLink_Get(Data+6+i*2);if(Next.ADC[i]>4095)return 0;}
 *Frame=Next;return 1;
}
static inline uint8_t InputLink_Encode(const InputLink_Frame *Frame,uint8_t Data[INPUT_LINK_SIZE])
{
 if(!Frame || !Data)return 0;
 uint8_t Next[INPUT_LINK_SIZE]={0xa6,1,Frame->Sequence,Frame->Valid};
 InputLink_Put(Next+4,Frame->Keys);
 for(unsigned i=0;i<5;i++)InputLink_Put(Next+6+i*2,Frame->ADC[i]);
 InputLink_Put(Next+18,InputLink_CRC(Next,18));
 InputLink_Frame Check;if(!InputLink_Decode(Next,sizeof(Next),&Check))return 0;
 memcpy(Data,Next,sizeof(Next));return 1;
}
// -1 malformed, 0 stale/duplicate, 1 accepted. Bad frames never refresh the lease.
static inline int8_t InputLink_Receive(InputLink_Receiver *Receiver,const uint8_t *Data,uint8_t Length,uint32_t TickMs)
{
 InputLink_Frame Next;if(!Receiver || !InputLink_Decode(Data,Length,&Next))return -1;
 uint8_t Delta=(uint8_t)(Next.Sequence-Receiver->Last.Sequence);
 if(Receiver->HasFrame && (uint32_t)(TickMs-Receiver->TickMs)<INPUT_LINK_RESYNC_MS &&
    (Delta==0 || Delta>=128U))return 0;
 Receiver->Last=Next;Receiver->TickMs=TickMs;Receiver->HasFrame=1;return 1;
}
static inline InputLink_Frame InputLink_Read(const InputLink_Receiver *Receiver,uint32_t NowMs)
{
 InputLink_Frame Frame=Receiver->Last;
 Frame.Valid=Frame.Valid && Receiver->HasFrame && (uint32_t)(NowMs-Receiver->TickMs)<INPUT_LINK_STALE_MS;
 // Retain the last physical key state across expiry; do not invent a key release.
 return Frame;
}
#endif
