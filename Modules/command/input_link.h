#ifndef INPUT_LINK_H
#define INPUT_LINK_H
#include <stdint.h>
#define INPUT_LINK_ADDRESS 0x32U
#define INPUT_LINK_SIZE 20U
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
#endif
