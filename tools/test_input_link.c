#include "command/input_link.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
 const uint8_t Check[]="123456789";
 assert(InputLink_CRC(Check,9)==0x4b37);
 uint8_t Data[INPUT_LINK_SIZE]={0xa6,1,255,1};
 InputLink_Put(Data+4,0x0801);
 for(uint8_t t=0;t<5;t++)InputLink_Put(Data+6+t*2,4095-t);
 InputLink_Put(Data+18,InputLink_CRC(Data,18));
 assert(InputLink_Get(Data+4)==0x0801);
 assert(InputLink_Get(Data+14)==4091);
 assert(InputLink_Get(Data+18)==InputLink_CRC(Data,18));
 Data[8]^=1;
 assert(InputLink_Get(Data+18)!=InputLink_CRC(Data,18));
 puts("input link CRC/endian/corruption tests passed");
 return 0;
}
