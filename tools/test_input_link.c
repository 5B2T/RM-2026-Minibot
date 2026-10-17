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
 InputLink_Frame Frame={.Sequence=254,.Valid=1,.Keys=0x0801};
 for(unsigned i=0;i<5;i++)Frame.ADC[i]=4095-i;
 assert(InputLink_Encode(&Frame,Data));InputLink_Frame Decoded={0};
 assert(InputLink_Decode(Data,sizeof(Data),&Decoded)&&Decoded.ADC[4]==4091&&Decoded.Keys==0x0801);
 InputLink_Receiver Receiver={0};
 assert(InputLink_Receive(&Receiver,Data,sizeof(Data),100)==1);
 assert(InputLink_Read(&Receiver,129).Valid&&!InputLink_Read(&Receiver,130).Valid);
 assert(InputLink_Receive(&Receiver,Data,sizeof(Data),120)==0&&Receiver.TickMs==100);
 Frame.Sequence=253;assert(InputLink_Encode(&Frame,Data));assert(InputLink_Receive(&Receiver,Data,sizeof(Data),125)==0);
 Frame.Sequence=255;assert(InputLink_Encode(&Frame,Data));assert(InputLink_Receive(&Receiver,Data,sizeof(Data),140)==1);
 Frame.Sequence=0;assert(InputLink_Encode(&Frame,Data));assert(InputLink_Receive(&Receiver,Data,sizeof(Data),160)==1);
 Frame.Sequence=254;assert(InputLink_Encode(&Frame,Data));assert(InputLink_Receive(&Receiver,Data,sizeof(Data),260)==1);
 assert(InputLink_Receive(&Receiver,Data,sizeof(Data),300)==0&&Receiver.TickMs==260);
 Data[16]=1;InputLink_Put(Data+18,InputLink_CRC(Data,18));assert(!InputLink_Decode(Data,sizeof(Data),&Decoded));
 assert(InputLink_Receive(&Receiver,Data,sizeof(Data),310)==-1&&Receiver.TickMs==260);
 Frame.ADC[0]=4096;assert(!InputLink_Encode(&Frame,Data));Frame.ADC[0]=2048;
 Frame.Keys=0x8000;assert(!InputLink_Encode(&Frame,Data));Frame.Keys=1;
 Frame.Valid=2;assert(!InputLink_Encode(&Frame,Data));Frame.Valid=0;Frame.Sequence=255;
 assert(InputLink_Encode(&Frame,Data));assert(InputLink_Receive(&Receiver,Data,sizeof(Data),320)==1);
 assert(!InputLink_Read(&Receiver,321).Valid);
 Frame.Valid=1;Frame.Sequence=0;assert(InputLink_Encode(&Frame,Data));
 memset(&Receiver,0,sizeof(Receiver));assert(InputLink_Receive(&Receiver,Data,sizeof(Data),UINT32_MAX-10U)==1);
 assert(InputLink_Read(&Receiver,18).Valid&&!InputLink_Read(&Receiver,19).Valid);
 assert(InputLink_Read(&Receiver,19).Keys==1); // Expiry does not invent a physical release.
 assert(!InputLink_Decode(NULL,20,&Decoded)&&!InputLink_Decode(Data,19,&Decoded));
 puts("PASS: input-link encoder/decoder, CRC/endian/corruption/reserved/ranges, duplicates/stale/wrap, lease and reset resync, invalid input and preserved key state");
 return 0;
}
