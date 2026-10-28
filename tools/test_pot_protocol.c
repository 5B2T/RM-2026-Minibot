#include "command/command_protocol.h"
#include "command/telemetry_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static RemoteControl_FrameTypeDef Frame(uint8_t flags) {
 RemoteControl_FrameTypeDef f={0};f.Header=0xa5;f.Version=3;f.Flags=flags;
 if(flags&4){f.Reserved[15]=0x1f;f.Reserved[16]=1;}return f;
}
int main(void) {
 uint8_t wire[32];RemoteControl_FrameTypeDef f=Frame(0),out;
 for(unsigned v=1;v<=3;v++){f.Version=v;assert(CommandProtocol_Encode(&f,wire));assert(CommandProtocol_Decode(wire,32,&out));}
 f=Frame(4);f.Reserved[5]=0x02;f.Reserved[6]=0x0d;
 assert(CommandProtocol_Encode(&f,wire));assert(wire[17]==2&&wire[18]==13);
 assert(CommandProtocol_Decode(wire,32,&out));assert(!memcmp(f.Reserved,out.Reserved,18));
 wire[17]^=1;assert(!CommandProtocol_Decode(wire,32,&out));
 for(unsigned i=5;i<18;i++){f=Frame(0);f.Reserved[i]=1;assert(!CommandProtocol_Encode(&f,wire));}
 const uint8_t badflags[]={5,6,7,0x84};
 for(unsigned i=0;i<sizeof(badflags);i++){f=Frame(badflags[i]);assert(!CommandProtocol_Encode(&f,wire));}
 for(unsigned v=1;v<=2;v++){f=Frame(4);f.Version=v;assert(!CommandProtocol_Encode(&f,wire));}
 f=Frame(4);f.Reserved[15]=15;assert(!CommandProtocol_Encode(&f,wire));
 f=Frame(4);f.Reserved[16]=2;assert(!CommandProtocol_Encode(&f,wire));
 f=Frame(4);f.Reserved[17]=1;assert(!CommandProtocol_Encode(&f,wire));
 f=Frame(1);f.M1=100;assert(CommandProtocol_Encode(&f,wire));
 f=Frame(2);f.M1=100;assert(!CommandProtocol_Encode(&f,wire));
 f=Frame(REMOTE_CONTROL_FLAG_STOP|REMOTE_CONTROL_FLAG_PASSIVE_STOP);
 assert(CommandProtocol_Encode(&f,wire) && CommandProtocol_Decode(wire,32,&out));
 assert(out.Flags==9);wire[3]^=8;assert(!CommandProtocol_Decode(wire,32,&out));
 const uint8_t BadPassive[]={8,10,11,12,13,14,15};
 for(unsigned i=0;i<sizeof BadPassive;i++){f=Frame(BadPassive[i]);assert(!CommandProtocol_Encode(&f,wire));}
 for(unsigned v=1;v<=2;v++){f=Frame(9);f.Version=v;assert(!CommandProtocol_Encode(&f,wire));}
 puts("PASS: passive STOP is CRC-protected V3 STOP only; invalid combinations rejected");
 puts("PASS V1/V2/V3, target 0.1 deg layout, reserved/mask/mode, CRC, STOP/RESUME");
 RemoteTelemetry_FrameTypeDef t={0},tele;
 t.Header=0xa5;t.Version=4;t.Flags=3;
 for(unsigned i=0;i<5;i++){t.AngleCdeg[i]=33300;t.Raw[i]=4095;}
 assert(RemoteTelemetry_Encode(&t,wire));assert(RemoteTelemetry_Decode(wire,32,&tele));
 assert(tele.AngleCdeg[4]==33300&&tele.Raw[4]==4095&&tele.Flags==3);
 wire[4]^=1;assert(!RemoteTelemetry_Decode(wire,32,&tele));
 t.Flags=2;assert(!RemoteTelemetry_Encode(&t,wire));
 t.Flags=5;assert(!RemoteTelemetry_Encode(&t,wire));
 t.Flags=3;t.AngleCdeg[0]=33301;assert(!RemoteTelemetry_Encode(&t,wire));
 t.AngleCdeg[0]=33300;t.Raw[0]=4096;assert(!RemoteTelemetry_Encode(&t,wire));
 t.Raw[0]=4095;t.Reserved[0]=1;memset(wire,0xcc,32);assert(!RemoteTelemetry_Encode(&t,wire));
 for(unsigned i=0;i<32;i++)assert(wire[i]==0xcc);
 puts("PASS POT_MODE telemetry, 333 deg range, flags/ADC/angle/CRC/reserved rejection");
 return 0;
}
