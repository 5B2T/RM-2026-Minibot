#include "command/arm_debug_protocol.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <ctype.h>

static uint8_t UInt(const char *Text,uint32_t Max,uint32_t *Value)
{
    if(*Text=='\0')return 0;
    uint32_t N=0;
    for(;*Text;Text++)
    {
        if(*Text<'0' || *Text>'9')return 0;
        uint32_t D=(uint32_t)(*Text-'0');
        if(D>Max || N>(Max-D)/10U)return 0;
        N=N*10U+D;
    }
    *Value=N;return 1;
}
static uint8_t Float(const char *Text,float *Value)
{
    // Decimal grammar only: reject hex floats, NaN, infinity and trailing junk.
    const char *P=Text;
    if(*P=='+' || *P=='-')P++;
    uint8_t Digits=0;
    while(isdigit((unsigned char)*P)) {P++;Digits=1;}
    if(*P=='.') {P++;while(isdigit((unsigned char)*P)) {P++;Digits=1;}}
    if(!Digits)return 0;
    if(*P=='e' || *P=='E')
    {
        P++;if(*P=='+' || *P=='-')P++;
        if(!isdigit((unsigned char)*P))return 0;
        while(isdigit((unsigned char)*P))P++;
    }
    if(*P)return 0;
    errno=0;char *End;float V=strtof(Text,&End);
    if(errno==ERANGE || *End || !isfinite(V))return 0;
    *Value=V;return 1;
}
uint8_t ArmDebugProtocol_Parse(const char *Line,ArmDebug_Command *Command)
{
    if(Line==NULL || Command==NULL)return 0;
    size_t Length=0;while(Length<ARM_DEBUG_LINE_SIZE && Line[Length])Length++;
    if(Length==0 || Length>=ARM_DEBUG_LINE_SIZE)return 0;
    char Buffer[ARM_DEBUG_LINE_SIZE];memcpy(Buffer,Line,Length+1);
    char *Tokens[6];unsigned Count=0;char *P=Buffer;
    while(*P)
    {
        while(*P==' ' || *P=='\t')P++;
        if(!*P)break;
        if(Count==6)return 0;
        Tokens[Count++]=P;
        while(*P && *P!=' ' && *P!='\t')
        {if((unsigned char)*P<32 || (unsigned char)*P>126)return 0;P++;}
        if(*P)*P++='\0';
    }
    if(Count<2 || strcmp(Tokens[0],"ARM"))return 0;
    ArmDebug_Command Result={0};
    const char *Verb=Tokens[1];
    const char *Names[]={"ENABLE","DISABLE","HB","STOP","STATUS"};
    for(unsigned i=0;i<5;i++)
        if(!strcmp(Verb,Names[i]))
        {if(Count!=2)return 0;Result.Operation=(ArmDebug_Operation)i;*Command=Result;return 1;}
    uint32_t Number;
    if(!strcmp(Verb,"INIT"))
    {
        if(Count!=2)return 0;
        Result.Operation=ARM_DEBUG_INITIALIZE;Result.DurationMs=1000;
    }
    else if(!strcmp(Verb,"ZERO"))
    {
        if(Count!=2)return 0;
        Result.Operation=ARM_DEBUG_ZERO;Result.DurationMs=1000;
    }
    else if(!strcmp(Verb,"STREAM"))
    {
        if(Count!=3 || !UInt(Tokens[2],1000,&Number) || (Number!=0 && Number<50))return 0;
        Result.Operation=ARM_DEBUG_STREAM;Result.DurationMs=Number;
    }
    else if(!strcmp(Verb,"PRESET"))
    {
        if(Count!=3 || !UInt(Tokens[2],5000,&Number) || Number==0)return 0;
        Result.Operation=ARM_DEBUG_PRESET;Result.DurationMs=Number;
    }
    else if(!strcmp(Verb,"NEUTRAL"))
    {
        if(Count!=4 || !UInt(Tokens[2],5,&Number))return 0;
        Result.Joint=(uint8_t)Number;
        if(!UInt(Tokens[3],2500,&Number) || Number<500)return 0;
        Result.Value=(float)Number;Result.DurationMs=1000;
        Result.Operation=ARM_DEBUG_NEUTRAL;
    }
    else if(!strcmp(Verb,"POS") || !strcmp(Verb,"RATE"))
    {
        if(Count!=5 || !UInt(Tokens[2],5,&Number))return 0;
        Result.Joint=(uint8_t)Number;
        if(!Float(Tokens[3],&Result.Value) || !UInt(Tokens[4],5000,&Number) || Number==0)return 0;
        Result.DurationMs=Number;
        Result.Operation=!strcmp(Verb,"POS")?ARM_DEBUG_POSITION:ARM_DEBUG_RATE;
        if(Result.Operation==ARM_DEBUG_RATE && (Result.Value<-1 || Result.Value>1))return 0;
        // Hardware/joint-specific travel limits are checked by Arm_CommandValid.
        if(Result.Operation==ARM_DEBUG_POSITION && Result.Value<0)return 0;
    }
    else return 0;
    *Command=Result;return 1;
}
