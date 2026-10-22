#include "components/host/arm_debug.h"
#include "command/arm_debug_protocol.h"
#include "uart/bsp_debug_uart.h"
#include "robot/robot.h"
#include "robot/robot_config.h"
#include "components/arm/arm.h"
#include "time/bsp_time.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#define REPLY_COUNT 8U
#define REPLY_SIZE 64U
static char Line[ARM_DEBUG_LINE_SIZE],Replies[REPLY_COUNT][REPLY_SIZE];
static unsigned LineLength,ReplyHead,ReplyTail,ReplyUsed;
static uint8_t Initialized,Enabled,Discard,StatusPending,SawCR;
static uint32_t LastContact,StreamTick,StreamMs=100,Faults,Commands,Errors,LineTick;

static void StopSession(void)
{
    Enabled=0;Robot_ArmStop();
}
static void Reply(const char *Text)
{
    if(ReplyUsed==REPLY_COUNT)
    {
        // Congestion must not silently accept motion whose acknowledgement is lost.
        StopSession();ReplyHead=ReplyTail=ReplyUsed=0;
        Text="ERR TX_OVERFLOW\r\n";
    }
    if(!strncmp(Text,"ERR ",4))Errors++;
    (void)snprintf(Replies[ReplyHead],REPLY_SIZE,"%s",Text);
    ReplyHead=(ReplyHead+1U)%REPLY_COUNT;ReplyUsed++;
}
static void Reject(const char *Reason)
{
    StopSession();Reply(Reason);
}
static void Handle(const ArmDebug_Command *Input,uint32_t Now)
{
    Commands++;
    switch(Input->Operation)
    {
    case ARM_DEBUG_ENABLE:
    {
        Robot_Status Status;Robot_GetStatus(&Status);
        if(!Status.Arm.Ready) {Reject("ERR NOT_READY\r\n");return;}
        Enabled=1;LastContact=Now;Reply("OK ENABLE\r\n");return;
    }
    case ARM_DEBUG_DISABLE:StopSession();Reply("OK DISABLE\r\n");return;
    case ARM_DEBUG_STOP:Robot_ArmStop();Reply("OK STOP\r\n");return;
    case ARM_DEBUG_STATUS:StatusPending=1;Reply("OK STATUS\r\n");return;
    case ARM_DEBUG_STREAM:
        StreamMs=Input->DurationMs;StreamTick=Now;
        Reply("OK STREAM\r\n");return;
    case ARM_DEBUG_HEARTBEAT:
        if(!Enabled) {Reply("ERR DISABLED\r\n");return;}
        LastContact=Now;return; // Periodic heartbeat does not flood the output.
    default:break;
    }
    if(!Enabled) {Reply("ERR DISABLED\r\n");return;}
    if(Input->Operation==ARM_DEBUG_INITIALIZE && !ROBOT_ARM_POSES_CALIBRATED)
    {Reject("ERR CALIBRATION\r\n");return;}
    if(Input->Operation==ARM_DEBUG_NEUTRAL || Input->Operation==ARM_DEBUG_PRESET)
    {Reject("ERR UNSUPPORTED\r\n");return;} // Legacy continuous neutral / all-five preset.
    Arm_Command Command={0};
    Command.Joint=(Arm_Joint)Input->Joint;Command.Value=Input->Value;
    Command.DurationMs=Input->DurationMs;
    switch(Input->Operation)
    {
    case ARM_DEBUG_POSITION:Command.Action=ARM_POSITION;break;
    case ARM_DEBUG_RATE:Command.Action=ARM_RATE;break;
    case ARM_DEBUG_PRESET:Command.Action=ARM_PRESET;break;
    case ARM_DEBUG_NEUTRAL:Command.Action=ARM_NEUTRAL;break;
    case ARM_DEBUG_ZERO:Command.Action=ARM_ZERO;break;
    case ARM_DEBUG_INITIALIZE:Command.Action=ARM_INITIALIZE;break;
    default:Reject("ERR SYNTAX\r\n");return;
    }
    if(!Arm_CommandValid(&Command)) {Reject("ERR RANGE\r\n");return;}
    if(!Robot_ArmSubmit(&Command)) {Robot_ArmStop();Reply("ERR BUSY\r\n");return;}
    LastContact=Now;Reply("OK QUEUED\r\n");
}
static long Milli(float Value)
{
    double Scaled=(double)Value*1000.0;
    if(!isfinite(Scaled))return 0;
    if(Scaled>INT32_MAX)return INT32_MAX;
    if(Scaled<INT32_MIN)return INT32_MIN;
    return (long)Scaled;
}
static void SendStatus(uint32_t Now)
{
    Robot_Status Robot;Robot_GetStatus(&Robot);Arm_Status *S=&Robot.Arm;
    char Output[BSP_DEBUG_UART_TX_SIZE];
    // ARMDBG V2: 25 integer CSV channels. Slots 10/11 now report J2/J3 angles.
    // snprintf(newlib-nano) has no reliable 64-bit conversion: encode counts as decimal ourselves.
    char Counts[24];uint64_t Magnitude=S->BaseCounts<0?(uint64_t)(-(S->BaseCounts+1))+1U:(uint64_t)S->BaseCounts;
    unsigned Index=sizeof(Counts)-1U;Counts[Index]='\0';
    do {Counts[--Index]=(char)('0'+Magnitude%10U);Magnitude/=10U;}while(Magnitude);
    if(S->BaseCounts<0)Counts[--Index]='-';
    int N=snprintf(Output,sizeof(Output),
        "%lu,%u,%u,%u,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u,%s,%ld,%ld,%d,%lu,%lu,%lu,%u\r\n",
        (unsigned long)S->TickMs,(unsigned)Enabled,(unsigned)S->Ready,(unsigned)S->Active,
        (unsigned)S->Fault,(unsigned)S->SelectedJoint,(unsigned)S->Action,
        Milli(S->CommandedAngleDeg[0]),Milli(S->CommandedAngleDeg[3]),Milli(S->CommandedAngleDeg[4]),
        Milli(S->CommandedAngleDeg[1]),Milli(S->CommandedAngleDeg[2]),(unsigned)S->PulseUs[0],(unsigned)S->PulseUs[1],
        (unsigned)S->PulseUs[2],(unsigned)S->PulseUs[3],(unsigned)S->PulseUs[4],Counts+Index,
        Milli(S->BaseRelativeAngleDeg),Milli(S->BaseRpm),(int)S->BasePWM,
        (unsigned long)Faults,(unsigned long)Commands,(unsigned long)Errors,(unsigned)S->PositionEnabledMask);
    if(N>0 && (unsigned)N<sizeof(Output) && BSP_DebugUart_Write(Output,(uint16_t)N))
    {StreamTick=Now;StatusPending=0;}
}
uint32_t ArmDebug_Step(void)
{
    uint32_t Now=BSP_TimeMs();
    if(!Initialized)
    {
        if(!BSP_DebugUart_Init())return 100U;
        Initialized=1;Faults=BSP_DebugUart_Faults();StreamTick=Now;
        Reply("HELLO ARMDBG V2\r\n");
    }
    BSP_DebugUart_Service();
    uint32_t NewFaults=BSP_DebugUart_Faults();
    if(NewFaults!=Faults)
    {
        Faults=NewFaults;LineLength=0;Discard=1;SawCR=0;BSP_DebugUart_FlushRx();
        Reject("ERR UART\r\n");
    }
    if(Enabled && (uint32_t)(Now-LastContact)>=ROBOT_ARM_DEBUG_TIMEOUT_MS)
    {StopSession();Reply("ERR TIMEOUT\r\n");}
    if(LineLength && (uint32_t)(Now-LineTick)>=ROBOT_ARM_DEBUG_TIMEOUT_MS)
    {LineLength=0;Discard=1;Reject("ERR LINE_TIMEOUT\r\n");}
    // Bound work so a noisy host cannot monopolize a task or starve control.
    uint8_t Byte;
    for(unsigned i=0;i<128U && BSP_DebugUart_Read(&Byte);i++)
    {
        if(Byte=='\n')
        {
            if(!Discard && LineLength)
            {
                Line[LineLength]='\0';ArmDebug_Command Command;
                if(ArmDebugProtocol_Parse(Line,&Command))Handle(&Command,Now);
                else Reject("ERR SYNTAX\r\n");
            }
            LineLength=0;Discard=0;SawCR=0;
        }
        else if(!Discard)
        {
            if(Byte=='\r' && !SawCR) {SawCR=1;continue;}
            if(SawCR || Byte==0 || Byte>126 || (Byte<32 && Byte!='\t') || LineLength>=sizeof(Line)-1U)
            {LineLength=0;Discard=1;Reject("ERR LINE\r\n");}
            else {if(!LineLength)LineTick=Now;Line[LineLength++]=(char)Byte;}
        }
    }
    if(ReplyUsed)
    {
        char *Text=Replies[ReplyTail];
        if(BSP_DebugUart_Write(Text,(uint16_t)strlen(Text)))
        {ReplyTail=(ReplyTail+1U)%REPLY_COUNT;ReplyUsed--;}
    }
    else if(StatusPending || (StreamMs && (uint32_t)(Now-StreamTick)>=StreamMs))SendStatus(Now);
    return 5U;
}
