// Actual target mapper/protocol; remote mode and input-validity are test inputs.
#include "arm_targets.h"
#include "control/remote_control.h"
#include "command/telemetry_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
static Remote_Status Status;
Remote_Status Controller_GetStatus(void){return Status;}
static uint16_t Pots[5]={2048,2048,2048,2048,2048};
static const uint16_t Chord=KEY_MASK(KEY_R_PRESS);
static void Enter(void)
{
    ArmTargets_Init();Status.Mode=REMOTE_ACTIVE;Status.InputValid=1;
    ArmTargets_Update(Pots,1,0,0);ArmTargets_Update(Pots,1,Chord,300);
    assert(ArmTargets_GetStatus().Active);
}
static uint8_t Flags(void)
{
    RemoteControl_FrameTypeDef F={0};uint8_t Wire[32];
    F.Header=0xa5;F.Version=3;ArmTargets_Apply(&F);
    assert(CommandProtocol_Encode(&F,Wire));return F.Flags;
}
int main(void)
{
    // An unpowered input board must not suppress local ADC/angle diagnostics.
    Status.Mode=REMOTE_LOCKED;Status.InputValid=0;ArmTargets_Init();
    ArmTargets_Update(Pots,1,0,0);ArmTargets_Status Diag=ArmTargets_GetStatus();
    assert(Diag.ValidMask==0x1f && Diag.AngleCdeg[1]>500 && !Diag.Active && Flags()==0);
    int32_t Previous=Diag.AngleCdeg[1];Pots[1]=3000;
    for(unsigned i=1;i<=20;i++)ArmTargets_Update(Pots,1,0,i*20);
    Diag=ArmTargets_GetStatus();assert(Diag.Raw[1]==3000 && Diag.AngleCdeg[1]<Previous);
    assert(Diag.ValidMask==0x1f && Diag.EntryBlock==ARM_TARGET_ENTRY_INPUT && !Diag.Active && Flags()==0);
    Pots[1]=2048;

    // A short press while pots settle is remembered, but only for a bounded time.
    Status.Mode=REMOTE_ACTIVE;Status.InputValid=1;ArmTargets_Init();
    ArmTargets_Update(Pots,1,0,0);ArmTargets_Update(Pots,1,Chord,20);
    assert(!ArmTargets_GetStatus().Active && ArmTargets_GetStatus().EntryPending);
    assert(ArmTargets_GetStatus().EntryBlock==ARM_TARGET_ENTRY_SETTLING);
    ArmTargets_Update(Pots,1,0,40);ArmTargets_Update(Pots,1,0,320);
    assert(ArmTargets_GetStatus().Active && !ArmTargets_GetStatus().EntryPending);
    ArmTargets_Update(Pots,1,Chord,340);assert(!ArmTargets_GetStatus().Active);
    ArmTargets_Update(Pots,1,Chord,700);assert(!ArmTargets_GetStatus().Active); // Held: no repeats.
    // Old joint-selector chord is no longer an entry shortcut.
    ArmTargets_Update(Pots,1,KEY_MASK(KEY_R_UP)|KEY_MASK(KEY_R_DOWN),720);
    assert(!ArmTargets_GetStatus().Active);
    // Invalid ADC cannot enter; expiration cannot auto-enter on later recovery.
    Pots[2]=0;ArmTargets_Update(Pots,1,Chord,740);
    assert(ArmTargets_GetStatus().EntryPending && ArmTargets_GetStatus().EntryBlock==ARM_TARGET_ENTRY_ADC);
    ArmTargets_Update(Pots,1,0,3740);
    assert(!ArmTargets_GetStatus().Active && !ArmTargets_GetStatus().EntryPending);
    assert(ArmTargets_GetStatus().EntryBlock==ARM_TARGET_ENTRY_EXPIRED);
    Pots[2]=2048;ArmTargets_Update(Pots,1,0,3760);ArmTargets_Update(Pots,1,0,4100);
    assert(!ArmTargets_GetStatus().Active);
    // No entry queued through a lock; a new press is required after arming.
    Status.Mode=REMOTE_LOCKED;ArmTargets_Update(Pots,1,Chord,4120);
    assert(!ArmTargets_GetStatus().EntryPending && ArmTargets_GetStatus().EntryBlock==ARM_TARGET_ENTRY_LOCKED);
    Status.Mode=REMOTE_ACTIVE;ArmTargets_Update(Pots,1,Chord,4140);assert(!ArmTargets_GetStatus().Active);
    ArmTargets_Update(Pots,1,0,4160);ArmTargets_Update(Pots,1,Chord,4180);assert(ArmTargets_GetStatus().Active);
    // Conversion outage resets the settling interval, even at an early tick.
    ArmTargets_Init();ArmTargets_Update(Pots,0,0,100);
    ArmTargets_Update(Pots,1,Chord,120);assert(!ArmTargets_GetStatus().Active);
    Status.InputValid=0;ArmTargets_Update(Pots,1,0,140);
    assert(!ArmTargets_GetStatus().EntryPending && ArmTargets_GetStatus().EntryBlock==ARM_TARGET_ENTRY_INPUT);
    Enter();assert(Flags()==4);
    RemoteControl_FrameTypeDef HeldFrame={.Header=0xa5,.Version=3};ArmTargets_Apply(&HeldFrame);
    Status.InputValid=0;Pots[0]=2200;assert(Flags()==0); // Input loss suspends output while retaining mode.
    ArmTargets_Update(Pots,1,0,320);assert(ArmTargets_GetStatus().Active);
    RemoteControl_FrameTypeDef Frozen={.Header=0xa5,.Version=3};ArmTargets_Apply(&Frozen);
    assert(Frozen.Flags==0);
    assert(ArmTargets_GetStatus().Raw[0]==2200 && ArmTargets_GetStatus().ValidMask==0x1f);
    Pots[0]=2048;
    Status.InputValid=1;ArmTargets_Update(Pots,1,0,340);assert(ArmTargets_GetStatus().Active);
    Status.InputValid=0;ArmTargets_Update(Pots,1,0,360);ArmTargets_Update(Pots,1,0,560);
    assert(ArmTargets_GetStatus().Active && Flags()==0);
    Status.InputValid=1;ArmTargets_Update(Pots,1,0,580);ArmTargets_Update(Pots,1,0,700);
    assert(ArmTargets_GetStatus().Active && Flags()==4); // Automatic continuation with fresh inputs.
    ArmTargets_Update(Pots,1,0,720);assert(Flags()==4);
    ArmTargets_Update(Pots,0,0,740);assert(ArmTargets_GetStatus().Active && Flags()==0);
    ArmTargets_Update(Pots,0,0,940);assert(ArmTargets_GetStatus().Active && Flags()==0);
    ArmTargets_Update(Pots,1,0,960);assert(ArmTargets_GetStatus().Active && Flags()==4);
    Enter();Status.Mode=REMOTE_LOCKED;assert(Flags()==0);
    ArmTargets_Update(Pots,1,0,320);assert(!ArmTargets_GetStatus().Active);
    Enter();ArmTargets_Update(Pots,1,0,320);
    ArmTargets_Update(Pots,1,KEY_MASK(KEY_R_LEFT)|KEY_MASK(KEY_R_RIGHT),340);
    assert(!ArmTargets_GetStatus().Active && Flags()==0);
    // Signed V3 base coordinates, unsigned normalized V4 display, zero/outside ADC rejection.
    Pots[0]=1;Enter();ArmTargets_Status S=ArmTargets_GetStatus();
    assert(S.AngleCdeg[0]>16000 && S.AngleCdeg[0]<18000 && ArmTargets_TelemetryAngle(0)==33300);
    Pots[0]=4094;Enter();S=ArmTargets_GetStatus();
    assert(S.AngleCdeg[0]==-18000 && ArmTargets_TelemetryAngle(0)==0);
    RemoteControl_FrameTypeDef F={0},Decoded;uint8_t Wire[32];F.Header=0xa5;F.Version=3;
    ArmTargets_Apply(&F);assert(CommandProtocol_Encode(&F,Wire) && CommandProtocol_Decode(Wire,32,&Decoded));
    int16_t BaseWire=(int16_t)((uint16_t)Decoded.Reserved[5]|((uint16_t)Decoded.Reserved[6]<<8));
    assert(BaseWire==-1800);
    RemoteTelemetry_FrameTypeDef T={0},U;T.Header=0xa5;T.Version=4;T.Flags=3;
    for(unsigned i=0;i<5;i++){T.Raw[i]=Pots[i];T.AngleCdeg[i]=ArmTargets_TelemetryAngle((uint8_t)i);}
    assert(RemoteTelemetry_Encode(&T,Wire) && RemoteTelemetry_Decode(Wire,32,&U));
    assert(U.AngleCdeg[0]==0 && U.Raw[0]==4094);
    Pots[0]=3144;Enter();S=ArmTargets_GetStatus();
    assert(S.AngleCdeg[0]>=-10127 && S.AngleCdeg[0]<=-10123); // Controller CCW90 commands reversed vehicle coordinate.
    Pots[0]=1964;Enter();S=ArmTargets_GetStatus();assert(S.AngleCdeg[0]>=-1 && S.AngleCdeg[0]<=1);
    assert(ArmTargets_TelemetryAngle(0)>17200 && ArmTargets_TelemetryAngle(0)<17500);
    ArmTargets_Apply(&F);uint16_t Held=(uint16_t)F.Reserved[5]|((uint16_t)F.Reserved[6]<<8);
    Pots[0]=0;ArmTargets_Update(Pots,1,0,320);assert(ArmTargets_GetStatus().Active && Flags()==4);
    assert(!(ArmTargets_GetStatus().ValidMask&1));ArmTargets_Apply(&F);
    assert(((uint16_t)F.Reserved[5]|((uint16_t)F.Reserved[6]<<8))==Held);
    Pots[0]=2048;ArmTargets_Update(Pots,1,0,340);assert(ArmTargets_GetStatus().Active && Flags()==4);
    Pots[2]=0;ArmTargets_Update(Pots,1,0,360);assert(ArmTargets_GetStatus().Active && Flags()==4);
    Status.InputValid=0;ArmTargets_Update(Pots,1,0,380);ArmTargets_Update(Pots,1,0,580);
    assert(Flags()==0); // Persistent invalid input suspends output, retains selection.
    Pots[2]=2048;Status.InputValid=1;
    // Human-paired reference residuals, not independent measured servo accuracy.
    const struct {uint8_t Channel;uint16_t Adc;int32_t DegCdeg,Tolerance;} Cases[]={
        {0,1964,0,2},{0,3144,-10125,2},{0,2554,-5063,2},{0,784,10125,2},
        // Shoulder trial offset -20deg, same slope, clamp AFTER offset.
        {1,1354,7500,2},{1,2425,500,2},{1,1890,3000,6},
        {1,1,15000,0},{1,4094,500,0},
        {2,938,10300,100},{2,1837,18000,100},{2,1297,13500,100},
        {3,2023,19000,2},{3,3056,27000,2},{3,2714,24000,400},
        {4,2238,0,50},{4,3323,9000,50},{4,2776,4500,50}
    };
    for(unsigned i=0;i<sizeof Cases/sizeof Cases[0];i++)
    {
        Pots[Cases[i].Channel]=Cases[i].Adc;ArmTargets_Init();ArmTargets_Update(Pots,1,0,0);
        S=ArmTargets_GetStatus();assert(S.ValidMask&(1U<<Cases[i].Channel));
        assert(abs(S.AngleCdeg[Cases[i].Channel]-Cases[i].DegCdeg)<=Cases[i].Tolerance);
    }
    puts("PASS: pot100ms entry, bridge/ADC outage pauses and automatically continues, paired calibration residuals");
    puts("PASS: input-board loss keeps local ADC/angle diagnostics live without updating frozen motion targets");
    return 0;
}
