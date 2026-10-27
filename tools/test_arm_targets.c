// Actual target mapper/protocol; remote mode and input-validity are test inputs.
#include "arm_targets.h"
#include "control/remote_control.h"
#include <assert.h>
#include <stdio.h>
static Remote_Status Status;
Remote_Status Controller_GetStatus(void){return Status;}
static uint16_t Pots[5]={2048,2048,2048,2048,2048};
static const uint16_t Chord=KEY_MASK(KEY_R_UP)|KEY_MASK(KEY_R_DOWN);
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
    Enter();assert(Flags()==4);
    Status.InputValid=0;assert(Flags()==0); // Stops before the next 20ms ADC update.
    ArmTargets_Update(Pots,1,0,320);assert(!ArmTargets_GetStatus().Active);
    Status.InputValid=1;ArmTargets_Update(Pots,1,0,340);
    assert(!ArmTargets_GetStatus().Active && Flags()==0); // Recovery never auto-resumes.
    ArmTargets_Update(Pots,1,Chord,360);assert(Flags()==4);
    ArmTargets_Update(Pots,0,0,380);assert(!ArmTargets_GetStatus().Active && Flags()==0);
    Enter();Status.Mode=REMOTE_LOCKED;assert(Flags()==0);
    ArmTargets_Update(Pots,1,0,320);assert(!ArmTargets_GetStatus().Active);
    Enter();ArmTargets_Update(Pots,1,0,320);
    ArmTargets_Update(Pots,1,KEY_MASK(KEY_R_LEFT)|KEY_MASK(KEY_R_RIGHT),340);
    assert(!ArmTargets_GetStatus().Active && Flags()==0);
    puts("PASS: pot-mode input failure, no auto-resume, explicit re-entry, ADC/lock/chord exits");
    return 0;
}
