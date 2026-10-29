// Exercise the real input bridge protocol, remote debounce/arming and pot entry together.
#include "command/input_link.h"
#include "control/remote_control.h"
#include "arm_targets.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static InputLink_Receiver Receiver;
static uint8_t Sequence;
static uint8_t BridgeValid=1;
static uint16_t Pots[5]={1964,1054,938,2023,2238};
static uint16_t InputAdc[5]={2048,2048,2048,2048,2048};
static void Feed(uint16_t Keys,uint32_t Now)
{
    InputLink_Frame F={.Sequence=Sequence++,.Valid=BridgeValid,.Keys=Keys};
    memcpy(F.ADC,InputAdc,sizeof InputAdc);
    uint8_t Wire[INPUT_LINK_SIZE];assert(InputLink_Encode(&F,Wire));
    assert(InputLink_Receive(&Receiver,Wire,sizeof Wire,Now)==1);
    InputLink_Frame Read=InputLink_Read(&Receiver,Now);
    Remote_Input Input={.Keys=Read.Keys,.Valid=Read.Valid,.TickMs=Now};
    memcpy(Input.ADC,Read.ADC,sizeof Input.ADC);
    Controller_OnTx(0,1,Now);
    Controller_Update(&Input,Now);
    ArmTargets_Update(Pots,1,Controller_GetStatus().Keys,Now);
}
int main(void)
{
    Controller_Init();ArmTargets_Init();
    for(uint32_t t=0;t<=800;t+=20)Feed(0,t);
    assert(Controller_GetStatus().Calibrated);
    // A mode press while locked cannot become a delayed automatic entry.
    Feed(KEY_MASK(KEY_R_PRESS),820);Feed(KEY_MASK(KEY_R_PRESS),860);
    assert(Controller_GetStatus().Keys&KEY_MASK(KEY_R_PRESS));
    assert(!ArmTargets_GetStatus().Active && !ArmTargets_GetStatus().EntryPending);
    Feed(0,880);Feed(0,920);
    Feed(KEY_MASK(KEY_L_UP),940);Feed(KEY_MASK(KEY_L_UP),980);
    assert(Controller_GetStatus().Mode==REMOTE_RESUMING);
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,980);
    assert(Controller_GetStatus().Mode==REMOTE_ACTIVE);
    Feed(0,1000);Feed(0,1040);
    Feed(KEY_MASK(KEY_R_PRESS),1060);Feed(KEY_MASK(KEY_R_PRESS),1100);
    assert(ArmTargets_GetStatus().Active);
    Feed(KEY_MASK(KEY_R_PRESS),1140);assert(ArmTargets_GetStatus().Active);
    Feed(0,1160);Feed(0,1200);
    Feed(KEY_MASK(KEY_R_PRESS),1220);Feed(KEY_MASK(KEY_R_PRESS),1260);
    assert(!ArmTargets_GetStatus().Active);
    Feed(0,1280);Feed(0,1320);
    Feed(KEY_MASK(KEY_R_PRESS),1340);Feed(KEY_MASK(KEY_R_PRESS),1380);
    assert(ArmTargets_GetStatus().Active);
    Feed(KEY_MASK(KEY_L_DOWN),1400); // STOP bypasses debounce, clears mode immediately.
    assert(Controller_GetStatus().Mode==REMOTE_LOCKED && !ArmTargets_GetStatus().Active);
    // A long upstream input outage keeps the ACTIVE session alive with neutral commands.
    memset(&Receiver,0,sizeof Receiver);Sequence=0;Controller_Init();ArmTargets_Init();
    for(uint32_t t=0;t<=800;t+=20)Feed(0,t);
    Feed(KEY_MASK(KEY_L_UP),820);Feed(KEY_MASK(KEY_L_UP),860);
    assert(Controller_GetStatus().Mode==REMOTE_RESUMING);
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,860);
    Feed(0,880);Feed(0,920);Feed(KEY_MASK(KEY_DEPLOY),940);Feed(KEY_MASK(KEY_DEPLOY),980);
    assert(Controller_GetStatus().DeployCounter==1);
    BridgeValid=0;
    for(uint32_t t=1000;t<=6000;t+=20)Feed(KEY_MASK(KEY_GRIPPER),t);
    Remote_Status S=Controller_GetStatus();RemoteControl_FrameTypeDef Command;
    assert(S.Mode==REMOTE_ACTIVE && !S.InputValid && S.LockCount==0);
    Controller_Command(&Command,0);
    assert(Command.Flags==0 && !Command.M1 && !Command.M2 && !Command.M3 && !Command.M4);
    assert(!Command.Reserved[1] && !Command.Reserved[2] && Command.Reserved[3]==1 && !Command.Reserved[4]);
    BridgeValid=1;Feed(KEY_MASK(KEY_GRIPPER),6020);Feed(KEY_MASK(KEY_GRIPPER),6060);
    assert(Controller_GetStatus().GripperCounter==0); // Held key across recovery is not a new event.
    Feed(0,6080);Feed(0,6120);Feed(KEY_MASK(KEY_GRIPPER),6140);Feed(KEY_MASK(KEY_GRIPPER),6180);
    assert(Controller_GetStatus().GripperCounter==1);
    BridgeValid=0;Feed(KEY_MASK(KEY_L_DOWN),6200);
    assert(Controller_GetStatus().Mode==REMOTE_LOCKED && Controller_GetStatus().LastLockReason==REMOTE_LOCK_STOP_KEY);
    // Invalid input must not finish an enable handshake, and radio failures recover automatically after authorization.
    BridgeValid=1;Feed(0,6220);Feed(0,6260);Feed(KEY_MASK(KEY_L_UP),6280);Feed(KEY_MASK(KEY_L_UP),6320);
    assert(Controller_GetStatus().Mode==REMOTE_RESUMING);
    BridgeValid=0;Feed(0,6340);
    assert(Controller_GetStatus().Mode==REMOTE_LOCKED && Controller_GetStatus().LastLockReason==REMOTE_LOCK_INPUT);
    BridgeValid=1;Feed(0,6360);Feed(0,6400);Feed(KEY_MASK(KEY_L_UP),6420);Feed(KEY_MASK(KEY_L_UP),6460);
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,6460);assert(Controller_GetStatus().Mode==REMOTE_ACTIVE);
    Controller_OnTx(0,0,6480);assert(Controller_GetStatus().Mode==REMOTE_ACTIVE);
    Controller_OnTx(0,1,6500);assert(Controller_GetStatus().Mode==REMOTE_ACTIVE);
    Controller_OnTx(0,0,6520);Controller_OnTx(0,0,6540);assert(Controller_GetStatus().Mode==REMOTE_ACTIVE);
    Controller_OnTx(0,0,6560);
    assert(Controller_GetStatus().Mode==REMOTE_ACTIVE && Controller_GetStatus().Recovering && Controller_GetStatus().LastLockReason==REMOTE_LOCK_TX_FAILED);
    Controller_Command(&Command,1);assert(Command.Flags==REMOTE_CONTROL_FLAG_RESUME && !Command.M1 && !Command.M4);
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,6570);
    assert(Controller_GetStatus().Mode==REMOTE_ACTIVE && !Controller_GetStatus().Recovering);
    // An80ms ACK gap preserves the selected POT mode and recovers without a key.
    Feed(0,6571);Feed(KEY_MASK(KEY_R_PRESS),6600);Feed(KEY_MASK(KEY_R_PRESS),6640);
    assert(ArmTargets_GetStatus().Active);
    Remote_Input I={.Valid=1};for(unsigned i=0;i<5;i++)I.ADC[i]=2048;
    Controller_Update(&I,6720);ArmTargets_Update(Pots,1,Controller_GetStatus().Keys,6720);
    assert(Controller_GetStatus().Recovering && ArmTargets_GetStatus().Active);
    Controller_Command(&Command,2);ArmTargets_Apply(&Command);assert(Command.Flags==REMOTE_CONTROL_FLAG_RESUME);
    Controller_OnTx(0,1,6730);assert(Controller_GetStatus().Recovering); // Late motion ACK cannot authorize recovery.
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,6740);Feed(0,6760);
    Controller_Command(&Command,3);ArmTargets_Apply(&Command);assert(Command.Flags==REMOTE_CONTROL_FLAG_ARM_TARGETS);
    // STOP pressed after RF lock must not inherit the passive reason, even though
    // the diagnostic LastLockReason deliberately preserves the first lock cause.
    Feed(KEY_MASK(KEY_L_DOWN),6780);Controller_Command(&Command,2);
    assert(Command.Flags==REMOTE_CONTROL_FLAG_STOP && Controller_GetStatus().LastLockReason==REMOTE_LOCK_STOP_KEY);
    Feed(0,6800);Feed(0,6840);Controller_Command(&Command,3);assert(Command.Flags==REMOTE_CONTROL_FLAG_STOP);
    Feed(KEY_MASK(KEY_L_UP),6860);Feed(KEY_MASK(KEY_L_UP),6900);
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,6900);Controller_Command(&Command,4);assert(Command.Flags==0);
    Controller_OnTx(0,0,6920);Controller_OnTx(0,0,6940);Controller_OnTx(0,0,6960);
    assert(Controller_GetStatus().Recovering);
    Feed(KEY_MASK(KEY_L_DOWN),6980);Controller_Command(&Command,5);
    assert(Command.Flags==REMOTE_CONTROL_FLAG_STOP && !Controller_GetStatus().Recovering);
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,7000);
    assert(Controller_GetStatus().Mode==REMOTE_LOCKED); // A delayed recovery ACK cannot override STOP.
    Controller_Lock();Controller_Command(&Command,5);assert(Command.Flags==REMOTE_CONTROL_FLAG_STOP);
    // Independently exercise physical left-stick directions and turn keys.
    Controller_Init();ArmTargets_Init();memset(&Receiver,0,sizeof Receiver);Sequence=0;
    for(uint32_t t=0;t<=800;t+=20)Feed(0,t);
    Feed(KEY_MASK(KEY_L_UP),820);Feed(KEY_MASK(KEY_L_UP),860);
    Controller_OnTx(REMOTE_CONTROL_FLAG_RESUME,1,860);Feed(0,880);Feed(0,920);
    InputAdc[0]=1000;Feed(0,940);Controller_Command(&Command,1);
    assert(Command.M1>0 && Command.M2==0 && Command.M3==0); // Forward.
    InputAdc[0]=3000;Feed(0,960);Controller_Command(&Command,2);
    assert(Command.M1<0 && Command.M2==0 && Command.M3==0); // Backward.
    InputAdc[0]=2048;InputAdc[1]=1000;Feed(0,980);Controller_Command(&Command,3);
    assert(Command.M1==0 && Command.M2>0 && Command.M3==0); // Left translation, no yaw.
    InputAdc[1]=3000;Feed(0,1000);Controller_Command(&Command,4);
    assert(Command.M1==0 && Command.M2<0 && Command.M3==0); // Right translation.
    InputAdc[0]=1000;InputAdc[1]=1000;Feed(0,1020);Controller_Command(&Command,5);
    assert(Command.M1>0 && Command.M2>0 && Command.M3==0);
    assert((int32_t)Command.M1*Command.M1+(int32_t)Command.M2*Command.M2<=541*541);
    InputAdc[0]=InputAdc[1]=2048;
    Feed(KEY_MASK(KEY_L_LEFT),1040);Feed(KEY_MASK(KEY_L_LEFT),1080);Controller_Command(&Command,6);
    assert(Command.M1==0 && Command.M2==0 && Command.M3>0); // Left button CCW.
    Feed(0,1100);Feed(0,1140);Feed(KEY_MASK(KEY_L_RIGHT),1160);Feed(KEY_MASK(KEY_L_RIGHT),1200);
    Controller_Command(&Command,7);assert(Command.M1==0 && Command.M2==0 && Command.M3<0);
    puts("PASS: physical stick up/down/left/right/diagonal translate only; left/right keys CCW/CW only");
    puts("PASS: automatic RF resume retains POT selection; operator STOP still requires fresh enable");
    puts("PASS: input-link right press, debounce, locked refusal, enable/ACK, toggle/hold, STOP");
    puts("PASS: 5s input outage neutralizes without locking, no stale actions on recovery, STOP/enable/radio interlocks retained");
    return 0;
}
