// Real robot scheduler + receiver + arm + servo mapping. Only hardware and
// unrelated components are mocked. Compile using tools/run_arm_follow_tests.ps1.
#include "robot/robot.h"
#include "robot/robot_config.h"
#include "components/command/remote_control.h"
#include "components/arm/arm.h"
#include "components/chassis/chassis.h"
#include "components/chassis/chassis_test.h"
#include "components/display/debug_display.h"
#include "components/power/power_monitor.h"
#include "arm/bsp_arm.h"
#include "radio/nrf24l01.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t Now;
static uint8_t Seq,Grip,Deploy,ApplyFailure;
static uint16_t Pulses[5];
static int Base;
uint32_t BSP_TimeMs(void){return Now;}
uint8_t BSP_Motor_Init(void){return 1;}
void BSP_Motor_Disable(void){}
uint8_t BSP_Encoder_Init(void){return 1;}
void NRF24L01_SPI_Init(void){}
NRF24L01_Result NRF24L01_RX_Mode(void){return NRF_OK;}
NRF24L01_Result NRF24L01_RxPacket(uint8_t *P){(void)P;return NRF_EMPTY;}
uint8_t BSP_I2C_Init(void){return 1;}
uint8_t BSP_Buttons_Read(void){return 0;}
void DebugDisplay_HandleKeys(uint8_t N,uint8_t B,uint32_t T){(void)N;(void)B;(void)T;}
DebugDisplay_State DebugDisplay_GetState(void){DebugDisplay_State S={0};return S;}
uint32_t DebugDisplay_Step(const Robot_Status *S,uint32_t T){(void)S;(void)T;return 20;}
void ChassisTest_View(uint8_t V,uint8_t O,uint32_t T){(void)V;(void)O;(void)T;}
void PowerMonitor_Step(uint32_t T){(void)T;}
uint8_t BSP_Arm_Init(const uint16_t P[5]){memcpy(Pulses,P,sizeof(Pulses));Base=0;return 1;}
uint8_t BSP_Arm_Apply(const uint16_t P[5],int B)
{if(ApplyFailure)return 0;memcpy(Pulses,P,sizeof(Pulses));Base=B;return 1;}
void BSP_Arm_Disable(void){memset(Pulses,0,sizeof(Pulses));Base=0;}
uint16_t BSP_Arm_EncoderCount(void){return 0;}
uint8_t Chassis_Init(void){return 1;}
void Chassis_ResetSampling(void){}
uint8_t Chassis_Measure(float Dt){(void)Dt;return 1;}
void Chassis_Control(const Robot_ChassisCommand *C,float Dt){(void)C;(void)Dt;}
void Chassis_Stop(void){}
void Chassis_GetFeedback(Robot_Status *S){(void)S;}
void ChassisTest_Init(void){}
void ChassisTest_Stop(void){}
uint8_t ChassisTest_Control(Robot_ChassisCommand *C,const Robot_Status *S,uint8_t V,
                           uint32_t T,uint32_t G,uint8_t L)
{(void)C;(void)S;(void)V;(void)T;(void)G;(void)L;return 0;}
uint8_t ChassisTest_TakeRelease(void){return 0;}
ChassisTest_Status ChassisTest_GetStatus(void){ChassisTest_Status S={0};return S;}
void DebugDisplay_Init(void){}
void PowerMonitor_Init(void){}
Robot_PowerStatus PowerMonitor_GetStatus(uint32_t T){(void)T;Robot_PowerStatus S={0};return S;}

static void Reset(uint32_t Tick)
{Now=Tick;Seq=Grip=Deploy=ApplyFailure=0;assert(Robot_Init());}
static void Step(void){Now+=10;Robot_ControlStep(.01f);}
static void Send(uint8_t Flags,const int16_t Angles[5],int16_t Rate,int16_t BaseRate,uint8_t Joint)
{
    RemoteControl_FrameTypeDef F={0};uint8_t Wire[32];
    F.Header=0xa5;F.Version=3;F.Sequence=Seq++;F.Flags=Flags;F.M4=BaseRate;
    F.Reserved[0]=Joint;F.Reserved[1]=(uint8_t)Rate;F.Reserved[2]=(uint8_t)((uint16_t)Rate>>8);
    F.Reserved[3]=Deploy;F.Reserved[4]=Grip;
    if(Flags&4)
    {
        for(unsigned i=0;i<5;i++)
        {uint16_t V=(uint16_t)Angles[i];F.Reserved[5+2*i]=(uint8_t)V;F.Reserved[6+2*i]=(uint8_t)(V>>8);}
        F.Reserved[15]=0x1f;F.Reserved[16]=1;
    }
    assert(CommandProtocol_Encode(&F,Wire));assert(RemoteControl_Process(Wire,32));
}
static void ArmRadio(void){Send(2,NULL,0,0,0);Step();}
static void Follow(const int16_t A[5]){Send(4,A,1000,1000,2);}
static void Near(float A,float B){assert(fabsf(A-B)<.002f);}
static void RunFollow(const int16_t A[5],unsigned Cycles)
{
    for(unsigned i=0;i<Cycles;i++)
    {
        Arm_Status Before=Arm_GetStatus();
        if(i%2==0)Follow(A);
        Step();Arm_Status After=Arm_GetStatus();
        for(unsigned j=0;j<5;j++)
            assert(fabsf(After.CommandedAngleDeg[j]-Before.CommandedAngleDeg[j])<=.902f);
    }
}

static void LimitsAndGripper(void)
{
    Reset(0);for(unsigned i=0;i<5;i++)assert(Pulses[i]==0);
    const int16_t High[5]={3330,3000,3000,3330,3330};
    Follow(High);Step();assert(Arm_GetStatus().Action!=ARM_FOLLOW); // Still locked.
    ArmRadio();Follow(High);Step();Arm_Status S=Arm_GetStatus();
    assert(S.Action==ARM_FOLLOW && S.PositionEnabledMask==0x0f && Base==0 && !Pulses[4]);
    Near(S.CommandedAngleDeg[0],10.9f);Near(S.CommandedAngleDeg[1],23.9f);
    Near(S.CommandedAngleDeg[2],22.9f);Near(S.CommandedAngleDeg[3],180);
    // No new frame this 10ms cycle: mode persists, sample timestamp does not advance.
    uint32_t Received=RemoteControl_TakeArmSnapshot().Targets.TickMs;
    Step();assert(Arm_GetStatus().Action==ARM_FOLLOW);
    assert(RemoteControl_TakeArmSnapshot().Targets.TickMs==Received);
    RunFollow(High,400);S=Arm_GetStatus();
    Near(S.CommandedAngleDeg[0],150);Near(S.CommandedAngleDeg[1],180);
    Near(S.CommandedAngleDeg[2],270);Near(S.CommandedAngleDeg[3],180);
    assert(Base==0 && !Pulses[4]);
    Grip++;RunFollow(High,60);Near(Arm_GetStatus().CommandedAngleDeg[4],30);
    RunFollow(High,60);Near(Arm_GetStatus().CommandedAngleDeg[4],30); // Repeated counter does not toggle.
    Grip++;RunFollow(High,60);Near(Arm_GetStatus().CommandedAngleDeg[4],6);
    const int16_t Low[5]={-1000,-100,-100,-100,-100};RunFollow(Low,400);S=Arm_GetStatus();
    Near(S.CommandedAngleDeg[0],10);Near(S.CommandedAngleDeg[1],23);
    Near(S.CommandedAngleDeg[2],22);Near(S.CommandedAngleDeg[3],0);
    assert(Base==0 && S.SequencePose==ARM_SEQUENCE_UNKNOWN);
    puts("PASS: locked/boot, parallel slew, cached mode, limits, base stopped, gripper dedup");
}
static void ArbitrationAndStop(void)
{
    Reset(0);ArmRadio();const int16_t A[5]={1800,900,1030,2700,0};
    assert(Robot_ArmSubmit(&(Arm_Command){ARM_POSITION,ARM_SHOULDER,150,1000}));
    Follow(A);Step();Near(Arm_GetStatus().CommandedAngleDeg[0],10.9f); // Queued serial loses ownership.
    RemoteTelemetry_FrameTypeDef Tele={0};uint8_t Wire[32];
    Tele.Header=0xa5;Tele.Version=4;Tele.Flags=3;Tele.AngleCdeg[0]=33300;
    assert(RemoteTelemetry_Encode(&Tele,Wire));assert(RemoteControl_Process(Wire,32));
    Step();assert(Arm_GetStatus().Action==ARM_FOLLOW); // V4 is not a mode-exit frame.
    Follow(A);Step();assert(!Robot_ArmSubmit(&(Arm_Command){ARM_POSITION,ARM_SHOULDER,90,1000}));
    float Before=Arm_GetStatus().CommandedAngleDeg[0];
    Send(0,NULL,1000,1000,0);Step(); // Mode-exit barrier precedes joystick branch.
    Near(Arm_GetStatus().CommandedAngleDeg[0],Before);assert(Base==0);
    Step();Near(Arm_GetStatus().CommandedAngleDeg[0],Before+.9f);assert(Base>0 && Base<=300);
    Follow(A);Step();assert(Base==0 && Arm_GetStatus().Action==ARM_FOLLOW);
    Before=Arm_GetStatus().CommandedAngleDeg[0];
    uint8_t Next=Seq;Seq=0;Send(1,NULL,0,0,0);Seq=Next;Step(); // Stale STOP must defeat current targets.
    assert(!Arm_GetStatus().Active && Base==0);Near(Arm_GetStatus().CommandedAngleDeg[0],Before);
    Step();assert(!Arm_GetStatus().Active);
    // Re-arm alone must not replay the cached targets or old gripper event.
    ArmRadio();Step();assert(Arm_GetStatus().Action!=ARM_FOLLOW);
    Follow(A);Step();RemoteControl_Disconnect();Step();assert(!Arm_GetStatus().Active && Base==0);
    Reset(0);ArmRadio();Follow(A);Step();
    for(unsigned i=0;i<10;i++){Step();}
    assert(!Arm_GetStatus().Active);
    assert(!RemoteControl_GetLinkStatus().Armed);
    puts("PASS: serial arbitration, stopped exit cycle, joystick return, STOP/re-arm/disconnect/timeout");
}
static void SequenceAndFaults(void)
{
    Reset(0);Arm_Command Init={ARM_INITIALIZE,ARM_SHOULDER,0,5000};
    assert(Arm_Request(&Init,Now,Now));for(unsigned i=0;i<500;i++)Step();
    assert(Arm_GetStatus().SequencePose==ARM_SEQUENCE_DEPLOYED);
    assert(Arm_Request(&Init,Now,Now));unsigned Last=2;uint8_t Seen=0;
    for(unsigned i=0;i<800;i++)
    {
        Step();Arm_Status S=Arm_GetStatus();
        if(S.SequenceRunning && S.SequenceStage!=Last)
        {assert((Last==2 && S.SequenceStage==1)||(Last==1 && S.SequenceStage==3)||(Last==3 && S.SequenceStage==0));Last=S.SequenceStage;Seen++;}
    }
    assert(Seen==3 && Arm_GetStatus().SequencePose==ARM_SEQUENCE_ZERO);
    assert(Arm_Request(&Init,Now,Now));ArmRadio();const int16_t A[5]={0,900,1030,2700,0};
    Follow(A);Step();assert(!Arm_GetStatus().SequenceRunning && Arm_GetStatus().Action==ARM_FOLLOW);
    Deploy++;Follow(A);Step();assert(Arm_GetStatus().Action==ARM_FOLLOW); // INIT ignored in pot mode.
    float Bad[5]={0,90,NAN,90,0};assert(!Arm_FollowTargets(Bad,0,Now,Now));
    assert(!Arm_GetStatus().Active && Arm_GetStatus().Fault==ARM_INVALID_COMMAND);
    Reset(0);float Good[5]={0,90,103,270,0};assert(!Arm_FollowTargets(Good,0,0,100));
    assert(Arm_GetStatus().Fault==ARM_EXPIRED);
    Reset(0);assert(Arm_FollowTargets(Good,0,Now,Now));ApplyFailure=1;Arm_Step(.01f,Now+10);
    assert(!Arm_GetStatus().Ready && Arm_GetStatus().Fault==ARM_HARDWARE_ERROR);
    for(unsigned i=0;i<5;i++)assert(!Pulses[i]);
    Reset(UINT32_MAX-25U);ArmRadio();RunFollow(A,10);assert(Arm_GetStatus().Action==ARM_FOLLOW);
    for(unsigned i=0;i<10;i++){Step();}
    assert(!Arm_GetStatus().Active);
    puts("PASS: original deploy/fold order, sequence takeover, invalid/stale/fault stop, tick wrap");
}
static void BaseLeaseAndRecovery(void)
{
    Reset(0);Arm_Command Rate={ARM_RATE,ARM_BASE,1,5000};
    assert(Robot_ArmSubmit(&Rate));Step();Step();int Before=Base;
    assert(Robot_ArmSubmit(&Rate));Step();assert(Base>Before); // Repeated host frame keeps ramp/PI.
    for(unsigned i=0;i<100;i++){Step();}
    assert(Arm_GetStatus().Fault==ARM_BASE_FEEDBACK_ERROR && Base==0 && Arm_GetStatus().Ready);
    assert(Robot_ArmSubmit(&Rate));Step();assert(Base==0); // No automatic fault retry.
    Arm_Command Zero={ARM_ZERO,ARM_BASE,0,1000};
    assert(Robot_ArmSubmit(&Zero));Step();assert(Arm_GetStatus().Fault==ARM_OK && Base==0);
    assert(Robot_ArmSubmit(&Rate));Step();assert(Base>0);
    Robot_ArmStop();Step();assert(Base==0);
    puts("PASS: repeated base RATE preserves PI, feedback fault latches, ZERO recovers, STOP clears drive");
}
int main(void){LimitsAndGripper();ArbitrationAndStop();SequenceAndFaults();BaseLeaseAndRecovery();return 0;}
