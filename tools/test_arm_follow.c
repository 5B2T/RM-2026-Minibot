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
#include "command/arm_debug_protocol.h"
#include "components/host/arm_debug.h"
#include "radio/nrf24l01.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t Now;
static uint8_t Seq,Grip,Deploy,ApplyFailure;
static uint16_t Pulses[5];
static int Base;
static uint16_t Encoder;
static Robot_ChassisCommand LastChassis;
static const char *UartInput="";
static uint32_t UartErrors;
static uint8_t TxWritable=1;
uint8_t BSP_DebugUart_Init(void){return 1;}
void BSP_DebugUart_Service(void){}
uint8_t BSP_DebugUart_Read(uint8_t *B){if(!*UartInput)return 0;*B=(uint8_t)*UartInput++;return 1;}
uint8_t BSP_DebugUart_Write(const char *D,uint16_t N){(void)D;(void)N;return TxWritable;}
uint32_t BSP_DebugUart_Faults(void){return UartErrors;}
void BSP_DebugUart_FlushRx(void){UartInput="";}
static void Host(const char *Text){UartInput=Text;ArmDebug_Step();}

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
uint16_t BSP_Arm_EncoderCount(void){return Encoder;}
uint8_t Chassis_Init(void){return 1;}
void Chassis_ResetSampling(void){}
uint8_t Chassis_Measure(float Dt){(void)Dt;return 1;}
void Chassis_Control(const Robot_ChassisCommand *C,float Dt){LastChassis=*C;(void)Dt;}
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
{Now=Tick;Seq=Grip=Deploy=ApplyFailure=0;Encoder=0;assert(Robot_Init());}
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
    const int16_t High[5]={0,3000,3000,3330,3330};
    Follow(High);Step();assert(Arm_GetStatus().Action!=ARM_FOLLOW); // Still locked.
    ArmRadio();Follow(High);Step();Arm_Status S=Arm_GetStatus();
    assert(S.Action==ARM_FOLLOW && S.PositionEnabledMask==0x0f && Base==0 && !Pulses[4]);
    Near(S.CommandedAngleDeg[0],5.9f);Near(S.CommandedAngleDeg[1],23.9f);
    Near(S.CommandedAngleDeg[2],22.9f);Near(S.CommandedAngleDeg[3],90.9f);
    // No new frame this 10ms cycle: mode persists, sample timestamp does not advance.
    uint32_t Received=RemoteControl_TakeArmSnapshot().Targets.TickMs;
    Step();assert(Arm_GetStatus().Action==ARM_FOLLOW);
    assert(RemoteControl_TakeArmSnapshot().Targets.TickMs==Received);
    RunFollow(High,400);S=Arm_GetStatus();
    Near(S.CommandedAngleDeg[0],150);Near(S.CommandedAngleDeg[1],180);
    Near(S.CommandedAngleDeg[2],270);Near(S.CommandedAngleDeg[3],180);
    assert(Base==0 && !Pulses[4]);
    Grip++;RunFollow(High,60);Near(Arm_GetStatus().CommandedAngleDeg[4],45);
    RunFollow(High,60);Near(Arm_GetStatus().CommandedAngleDeg[4],45); // Repeated counter does not toggle.
    Grip++;RunFollow(High,60);Near(Arm_GetStatus().CommandedAngleDeg[4],6);
    const int16_t Low[5]={0,-100,-100,-100,-100};RunFollow(Low,400);S=Arm_GetStatus();
    Near(S.CommandedAngleDeg[0],5);Near(S.CommandedAngleDeg[1],23);
    Near(S.CommandedAngleDeg[2],22);Near(S.CommandedAngleDeg[3],0);
    assert(Base==0 && S.SequencePose==ARM_SEQUENCE_UNKNOWN);
    puts("PASS: locked/boot, parallel slew, cached mode, limits, base stopped, gripper dedup");
}
static void ArbitrationAndStop(void)
{
    Reset(0);ArmRadio();const int16_t A[5]={1800,900,1030,2700,0};
    assert(Robot_ArmSubmit(&(Arm_Command){ARM_POSITION,ARM_SHOULDER,150,1000}));
    Follow(A);Step();Near(Arm_GetStatus().CommandedAngleDeg[0],5.9f); // Queued serial loses ownership.
    RemoteTelemetry_FrameTypeDef Tele={0};uint8_t Wire[32];
    Tele.Header=0xa5;Tele.Version=4;Tele.Flags=3;Tele.AngleCdeg[0]=33300;
    assert(RemoteTelemetry_Encode(&Tele,Wire));assert(RemoteControl_Process(Wire,32));
    Step();assert(Arm_GetStatus().Action==ARM_FOLLOW); // V4 is not a mode-exit frame.
    Follow(A);Step();assert(!Robot_ArmSubmit(&(Arm_Command){ARM_POSITION,ARM_SHOULDER,90,1000}));
    float Before=Arm_GetStatus().CommandedAngleDeg[0];
    Send(0,NULL,1000,1000,0);Step(); // Mode-exit barrier precedes joystick branch.
    Near(Arm_GetStatus().CommandedAngleDeg[0],Before);assert(Base==0);
    Step();Near(Arm_GetStatus().CommandedAngleDeg[0],Before+.9f);assert(Base>0 && Base<=300);
    Follow(A);Step();assert(Base<=ROBOT_ARM_BASE_PWM_LIMIT && Arm_GetStatus().Action==ARM_FOLLOW);
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
    puts("PASS: deploy/fold order, sequence takeover, invalid/stale/fault stop, tick wrap");
}
static void DeployForFold(void)
{
    Arm_Command Init={ARM_INITIALIZE,ARM_SHOULDER,0,5000};
    assert(Arm_Request(&Init,Now,Now));unsigned Last=0,Changes=0;
    for(unsigned i=0;i<500;i++)
    {
        Step();Arm_Status S=Arm_GetStatus();
        if(S.SequenceRunning && S.SequenceStage!=Last)
        {
            assert((Last==0 && S.SequenceStage==255)||(Last==255 && S.SequenceStage==4));
            Last=S.SequenceStage;Changes++;
        }
        if(S.SequenceStage==0){Near(S.CommandedAngleDeg[1],23);Near(S.CommandedAngleDeg[2],22);Near(S.CommandedAngleDeg[3],90);}
        if(S.SequenceStage==255)Near(S.CommandedAngleDeg[4],6);
    }
    Arm_Status S=Arm_GetStatus();assert(Changes==2 && S.SequencePose==ARM_SEQUENCE_DEPLOYED);
    const float Expected[5]={95,103,190,0,45};
    for(unsigned i=0;i<5;i++)Near(S.CommandedAngleDeg[i],Expected[i]);
}
static void FoldBaseAndWrist(void)
{
    Reset(0);DeployForFold();
    // Existing manual wrist jog while deployed creates a nonzero first-stage clear move.
    for(unsigned i=0;i<40;i++){Arm_RemoteInput(0,ARM_WRIST_ROLL,1,0,0,Now);Step();}
    Arm_Stop();Near(Arm_GetStatus().CommandedAngleDeg[3],36);
    for(unsigned i=0;i<55;i++){Encoder=(uint16_t)(Encoder-10);Step();}
    for(unsigned i=0;i<100;i++)Step();
    Near(Arm_GetStatus().BaseRelativeAngleDeg,30);
    uint32_t Zero=Arm_GetStatus().ZeroGeneration;
    Arm_Command Init={ARM_INITIALIZE,ARM_SHOULDER,0,5000};assert(Arm_Request(&Init,Now,Now));
    for(unsigned i=0;i<250;i++)
    {
        Encoder++;Step();Arm_Status S=Arm_GetStatus();
        assert(S.SequenceRunning && S.SequenceStage==2 && S.Fault==ARM_OK);
        Near(S.CommandedAngleDeg[0],95);Near(S.CommandedAngleDeg[1],103);
        assert(Base<=0); // Return to the original boot zero; never require pot pickup.
        if(i==10){assert(Arm_Request(&Init,Now,Now));assert(S.BasePositionPickedUp);}
    }
    Arm_Status S=Arm_GetStatus();Near(S.CommandedAngleDeg[2],22);
    Near(S.CommandedAngleDeg[3],0);Near(S.CommandedAngleDeg[4],6);
    assert(S.BaseCounts==300 && S.ZeroGeneration==Zero); // Base has not arrived: no next stage.
    for(unsigned i=0;i<30;i++){Encoder=(uint16_t)(Encoder+10);Step();}
    unsigned Last=2,Changes=0;
    for(unsigned i=0;i<600;i++)
    {
        Step();S=Arm_GetStatus();
        if(S.SequenceRunning && S.SequenceStage!=Last)
        {
            assert((Last==2 && S.SequenceStage==1)||(Last==1 && S.SequenceStage==3)||(Last==3 && S.SequenceStage==0));
            Last=S.SequenceStage;Changes++;
        }
        if(S.SequenceStage==1){Near(S.CommandedAngleDeg[3],0);Near(S.CommandedAngleDeg[0],95);assert(Base==0);}
        if(S.SequenceStage==3){Near(S.CommandedAngleDeg[1],23);Near(S.CommandedAngleDeg[0],95);}
    }
    const float Folded[5]={5,23,22,90,6};
    assert(Changes==3 && S.SequencePose==ARM_SEQUENCE_ZERO && !S.Active && !Base);
    assert(S.BaseCounts==0 && S.ZeroGeneration==Zero);
    for(unsigned i=0;i<5;i++)Near(S.CommandedAngleDeg[i],Folded[i]);
    // An encoder that moves too slowly must time out; it must not declare success by redefining zero.
    Reset(0);DeployForFold();
    for(unsigned i=0;i<330;i++){Encoder=(uint16_t)(Encoder-10);Step();}
    for(unsigned i=0;i<100;i++)Step();
    assert(Arm_Request(&Init,Now,Now));
    for(unsigned i=0;i<1250;i++){Encoder++;Step();if(!Arm_GetStatus().SequenceRunning)break;}
    S=Arm_GetStatus();assert(S.Fault==ARM_EXPIRED && S.SequencePose==ARM_SEQUENCE_UNKNOWN && !Base);
    assert(S.BaseCounts>2000 && S.ZeroGeneration==1 && !S.BasePositionEnabled);
    // Missing encoder feedback and explicit STOP both cancel the whole first stage.
    Reset(0);DeployForFold();
    for(unsigned i=0;i<55;i++){Encoder=(uint16_t)(Encoder-10);Step();}
    for(unsigned i=0;i<100;i++)Step();
    assert(Arm_Request(&Init,Now,Now));for(unsigned i=0;i<150;i++){Step();}
    S=Arm_GetStatus();assert(S.Fault==ARM_BASE_FEEDBACK_ERROR && !S.SequenceRunning && !Base);
    Reset(0);DeployForFold();assert(Arm_Request(&Init,Now,Now));Step();Arm_Stop();
    S=Arm_GetStatus();assert(!S.SequenceRunning && !S.BasePositionEnabled && !Base && S.SequencePose==ARM_SEQUENCE_UNKNOWN);
    puts("PASS: new deploy targets, fold wrist0/base0 barrier then elbow/wrist90/shoulder5, original zero, timeout/feedback/STOP");
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
static void FoldArrivalHysteresis(void)
{
    Reset(0);DeployForFold();
    // A long wrist-clear move gives the base time to enter hold, then drift slightly.
    for(unsigned i=0;i<200;i++){Arm_RemoteInput(0,ARM_WRIST_ROLL,1,0,0,Now);Step();}
    Arm_Stop();Near(Arm_GetStatus().CommandedAngleDeg[3],180);
    Arm_Command Init={ARM_INITIALIZE,ARM_SHOULDER,0,5000};assert(Arm_Request(&Init,Now,Now));
    Step();assert(Arm_GetStatus().BasePositionAtTarget);
    for(unsigned i=0;i<45;i++)
    {
        if(i%5==0)Encoder--; // 0.18RPM, then stationary at +0.49deg: inside0.7deg hold band.
        Step();assert(Base==0 && Arm_GetStatus().BasePositionAtTarget);
    }
    for(unsigned i=0;i<650;i++)Step();
    Arm_Status S=Arm_GetStatus();
    assert(S.BaseRelativeAngleDeg>.3f && S.BaseRelativeAngleDeg<.7f);
    assert(!S.SequenceRunning && S.SequencePose==ARM_SEQUENCE_ZERO && S.Fault==ARM_OK);
    assert(S.BaseCounts==9 && S.ZeroGeneration==1 && !Base);
    puts("PASS: base entered0.3deg arrival then settled inside0.7deg hold band does not deadlock fold");
}
static void SequenceWithNeutralRadio(void)
{
    Reset(0);ArmRadio();Deploy++;Send(0,NULL,0,0,0);Step();
    assert(Arm_GetStatus().SequenceRunning);
    // This is the neutral V3 command retained by the relay while its input bridge is invalid.
    for(unsigned i=0;i<500;i++){if(i%2==0)Send(0,NULL,0,0,0);Step();}
    assert(Arm_GetStatus().SequencePose==ARM_SEQUENCE_DEPLOYED && Arm_GetStatus().Fault==ARM_OK);
    Deploy++;Send(0,NULL,0,0,0);Step();assert(Arm_GetStatus().SequenceReturning);
    for(unsigned i=0;i<650;i++){if(i%2==0)Send(0,NULL,0,0,0);Step();}
    assert(Arm_GetStatus().SequencePose==ARM_SEQUENCE_ZERO && Arm_GetStatus().Fault==ARM_OK);
    Deploy++;Send(0,NULL,0,0,0);Step();Send(1,NULL,0,0,0);Step();
    assert(!Arm_GetStatus().SequenceRunning && Arm_GetStatus().SequencePose==ARM_SEQUENCE_UNKNOWN && !Base);
    puts("PASS: neutral radio stream preserves deploy/fold, explicit STOP still cancels");
}
static void FoldFromOutsidePositionLimits(void)
{
    for(int Sign=-1;Sign<=1;Sign+=2)
    {
        Reset(0);DeployForFold();
        // Manual speed control may leave the base just outside the position target range.
        for(unsigned i=0;i<332;i++)
        {Encoder=(uint16_t)(Encoder+Sign*10*ROBOT_ARM_BASE_ENCODER_SIGN);Step();}
        for(unsigned i=0;i<100;i++)Step();
        assert(fabsf(Arm_GetStatus().BaseRelativeAngleDeg)>180);
        Arm_Command Init={ARM_INITIALIZE,ARM_SHOULDER,0,5000};
        assert(Arm_Request(&Init,Now,Now));
        assert(Arm_GetStatus().SequenceReturning && Arm_GetStatus().BasePositionTargetDeg==0);
        for(unsigned i=0;i<332;i++)
        {
            Encoder=(uint16_t)(Encoder-Sign*10*ROBOT_ARM_BASE_ENCODER_SIGN);Step();
            assert(Arm_GetStatus().Fault==ARM_OK && Base*Sign<=0); // Only inward PWM.
        }
        for(unsigned i=0;i<500;i++)Step();
        Arm_Status S=Arm_GetStatus();
        assert(S.SequencePose==ARM_SEQUENCE_ZERO && S.Fault==ARM_OK && !Base);
        assert(S.BaseCounts==0 && S.ZeroGeneration==1);
        Arm_Command Bad={ARM_POSITION,ARM_BASE,181,1000};
        assert(!Arm_CommandValid(&Bad)); // Ordinary target bounds are unchanged.
    }
    puts("PASS: fold from either side beyond180deg returns inward to original zero; target limits unchanged");
}
static void FollowExitAndExplicitPresets(void)
{
    Reset(0);ArmRadio();const int16_t A[5]={0,600,1350,2400,450};
    RunFollow(A,300);assert(Arm_GetStatus().SequencePose==ARM_SEQUENCE_UNKNOWN);
    Send(0,NULL,0,0,0);Step();assert(!Arm_GetStatus().Active); // Keep the mode-exit stop barrier.
    Deploy++;Send(0,NULL,0,0,0);Step();
    assert(Arm_GetStatus().SequenceRunning && Arm_GetStatus().SequenceReturning);
    for(unsigned i=0;i<800;i++){if(i%2==0)Send(0,NULL,0,0,0);Step();}
    assert(Arm_GetStatus().SequencePose==ARM_SEQUENCE_ZERO && Arm_GetStatus().Fault==ARM_OK);
    // The mode-exit packet itself may also carry the new SW6 counter.
    Reset(0);ArmRadio();RunFollow(A,50);Deploy++;Send(0,NULL,0,0,0);Step();
    assert(!Arm_GetStatus().SequenceRunning);Step();
    assert(Arm_GetStatus().SequenceRunning && Arm_GetStatus().SequenceReturning);
    // Deferred event is canceled by STOP, disconnect, expiry or return to pot mode.
    Reset(0);ArmRadio();RunFollow(A,50);Deploy++;Send(0,NULL,0,0,0);Step();
    Send(1,NULL,0,0,0);Step();ArmRadio();Step();assert(!Arm_GetStatus().SequenceRunning);
    Reset(0);ArmRadio();RunFollow(A,50);Deploy++;Send(0,NULL,0,0,0);Step();
    RemoteControl_Disconnect();Step();ArmRadio();Step();assert(!Arm_GetStatus().SequenceRunning);
    Reset(0);ArmRadio();RunFollow(A,50);Deploy++;Send(0,NULL,0,0,0);Step();
    Now+=ROBOT_REMOTE_TIMEOUT_MS;Step();assert(!Arm_GetStatus().SequenceRunning);
    Reset(0);ArmRadio();RunFollow(A,50);Deploy++;Send(0,NULL,0,0,0);Step();
    Follow(A);Step();Send(0,NULL,0,0,0);Step();Step();assert(!Arm_GetStatus().SequenceRunning);
    Arm_Stop();
    assert(Arm_GoToPose(ARM_POSE_EXTENDED,Now));
    uint8_t Stage=Arm_GetStatus().SequenceStage;
    assert(!Arm_GoToPose(ARM_POSE_USER1,Now));assert(!Arm_GoToPose(ARM_POSE_USER2,Now));
    assert(Arm_GetStatus().SequenceRunning && Arm_GetStatus().SequenceStage==Stage);
    for(unsigned i=0;i<500;i++){if(i%2==0)Send(0,NULL,0,0,0);Step();}
    assert(Arm_GetStatus().SequencePose==ARM_SEQUENCE_DEPLOYED);
    assert(Arm_GoToPose(ARM_POSE_FOLDED,Now));assert(Arm_GetStatus().SequenceReturning);
    Send(1,NULL,0,0,0);Step();assert(!Arm_GetStatus().SequenceRunning && !Base);
    // Position servos have no multi-degree software deadband: a0.2deg command updates PWM.
    Reset(0);ArmRadio();int16_t Small[5]={0,50,230,220,900};Follow(Small);Step();
    for(unsigned j=0;j<4;j++)
    {
        Arm_Status Before=Arm_GetStatus();Small[j+1]+=2;Follow(Small);Step();Arm_Status After=Arm_GetStatus();
        Near(After.CommandedAngleDeg[j],Before.CommandedAngleDeg[j]+.2f);
        assert(After.PulseUs[j]!=Before.PulseUs[j]);
    }
    puts("PASS: exit teaching then SW6 folds from tracked pose, explicit preset API/user slots, STOP, servo0.2deg PWM response");
}
static void BasePositionIntegration(void)
{
    ArmDebug_Command Parsed;
    assert(ArmDebugProtocol_Parse("ARM CLEAR",&Parsed) && Parsed.Operation==ARM_DEBUG_CLEAR);
    assert(!ArmDebugProtocol_Parse("ARM CLEAR 1",&Parsed));
    assert(ArmDebugProtocol_Parse("ARM POS 5 -90 1000",&Parsed) && Parsed.Value==-90);
    assert(!ArmDebugProtocol_Parse("ARM POS 0 -90 1000",&Parsed));
    Reset(0);Arm_Command Position={ARM_POSITION,ARM_BASE,-90,5000};
    assert(Arm_CommandValid(&Position));Position.Value=181;assert(!Arm_CommandValid(&Position));Position.Value=-90;
    assert(Arm_Request(&Position,Now,Now));for(unsigned i=0;i<5;i++)Step();
    Arm_Status S=Arm_GetStatus();assert(S.BasePositionEnabled && S.BasePositionPickedUp && Base<0);
    float Ref=S.BasePositionReferenceDeg;
    assert(Arm_Request(&Position,Now,Now));Step();assert(Arm_GetStatus().BasePositionReferenceDeg<Ref);
    Arm_Command Zero={ARM_ZERO,ARM_BASE,0,1000};assert(!Arm_CommandValid(&Zero));
    Arm_Stop();Step();assert(Arm_CommandValid(&Zero));assert(Arm_Request(&Zero,Now,Now));
    assert(Arm_GetStatus().BaseCounts==0 && !Arm_GetStatus().BasePositionEnabled);
    Reset(0);ArmRadio();int16_t A[5]={900,100,230,220,1800};
    RunFollow(A,8);S=Arm_GetStatus();assert(Base>0 && S.BasePositionEnabled && S.BasePositionPickedUp);
    A[0]=0;Follow(A);Step();assert(Arm_GetStatus().BasePositionPickedUp);
    A[0]=-900;RunFollow(A,25);S=Arm_GetStatus();assert(Base<0 && S.BasePositionTargetDeg==-90);
    assert(RemoteControl_TakeArmSnapshot().Targets.AngleDeg[0]==-90);
    Send(1,NULL,0,0,0);Step();assert(Base==0 && !Arm_GetStatus().BasePositionEnabled);
    ArmRadio();A[0]=-900;RunFollow(A,4);assert(Base<0 && Arm_GetStatus().BasePositionPickedUp);
    A[0]=0;Follow(A);Step();assert(Arm_GetStatus().BasePositionPickedUp);
    RemoteControl_Disconnect();Step();assert(Base==0 && !Arm_GetStatus().BasePositionEnabled);
    Reset(0);Encoder=65530;Arm_ResetSampling();Encoder=5;Step();
    assert(Arm_GetStatus().BaseCounts==11*ROBOT_ARM_BASE_ENCODER_SIGN);
    assert(!Arm_CommandValid(&Zero)); // Moving feedback forbids redefining the zero.
    for(unsigned i=0;i<50;i++)Step();
    Arm_Command Rate={ARM_RATE,ARM_BASE,1,5000};assert(Arm_Request(&Rate,Now,Now));
    for(unsigned i=0;i<100;i++)Step();
    assert(Arm_GetStatus().Fault==ARM_BASE_FEEDBACK_ERROR);
    int64_t Counts=Arm_GetStatus().BaseCounts;uint32_t Generation=Arm_GetStatus().ZeroGeneration;
    Arm_Command Clear={ARM_CLEAR,ARM_BASE,0,1000};assert(Arm_Request(&Clear,Now,Now));
    assert(Arm_GetStatus().Fault==ARM_OK && Arm_GetStatus().BaseCounts==Counts &&
        Arm_GetStatus().ZeroGeneration==Generation && !Arm_GetStatus().Active && Base==0);
    puts("PASS: signed BASE POS/parser, lease renewal, stopped ZERO, radio pickup/STOP/re-pickup/loss, encoder wrap");
}
static void HostIsolationAndRemoteRecovery(void)
{
    Reset(10000);Step();Host("ARM ENABLE\n");Host("ARM POS 0 90 1000\n");Step();
    assert(Arm_GetStatus().Active);
    // Queue a debug-session stop, then let radio take control before the next tick.
    Host("BAD\n");ArmRadio();Deploy++;Send(0,NULL,0,0,0);Step();
    assert(Arm_GetStatus().SequenceRunning);
    Host("ARM ENABLE\n");Host("ARM POS 0 120 1000\n");Step();
    assert(Arm_GetStatus().SequenceRunning); // Busy/invalid commands cannot cancel radio.
    Host("ARM POS 0 9000 1000\n");Step();assert(Arm_GetStatus().SequenceRunning);
    UartErrors++;ArmDebug_Step();Host("\n");Step();assert(Arm_GetStatus().SequenceRunning);
    TxWritable=0;Host("ARM STATUS\nARM STATUS\nARM STATUS\nARM STATUS\nARM STATUS\nARM STATUS\nARM STATUS\nARM STATUS\nARM STATUS\nARM STATUS\n");
    ArmDebug_Step();Step();assert(Arm_GetStatus().SequenceRunning);TxWritable=1;
    for(unsigned i=0;i<35;i++){Send(0,NULL,0,0,0);Step();ArmDebug_Step();}
    assert(Arm_GetStatus().SequenceRunning);
    Host("ARM STOP\n");Step();assert(!Arm_GetStatus().Active); // Explicit stop is still global.
    // A session enabled before radio takeover may subsequently time out harmlessly.
    Reset(15000);Step();Host("ARM ENABLE\n");Host("ARM POS 0 90 1000\n");Step();
    ArmRadio();Deploy++;Send(0,NULL,0,0,0);Step();
    for(unsigned i=0;i<35;i++){Send(0,NULL,0,0,0);Step();ArmDebug_Step();}
    assert(Arm_GetStatus().SequenceRunning);
    // A host-owned motion still stops on its own malformed command/heartbeat loss.
    Reset(20000);Step();Host("ARM ENABLE\n");Host("ARM POS 0 90 1000\n");Step();assert(Arm_GetStatus().Active);
    Host("ARM POS BAD\n");Step();assert(!Arm_GetStatus().Active);
    Host("ARM ENABLE\n");Host("ARM POS 0 90 1000\n");Step();
    for(unsigned i=0;i<31;i++){Step();}ArmDebug_Step();Step();assert(!Arm_GetStatus().Active);
    Host("ARM DISABLE\n");
    // Base feedback faults stay latched until an explicit stopped re-enable.
    Reset(30000);Encoder=(uint16_t)(100*ROBOT_ARM_BASE_ENCODER_SIGN);Step();
    for(unsigned i=0;i<80;i++){Step();}ArmRadio();
    for(unsigned i=0;i<80;i++){Send(0,NULL,0,1000,0);Step();}
    assert(Arm_GetStatus().Fault==ARM_BASE_FEEDBACK_ERROR && Base==0);
    for(unsigned i=0;i<40;i++){Send(0,NULL,0,0,0);Step();}
    assert(Arm_GetStatus().Fault==ARM_BASE_FEEDBACK_ERROR);
    // Even an explicit acknowledgement must wait for feedback to stop.
    Encoder=(uint16_t)(Encoder+20*ROBOT_ARM_BASE_ENCODER_SIGN);Step();
    Arm_Status Before=Arm_GetStatus();Send(1,NULL,0,0,0);Step();ArmRadio();
    assert(Arm_GetStatus().Fault==ARM_BASE_FEEDBACK_ERROR && !Base);
    for(unsigned i=0;i<80;i++){Send(0,NULL,0,0,0);Step();}
    Arm_Status After=Arm_GetStatus();assert(After.Fault==ARM_OK && !After.Active && !Base);
    assert(After.BaseCounts==Before.BaseCounts && After.ZeroGeneration==Before.ZeroGeneration);
    for(unsigned i=0;i<5;i++)Near(After.CommandedAngleDeg[i],Before.CommandedAngleDeg[i]);
    // Runtime peripheral failure can be explicitly reopened without resetting coordinates.
    Reset(40000);ArmRadio();int16_t A[5]={0,600,1000,1900,0};RunFollow(A,10);
    ApplyFailure=1;Step();assert(!Arm_GetStatus().Ready);
    Encoder=(uint16_t)(70*ROBOT_ARM_BASE_ENCODER_SIGN);Step();
    assert(Arm_GetStatus().BaseCounts==70);
    ApplyFailure=0;Send(1,NULL,0,0,0);Step();
    for(unsigned i=0;i<80;i++){Step();}Before=Arm_GetStatus();ArmRadio();After=Arm_GetStatus();
    assert(After.Ready && After.Fault==ARM_OK && After.BaseCounts==70 && After.ZeroGeneration==1);
    assert(!After.Active && !After.PositionEnabledMask && !Base);
    for(unsigned i=0;i<5;i++)Near(After.CommandedAngleDeg[i],Before.CommandedAngleDeg[i]);
    // A brief/small manual nudge cannot cancel an automatic sequence; sustained override can.
    Reset(50000);ArmRadio();Deploy++;Send(0,NULL,0,0,0);Step();
    for(unsigned i=0;i<8;i++){Send(0,NULL,1000,0,0);Step();assert(Arm_GetStatus().SequenceRunning);}
    Send(0,NULL,0,0,0);Step();
    for(unsigned i=0;i<8;i++){Send(0,NULL,100,0,0);Step();assert(Arm_GetStatus().SequenceRunning);}
    for(unsigned i=0;i<23;i++){Send(0,NULL,1000,0,0);Step();}
    assert(!Arm_GetStatus().SequenceRunning && Arm_GetStatus().Action==ARM_REMOTE);
    Send(1,NULL,0,0,0);Step();assert(!Arm_GetStatus().Active);
    puts("PASS: UART error/busy/overflow cannot interrupt radio; host timeout/explicit STOP retained");
    puts("PASS: stopped RESUME clears feedback/runtime output fault without re-zero/resume; deliberate manual override");
}
static void NeutralTicks(unsigned N)
{for(unsigned i=0;i<N;i++){if(i%2==0)Send(0,NULL,0,0,0);Step();}}
static void ReachRadioStage(uint8_t Returning,uint8_t Stage)
{
    Reset(0);ArmRadio();Deploy++;Send(0,NULL,0,0,0);Step();
    if(Returning){NeutralTicks(500);Deploy++;Send(0,NULL,0,0,0);Step();}
    for(unsigned i=0;i<800;i++)
    {
        Arm_Status S=Arm_GetStatus();
        if(S.SequenceRunning && S.SequenceStage==Stage)return;
        Send(0,NULL,0,0,0);Step();
    }
    assert(!"sequence stage not reached");
}
static void AutonomousLinkPolicy(void)
{
    const uint8_t Stages[][2]={{0,0},{0,255},{0,4},{1,2},{1,1},{1,3},{1,0}};
    const uint8_t Passive=REMOTE_CONTROL_FLAG_STOP|REMOTE_CONTROL_FLAG_PASSIVE_STOP;
    // Loss at every deploy/fold stage, both silence and reconnecting locked frames.
    for(unsigned Mode=0;Mode<2;Mode++)for(unsigned j=0;j<sizeof Stages/sizeof Stages[0];j++)
    {
        ReachRadioStage(Stages[j][0],Stages[j][1]);
        Arm_Status Start=Arm_GetStatus();
        for(unsigned i=0;i<900;i++)
        {
            if(Mode && i%2==0)Send(Passive,NULL,0,0,0);
            Step();if(Mode || i>=10)assert(!LastChassis.Enabled);assert(Arm_GetStatus().Fault==ARM_OK);
        }
        Arm_Status End=Arm_GetStatus();assert(!End.SequenceRunning && !End.Active && !Base);
        assert(End.SequencePose==(Stages[j][0]?ARM_SEQUENCE_ZERO:ARM_SEQUENCE_DEPLOYED));
        assert(End.ZeroGeneration==Start.ZeroGeneration);
        // Fresh motion alone after a lock cannot arm or replay a saved toggle.
        Deploy++;Send(0,NULL,0,1000,0);Step();assert(!Arm_GetStatus().Active && !Base);
        ArmRadio();Send(0,NULL,0,0,0);Step();assert(!Arm_GetStatus().SequenceRunning);
    }
    // Re-enabling during a still-running sequence must not insert a stopped cycle.
    ReachRadioStage(0,255);for(unsigned i=0;i<15;i++)Step();
    assert(Arm_GetStatus().SequenceRunning && !RemoteControl_GetLinkStatus().Armed);
    ArmRadio();assert(Arm_GetStatus().SequenceRunning);NeutralTicks(500);
    assert(Arm_GetStatus().SequencePose==ARM_SEQUENCE_DEPLOYED);
    // Explicit STOP dominates a later passive frame in the same control interval.
    ReachRadioStage(0,255);for(unsigned i=0;i<12;i++)Step();
    Send(1,NULL,0,0,0);Send(Passive,NULL,0,0,0);Step();assert(!Arm_GetStatus().Active && !Base);
    // It is honored even if its sequence number is stale after a passive STOP.
    ReachRadioStage(1,2);Send(Passive,NULL,0,0,0);Step();assert(Arm_GetStatus().SequenceRunning);
    Seq=(uint8_t)(Seq-2);Send(1,NULL,0,0,0);Step();assert(!Arm_GetStatus().SequenceRunning);
    ReachRadioStage(0,0);RemoteControl_Disconnect();Step();assert(Arm_GetStatus().SequenceRunning);
    Robot_ArmStop();Step();assert(!Arm_GetStatus().Active); // Host explicit STOP remains global.
    // Manual drive and pot targets do not inherit the autonomous exception.
    Reset(0);ArmRadio();RemoteControl_FrameTypeDef F={.Header=0xa5,.Version=3,.Sequence=Seq++,.M1=300,.M4=1000};
    F.Reserved[2]=1;uint8_t Wire[32];assert(CommandProtocol_Encode(&F,Wire));
    assert(RemoteControl_Process(Wire,32));Step();assert(LastChassis.Enabled && Base>0);
    for(unsigned i=0;i<12;i++){Step();}assert(!LastChassis.Enabled && !Arm_GetStatus().Active && !Base);
    Reset(0);ArmRadio();int16_t A[5]={0,950,1030,1900,0};RunFollow(A,10);
    for(unsigned i=0;i<12;i++){Step();}float Angle=Arm_GetStatus().CommandedAngleDeg[0];
    for(unsigned i=0;i<30;i++){Step();}Near(Arm_GetStatus().CommandedAngleDeg[0],Angle);
    assert(!Arm_GetStatus().Active && !Arm_GetStatus().BasePositionEnabled);
    // Feedback protection remains active while a fold continues without RF.
    Reset(0);ArmRadio();Deploy++;Send(0,NULL,0,0,0);Step();NeutralTicks(500);
    for(unsigned i=0;i<10;i++){Encoder=(uint16_t)(Encoder+20*ROBOT_ARM_BASE_ENCODER_SIGN);Send(0,NULL,0,0,0);Step();}
    NeutralTicks(80);Deploy++;Send(0,NULL,0,0,0);Step();assert(Arm_GetStatus().SequenceReturning);
    for(unsigned i=0;i<150;i++){Step();}
    assert(!Arm_GetStatus().SequenceRunning && Arm_GetStatus().Fault==ARM_BASE_FEEDBACK_ERROR && !Base);
    // Valid but non-progressing feedback must still hit the local12s deadline.
    Reset(0);ArmRadio();Deploy++;Send(0,NULL,0,0,0);Step();NeutralTicks(500);
    for(unsigned i=0;i<10;i++){Encoder=(uint16_t)(Encoder+20*ROBOT_ARM_BASE_ENCODER_SIGN);Send(0,NULL,0,0,0);Step();}
    NeutralTicks(80);Deploy++;Send(0,NULL,0,0,0);Step();
    for(unsigned i=0;i<1300;i++){Encoder=(uint16_t)(Encoder+(i%2?1:-1));Step();}
    assert(!Arm_GetStatus().SequenceRunning && Arm_GetStatus().Fault==ARM_EXPIRED && !Base);
    puts("PASS: all7 sequence stages survive RF silence/passive STOP; re-enable preserves progress, old actions do not replay");
    puts("PASS: operator/host STOP overrides offline sequence; manual/pot loss still stops, feedback and deadlines retained");
}
static void AutomaticLinkHandshake(void)
{
    Reset(0);ArmRadio();int16_t A[5]={0,950,1030,1900,0};RunFollow(A,10);
    for(unsigned i=0;i<30;i++)Step();
    float Held=Arm_GetStatus().CommandedAngleDeg[0];
    assert(!Arm_GetStatus().Active && !Base && !LastChassis.Enabled);
    // Relay absorbs action counters into neutral RESUME; they must not replay.
    Deploy++;Grip++;Send(2,NULL,0,0,0);Step();
    assert(!Arm_GetStatus().SequenceRunning && !Base);
    Near(Arm_GetStatus().CommandedAngleDeg[0],Held);
    Follow(A);Step();assert(Arm_GetStatus().Action==ARM_FOLLOW && !Arm_GetStatus().SequenceRunning);
    assert(Arm_GetStatus().CommandedAngleDeg[0]>=Held);
    // Explicit STOP is not undone by ordinary returning follow frames.
    Send(1,NULL,0,0,0);Step();Follow(A);Step();assert(!Arm_GetStatus().Active && !Base);
    puts("PASS: automatic neutral RESUME after100ms loss restores follow without stale events; STOP remains latched");
}
int main(void){LimitsAndGripper();ArbitrationAndStop();SequenceAndFaults();FoldBaseAndWrist();FoldArrivalHysteresis();SequenceWithNeutralRadio();FoldFromOutsidePositionLimits();FollowExitAndExplicitPresets();BaseLeaseAndRecovery();BasePositionIntegration();HostIsolationAndRemoteRecovery();AutonomousLinkPolicy();AutomaticLinkHandshake();return 0;}
