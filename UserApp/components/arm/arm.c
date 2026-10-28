#include "components/arm/arm.h"
#include "robot/robot_config.h"
#include "actuator/servo.h"
#include "actuator/base_motor.h"
#include "actuator/base_position.h"
#include "arm/bsp_arm.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static Servo_Config Servos[ARM_SERVO_COUNT]=ROBOT_ARM_SERVO_CONFIG;
static const Servo_Config DefaultServos[ARM_SERVO_COUNT]=ROBOT_ARM_SERVO_CONFIG;
static const float Preset[ARM_SERVO_COUNT]=ROBOT_ARM_PRESET_DEG;
static const float Deploy[ARM_SERVO_COUNT]=ROBOT_ARM_DEPLOY_DEG;
static const float PositionMin[ARM_SERVO_COUNT]=ROBOT_ARM_POSITION_MIN_DEG;
static const float PositionMax[ARM_SERVO_COUNT]=ROBOT_ARM_POSITION_MAX_DEG;
static const int8_t JogDirection[ARM_SERVO_COUNT]=ROBOT_ARM_JOG_DIRECTION;
static Arm_Status State;
static float Target[ARM_SERVO_COUNT];
static uint32_t Started,Duration;
static uint16_t LastCount;
static uint32_t SequenceStarted,StageStarted,JointDuration[ARM_SERVO_COUNT],ReturnWristDuration;
static float SequenceValues[ARM_SERVO_COUNT];
static uint8_t RemoteManual,ReturnStep,OverridePending,ReferenceReady;
static uint32_t OverrideTick;
static BaseMotor_Controller BaseControl;
static BasePosition_Controller BasePosition;
static float BaseRate;
static const Arm_Joint SequenceJoints[]={ARM_SHOULDER,ARM_ELBOW,ARM_WRIST_PITCH,ARM_WRIST_ROLL,ARM_GRIPPER};
static const uint8_t ReturnOrder[]={2,1,3,0};
_Static_assert(ARM_SERVO_COUNT==BSP_ARM_SERVO_COUNT,"Arm servo mapping mismatch");
_Static_assert(ROBOT_ARM_BASE_PWM_LIMIT>0 && ROBOT_ARM_BASE_PWM_LIMIT<=BSP_ARM_BASE_PWM_MAX,"Arm base PWM limit invalid");
_Static_assert(ROBOT_ARM_BASE_MOTOR_SIGN==1 || ROBOT_ARM_BASE_MOTOR_SIGN==-1,"Arm motor polarity invalid");
_Static_assert(ROBOT_ARM_BASE_ENCODER_SIGN==1 || ROBOT_ARM_BASE_ENCODER_SIGN==-1,"Arm encoder polarity invalid");
static void PublishBasePosition(void)
{
    State.BasePositionTargetDeg=BasePosition.TargetDeg;
    State.BasePositionReferenceDeg=BasePosition.ReferenceDeg;
    State.BasePositionErrorDeg=BasePosition.ErrorDeg;
    State.BasePositionEnabled=BasePosition.Enabled;
    State.BasePositionPickedUp=BasePosition.PickedUp;
    State.BasePositionAtTarget=BasePosition.AtTarget;
}
static float PositionLimit(uint8_t Joint)
{
    return PositionMax[Joint];
}
static uint8_t PositionPulse(uint8_t Joint,float PositionDeg,uint16_t *PulseUs)
{
    if(!isfinite(PositionDeg) || PositionDeg<PositionMin[Joint] || PositionDeg>PositionLimit(Joint))return 0;
    return Servo_Pulse(&Servos[Joint],PositionDeg,PulseUs);
}
static uint8_t Apply(void)
{
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
    {
        if(Servos[i].Mode==SERVO_POSITION && !(State.PositionEnabledMask&(1U<<i)))
            State.PulseUs[i]=0;
        else if(Servos[i].Mode==SERVO_POSITION)
        {if(!PositionPulse(i,State.CommandedAngleDeg[i],&State.PulseUs[i]))return 0;}
        else if(!Servo_Pulse(&Servos[i],State.Rate[i],&State.PulseUs[i]))return 0;
    }
    return BSP_Arm_Apply(State.PulseUs,State.BasePWM);
}
static void Halt(void)
{
    if(State.SequenceRunning)State.SequencePose=ARM_SEQUENCE_UNKNOWN;
    State.SequenceRunning=0;
    RemoteManual=OverridePending=0;
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++) {State.Rate[i]=0;Target[i]=State.CommandedAngleDeg[i];}
    BaseRate=0;BaseMotor_Stop(&BaseControl);State.BaseTargetRpm=0;
    BasePosition_Stop(&BasePosition);PublishBasePosition();
    State.BasePWM=0;State.Active=0;State.Action=ARM_STOP;
}
static void HardwareFault(void)
{
    Arm_Disable();State.Fault=ARM_HARDWARE_ERROR;
}
static void SetZero(void)
{
    (void)BaseMotor_Init(&BaseControl,&BaseControl.Config);
    LastCount=BSP_Arm_EncoderCount();State.BaseCounts=0;
    State.BaseRelativeAngleDeg=State.BaseRpm=State.BaseRawRpm=State.BaseTargetRpm=0;
    BasePosition_Stop(&BasePosition);
    BasePosition.TargetDeg=BasePosition.ReferenceDeg=BasePosition.ErrorDeg=0;
    PublishBasePosition();
    State.ZeroGeneration++;
    // Reset relative base feedback only: no servo position is measured or redefined.
}
void Arm_Disable(void) {Halt();State.Ready=0;BSP_Arm_Disable();}
void Arm_Stop(void)
{
    Halt();
    if(State.Ready && !Apply())HardwareFault();
}
uint8_t Arm_Init(void)
{
    ReferenceReady=0;Arm_Disable();
    memset(&State,0,sizeof(State));State.SelectedJoint=ARM_SHOULDER;
    memcpy(Servos,DefaultServos,sizeof(Servos));
    const BaseMotor_Config BaseConfig=ROBOT_ARM_BASE_CONTROL_CONFIG;
    const BasePosition_Config PositionConfig=ROBOT_ARM_BASE_POSITION_CONFIG;
    if(!BaseMotor_Init(&BaseControl,&BaseConfig) || !BasePosition_Init(&BasePosition,&PositionConfig) ||
       PositionConfig.MaxRpm>BaseConfig.MaxRpm){HardwareFault();return 0;}
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
    {
        if(!isfinite(PositionMin[i]) || !isfinite(PositionMax[i]) || PositionMin[i]<0 ||
           PositionMin[i]>=PositionMax[i] || PositionMax[i]>Servos[i].MaxAngleDeg ||
           (JogDirection[i]!=1 && JogDirection[i]!=-1))
        {HardwareFault();return 0;}
        State.CommandedAngleDeg[i]=Target[i]=Preset[i];
        if(!(Servos[i].Mode==SERVO_POSITION?PositionPulse(i,Preset[i],&State.PulseUs[i]):
             Servo_Pulse(&Servos[i],0,&State.PulseUs[i])))
        {HardwareFault();return 0;}
        if(Servos[i].Mode==SERVO_POSITION)State.PulseUs[i]=0;
    }
    if(!isfinite(ROBOT_ARM_POSITION_RATE_DPS) || ROBOT_ARM_POSITION_RATE_DPS<=0 ||
       !isfinite(ROBOT_ARM_MAX_SAMPLE_S) || ROBOT_ARM_MAX_SAMPLE_S<=0 ||
       !isfinite(ROBOT_ARM_BASE_CPR) || ROBOT_ARM_BASE_CPR<=0 ||
       !BSP_Arm_Init(State.PulseUs)) {HardwareFault();return 0;}
    SetZero();State.SequencePose=ROBOT_ARM_POSES_CALIBRATED?ARM_SEQUENCE_ZERO:ARM_SEQUENCE_UNKNOWN;
    ReferenceReady=1;State.Ready=1;State.Fault=ARM_OK;return 1;
}
uint8_t Arm_CommandValid(const Arm_Command *Command)
{
    if(Command==NULL)return 0;
    if(Command->Action==ARM_STOP)return 1;
    if(Command->DurationMs==0 || Command->DurationMs>ROBOT_ARM_COMMAND_MAX_MS)return 0;
    if(Command->Action==ARM_ZERO || Command->Action==ARM_CLEAR)
        return !State.Active && State.BasePWM==0 && fabsf(State.BaseRpm)<=.4f && fabsf(State.BaseRawRpm)<=.4f;
    if(Command->Action==ARM_INITIALIZE)return ROBOT_ARM_POSES_CALIBRATED;
    if(Command->Action==ARM_PRESET)return 0; // Use INIT, not the legacy all-five-joint preset.
    if((unsigned)Command->Joint>=ARM_JOINT_COUNT || !isfinite(Command->Value))return 0;
    if(Command->Action==ARM_RATE)
        return Command->Value>=-1 && Command->Value<=1 &&
               (Command->Joint==ARM_BASE || Servos[Command->Joint].Mode==SERVO_CONTINUOUS ||
                (State.PositionEnabledMask&(1U<<Command->Joint)));
    if(Command->Action==ARM_POSITION)
    {
        if(Command->Joint==ARM_BASE)
            return Command->Value>=BasePosition.Config.MinDeg && Command->Value<=BasePosition.Config.MaxDeg;
        return Command->Joint<ARM_SERVO_COUNT && Servos[Command->Joint].Mode==SERVO_POSITION &&
               Command->Value>=PositionMin[Command->Joint] && Command->Value<=PositionLimit((uint8_t)Command->Joint);
    }
    // NEUTRAL is a legacy wire verb; there are no continuous servos to calibrate.
    return 0;
}
static void BeginSequenceStage(uint32_t NowMs)
{
    unsigned Stage=State.SequenceStage;
    Arm_Joint Joint=SequenceJoints[Stage];
    for(unsigned i=0;i<ARM_SERVO_COUNT;i++)State.Rate[i]=0;
    Target[Joint]=SequenceValues[Stage];State.PositionEnabledMask|=(uint8_t)(1U<<Joint);
    State.SelectedJoint=Joint;StageStarted=NowMs;
}
static uint8_t BeginSequence(uint32_t NowMs)
{
    float Values[ARM_SERVO_COUNT];uint32_t Durations[ARM_SERVO_COUNT];
    if(State.SequenceReturning)
    {
        uint16_t Pulse;
        if(!PositionPulse(ARM_WRIST_ROLL,ROBOT_ARM_FOLD_WRIST_CLEAR_DEG,&Pulse) ||
           !isfinite(State.BaseRelativeAngleDeg) || BaseControl.Fault)return 0;
        // +/-180deg bounds constrain POSITION TARGETS, not the measured start.
        // Manual speed control can leave the base beyond them. The fixed zero
        // target and BasePosition_Step's inward-only limit check allow recovery.
    }
    for(unsigned i=0;i<ARM_SERVO_COUNT;i++)
    {
        Arm_Joint Joint=SequenceJoints[i];
        Values[i]=State.SequenceReturning?Preset[Joint]:Deploy[Joint];
        float Ms=ceilf(fabsf(Values[i]-State.CommandedAngleDeg[Joint])*1000.0f/ROBOT_ARM_POSITION_RATE_DPS)+
                 ROBOT_ARM_SEQUENCE_SETTLE_MS;
        // Fold visits wrist-clear first, then moves from clear to the final preset.
        if(State.SequenceReturning && Joint==ARM_WRIST_ROLL)
            Ms=ceilf(fabsf(Values[i]-ROBOT_ARM_FOLD_WRIST_CLEAR_DEG)*1000.0f/ROBOT_ARM_POSITION_RATE_DPS)+
               ROBOT_ARM_SEQUENCE_SETTLE_MS;
        if(!isfinite(Ms) || Ms<=0 || Ms>ROBOT_ARM_COMMAND_MAX_MS)return 0;
        Arm_Command Check={ARM_POSITION,Joint,Values[i],(uint32_t)Ms};
        if(!Arm_CommandValid(&Check))return 0;
        Durations[i]=(uint32_t)Ms;
    }
    // Validate all targets before activating any joint.
    for(unsigned i=0;i<ARM_SERVO_COUNT;i++)
    {
        JointDuration[i]=Durations[i];SequenceValues[i]=Values[i];
    }
    SequenceStarted=NowMs;
    for(unsigned i=0;i<ARM_SERVO_COUNT;i++)State.Rate[i]=0;
    if(State.SequenceReturning)
    {
        ReturnStep=0;State.SequenceStage=ReturnOrder[ReturnStep];
        BeginSequenceStage(NowMs);
        Target[ARM_GRIPPER]=SequenceValues[4];
        State.PositionEnabledMask|=(uint8_t)(1U<<ARM_GRIPPER);
        ReturnWristDuration=(uint32_t)ceilf(fabsf(State.CommandedAngleDeg[ARM_WRIST_ROLL]-
            ROBOT_ARM_FOLD_WRIST_CLEAR_DEG)*1000.0f/ROBOT_ARM_POSITION_RATE_DPS)+ROBOT_ARM_SEQUENCE_SETTLE_MS;
        Target[ARM_WRIST_ROLL]=ROBOT_ARM_FOLD_WRIST_CLEAR_DEG;
        State.PositionEnabledMask|=(uint8_t)(1U<<ARM_WRIST_ROLL);
        // Explicit fold owns the base, unlike pot mode it does not wait for pickup.
        // The boot encoder reference is preserved: do not redefine zero here.
        BasePosition_Start(&BasePosition,State.BaseRelativeAngleDeg,0);
        (void)BasePosition_SetTarget(&BasePosition,0);
        BaseRate=0;PublishBasePosition();
    }
    else
    {
        State.SequenceStage=0U;BeginSequenceStage(NowMs);
    }
    return 1;
}
uint8_t Arm_GoToPose(Arm_PoseId Pose,uint32_t NowMs)
{
    if(Pose!=ARM_POSE_FOLDED && Pose!=ARM_POSE_EXTENDED)return 0;
    if(!State.Ready)return 0;
    if(!ROBOT_ARM_POSES_CALIBRATED){State.Fault=ARM_NEEDS_CALIBRATION;return 0;}
    if(State.SequenceRunning)return 1; // A held/repeated button cannot restart the timer.
    Arm_Stop();if(!State.Ready)return 0;
    State.SequenceReturning=Pose==ARM_POSE_FOLDED;
    State.Active=1;State.SequenceDoneMask=0;
    State.Action=ARM_INITIALIZE;State.Fault=ARM_OK;State.SelectedJoint=ARM_SHOULDER;
    if(!BeginSequence(NowMs))
    {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    State.SequenceRunning=1; // Validation precedes activation/marking an interrupted reference.
    return 1;
}
static uint8_t StartSequence(uint32_t NowMs)
{
    // After teaching/manual control, UNKNOWN means "not at a named pose", not
    // that the commanded servo coordinates or base encoder zero were erased.
    // The toggle deploys only from folded; every other tracked pose first folds.
    return Arm_GoToPose(State.SequencePose==ARM_SEQUENCE_ZERO?ARM_POSE_EXTENDED:ARM_POSE_FOLDED,NowMs);
}
static void SequenceStep(uint32_t NowMs)
{
    if((uint32_t)(NowMs-SequenceStarted)>=ROBOT_ARM_SEQUENCE_MAX_MS)
    {Arm_Stop();State.Fault=ARM_EXPIRED;return;}
    if(State.SequenceReturning)
    {
        unsigned Stage=State.SequenceStage;Arm_Joint Joint=SequenceJoints[Stage];
        if(Stage==2U)
        {
            if((uint32_t)(NowMs-StageStarted)>=ROBOT_ARM_BASE_RETURN_TIMEOUT_MS)
            {Arm_Stop();State.Fault=ARM_EXPIRED;return;}
            if((uint32_t)(NowMs-StageStarted)>=JointDuration[4] &&
               fabsf(Target[ARM_GRIPPER]-State.CommandedAngleDeg[ARM_GRIPPER])<=0.001f)
                State.SequenceDoneMask|=0x10U;
            if(!(State.SequenceDoneMask&0x10U))return;
            if((uint32_t)(NowMs-StageStarted)<ReturnWristDuration ||
               fabsf(State.CommandedAngleDeg[ARM_WRIST_ROLL]-ROBOT_ARM_FOLD_WRIST_CLEAR_DEG)>0.001f ||
               // Use the position controller's arrival latch and hold band together;
               // requiring the narrower arrival band again can deadlock after coast.
               !BasePosition.AtTarget || fabsf(State.BaseRelativeAngleDeg)>BasePosition.Config.ReleaseDeg ||
               fabsf(State.BaseRpm)>BasePosition.Config.StoppedRpm)return;
        }
        if((uint32_t)(NowMs-StageStarted)<JointDuration[Stage] ||
           fabsf(Target[Joint]-State.CommandedAngleDeg[Joint])>0.001f)return;
        State.SequenceDoneMask|=(uint8_t)(1U<<Stage);
        if(Stage==2U)
        {
            BasePosition_Stop(&BasePosition);BaseRate=0;
            BaseMotor_Stop(&BaseControl);State.BaseTargetRpm=0;PublishBasePosition();
        }
    }
    else
    {
        if(State.SequenceStage==0U)
        {
            if((uint32_t)(NowMs-StageStarted)<JointDuration[0] ||
               fabsf(Target[ARM_SHOULDER]-State.CommandedAngleDeg[ARM_SHOULDER])>0.001f)return;
            State.SequenceDoneMask|=1U;
            // Shoulder has finished; activate the remaining three together.
            State.SequenceStage=255U;StageStarted=NowMs;
            for(unsigned i=1;i<4;i++)
            {
                Arm_Joint Joint=SequenceJoints[i];Target[Joint]=SequenceValues[i];
                State.PositionEnabledMask|=(uint8_t)(1U<<Joint);
            }
            return;
        }
        if(State.SequenceStage==4U)
        {
            if((uint32_t)(NowMs-StageStarted)>=JointDuration[4] &&
               fabsf(Target[ARM_GRIPPER]-State.CommandedAngleDeg[ARM_GRIPPER])<=0.001f)
                State.SequenceDoneMask|=0x10U;
        }
        else
        {
            for(unsigned i=1;i<4;i++)
                if((uint32_t)(NowMs-StageStarted)>=JointDuration[i] &&
                   fabsf(Target[SequenceJoints[i]]-State.CommandedAngleDeg[SequenceJoints[i]])<=0.001f)
                    State.SequenceDoneMask|=(uint8_t)(1U<<i);
            if((State.SequenceDoneMask&0x0fU)==0x0fU)
            {
                State.SequenceStage=4U;BeginSequenceStage(NowMs);return;
            }
        }
    }
    if(State.SequenceDoneMask==0x1fU)
    {
        State.SequenceRunning=0;Halt();
        State.SequencePose=State.SequenceReturning?ARM_SEQUENCE_ZERO:ARM_SEQUENCE_DEPLOYED;
        return;
    }
    if(State.SequenceReturning)
    {
        ReturnStep++;State.SequenceStage=ReturnOrder[ReturnStep];
        BeginSequenceStage(NowMs);
    }
}
uint8_t Arm_ClearFault(void)
{
    Arm_Command Clear={ARM_CLEAR,ARM_BASE,0,1000};
    if(!ReferenceReady || !Arm_CommandValid(&Clear))return 0;
    if(!State.Ready)
    {
        if(State.Fault!=ARM_HARDWARE_ERROR)return 0;
        // Reopen only output peripherals after a runtime failure. Encoder sampling
        // continued while disabled; never call Arm_Init/SetZero or restore old PWM.
        uint16_t Off[ARM_SERVO_COUNT]={0};
        if(!BSP_Arm_Init(Off))return 0;
        State.PositionEnabledMask=0;memset(State.PulseUs,0,sizeof State.PulseUs);
        State.Ready=1;
    }
    Arm_Stop();if(!State.Ready)return 0;
    if(!BaseMotor_Init(&BaseControl,&BaseControl.Config))return 0;
    State.Fault=ARM_OK;return 1;
}
uint8_t Arm_Request(const Arm_Command *Command,uint32_t CreatedMs,uint32_t NowMs)
{
    // CLEAR may reopen a runtime peripheral failure; startup without a reference
    // is still rejected by Arm_ClearFault. Never accept an expired acknowledgement.
    if(Command && Command->Action==ARM_CLEAR)
        return Arm_CommandValid(Command) && (uint32_t)(NowMs-CreatedMs)<Command->DurationMs && Arm_ClearFault();
    if(!State.Ready)return 0;
    if(!Arm_CommandValid(Command)) {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    if(Command->Action!=ARM_STOP && (uint32_t)(NowMs-CreatedMs)>=Command->DurationMs)
    {Arm_Stop();State.Fault=ARM_EXPIRED;return 0;}
    if(Command->Action==ARM_INITIALIZE)return StartSequence(NowMs);
    // Host keepalive renews base RATE/POS without resetting its controllers/ramp.
    if(!(Command->Joint==ARM_BASE && State.Active && State.SelectedJoint==ARM_BASE &&
         Command->Action==State.Action && (State.Action==ARM_RATE || State.Action==ARM_POSITION)))Arm_Stop();
    if(!State.Ready)return 0;
    State.Fault=ARM_OK;
    if(Command->Action==ARM_STOP)return 1;
    if(Command->Action==ARM_ZERO)
    {
        SetZero();State.Action=ARM_ZERO;return 1;
    }
    if(Command->Joint<=ARM_WRIST_ROLL)
        State.SequencePose=ARM_SEQUENCE_UNKNOWN;
    Started=CreatedMs;Duration=Command->DurationMs;
    State.Action=Command->Action;State.Active=1;
    if(Command->Action==ARM_PRESET)
    {
        memcpy(Target,Preset,sizeof(Target));
        for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)if(Servos[i].Mode==SERVO_POSITION)
        {
            if(!(State.PositionEnabledMask&(1U<<i)))State.CommandedAngleDeg[i]=Preset[i];
            State.PositionEnabledMask|=(uint8_t)(1U<<i);
        }
        State.SequencePose=ARM_SEQUENCE_ZERO;
    }
    else
    {
        State.SelectedJoint=Command->Joint;
        if(Command->Action==ARM_POSITION && Command->Joint==ARM_BASE)
        {
            if(!BasePosition.Enabled)BasePosition_Start(&BasePosition,State.BaseRelativeAngleDeg,0);
            (void)BasePosition_SetTarget(&BasePosition,Command->Value);
            BaseRate=0;PublishBasePosition();
        }
        else if(Command->Action==ARM_POSITION)
        {
            if(!(State.PositionEnabledMask&(1U<<Command->Joint)))State.CommandedAngleDeg[Command->Joint]=Command->Value;
            State.PositionEnabledMask|=(uint8_t)(1U<<Command->Joint);
            Target[Command->Joint]=Command->Value;
        }
        else if(Command->Joint==ARM_BASE)BaseRate=Command->Value;
        else
        {
            State.Rate[Command->Joint]=Command->Value;
        }
    }
    return 1;
}
void Arm_ResetSampling(void) {LastCount=BSP_Arm_EncoderCount();}
static void ToggleGripper(void)
{
    if(!ROBOT_ARM_GRIPPER_CALIBRATED)return;
    Target[ARM_GRIPPER]=Target[ARM_GRIPPER]==ROBOT_ARM_GRIPPER_OPEN_DEG?
                       ROBOT_ARM_GRIPPER_CLOSED_DEG:ROBOT_ARM_GRIPPER_OPEN_DEG;
    State.PositionEnabledMask|=(uint8_t)(1U<<ARM_GRIPPER);
}
uint8_t Arm_FollowTargets(const float AngleDeg[5],uint8_t Gripper,uint32_t CreatedMs,uint32_t NowMs)
{
    if(!State.Ready)return 0;
    if(AngleDeg==NULL){Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    if(!isfinite(AngleDeg[0])){Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    for(unsigned i=0;i<4;i++)
        if(!isfinite(AngleDeg[i+1]) || Servos[i].Mode!=SERVO_POSITION)
        {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    if((uint32_t)(NowMs-CreatedMs)>=ROBOT_REMOTE_TIMEOUT_MS)
    {Arm_Stop();State.Fault=ARM_EXPIRED;return 0;}
    if(State.Action!=ARM_FOLLOW)
    {
        Arm_Stop();if(!State.Ready)return 0;
        // Explicit mode entry authorizes a bounded ramp from the tracked position.
        BasePosition_Start(&BasePosition,State.BaseRelativeAngleDeg,0);
    }
    State.SequencePose=ARM_SEQUENCE_UNKNOWN;
    State.Action=ARM_FOLLOW;State.Active=1;State.Fault=ARM_OK;
    (void)BasePosition_SetTarget(&BasePosition,AngleDeg[0]);PublishBasePosition();
    BaseRate=0;Started=CreatedMs;Duration=ROBOT_REMOTE_TIMEOUT_MS;
    for(unsigned i=0;i<4;i++)
    {
        float Value=AngleDeg[i+1];
        if(Value<PositionMin[i])Value=PositionMin[i];
        if(Value>PositionLimit(i))Value=PositionLimit(i);
        Target[i]=Value;State.Rate[i]=0;
        State.PositionEnabledMask|=(uint8_t)(1U<<i);
    }
    if(Gripper)ToggleGripper();
    return 1;
}
void Arm_RemoteInput(float Base,Arm_Joint Joint,float Rate,uint8_t Initialize,uint8_t Gripper,uint32_t NowMs)
{
    if(!State.Ready)return;
    if(!isfinite(Base) || !isfinite(Rate) || fabsf(Base)>1 || fabsf(Rate)>1 || (unsigned)Joint>ARM_WRIST_ROLL)
    {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return;}
    if(Initialize && ROBOT_ARM_POSES_CALIBRATED && fabsf(Base)<=ROBOT_ARM_SEQUENCE_TRIGGER_RATE && fabsf(Rate)<=ROBOT_ARM_SEQUENCE_TRIGGER_RATE){(void)StartSequence(NowMs);return;}
    // Ignore obsolete shortcut events without interrupting ordinary manual control.
    if(!ROBOT_ARM_GRIPPER_CALIBRATED)Gripper=0;
    if(State.SequenceRunning)
    {
        // Brief/small stick bumps do not cancel a preset. STOP remains immediate.
        if(fabsf(Base)<ROBOT_ARM_SEQUENCE_OVERRIDE_RATE && fabsf(Rate)<ROBOT_ARM_SEQUENCE_OVERRIDE_RATE){OverridePending=0;return;}
        if(!OverridePending){OverridePending=1;OverrideTick=NowMs;return;}
        if((uint32_t)(NowMs-OverrideTick)<ROBOT_ARM_SEQUENCE_OVERRIDE_MS)return;
        Arm_Stop(); // Deliberate manual override, retaining tracked coordinates.
    }
    if(!RemoteManual)
    {
        if(Base==0 && Rate==0 && !Gripper)return;
        Arm_Stop();if(!State.Ready)return;
        RemoteManual=1;
    }
    if(State.SelectedJoint!=Joint && State.SelectedJoint<ARM_SERVO_COUNT)
        Target[State.SelectedJoint]=State.CommandedAngleDeg[State.SelectedJoint];
    for(unsigned i=0;i<ARM_SERVO_COUNT;i++)State.Rate[i]=0;
    State.SelectedJoint=Joint;State.Action=ARM_REMOTE;State.Active=1;State.Fault=ARM_OK;
    Started=NowMs;Duration=ROBOT_REMOTE_TIMEOUT_MS;
    BaseRate=Base;
    State.Rate[Joint]=Rate;
    if(Rate!=0 && Servos[Joint].Mode==SERVO_POSITION)State.PositionEnabledMask|=(uint8_t)(1U<<Joint);
    if(Gripper)
    {
        ToggleGripper();
    }
}
void Arm_Step(float DtS,uint32_t NowMs)
{
    if(!ReferenceReady)return;
    State.TickMs=NowMs;
    uint16_t Count=BSP_Arm_EncoderCount();
    uint16_t Raw=(uint16_t)(Count-LastCount);LastCount=Count;
    int32_t Delta=Raw>=32768?(int32_t)Raw-65536:Raw;
    Delta*=ROBOT_ARM_BASE_ENCODER_SIGN;
    State.BaseCounts+=Delta;
    State.BaseRelativeAngleDeg=(float)State.BaseCounts*360.0f/ROBOT_ARM_BASE_CPR;
    if(!isfinite(DtS) || DtS<=0 || DtS>ROBOT_ARM_MAX_SAMPLE_S)
    {State.BaseRpm=0;Arm_Stop();if(State.Ready)State.Fault=ARM_BAD_SAMPLE;return;}
    State.BaseRawRpm=(float)Delta*60.0f/(ROBOT_ARM_BASE_CPR*DtS);
    if(!State.Ready)
    {
        // Outputs stay disabled, but position/stopped validation remains observable
        // for an explicit remote acknowledgement without redefining boot zero.
        State.BaseRpm+=(State.BaseRawRpm-State.BaseRpm)*DtS/(.03f+DtS);
        return;
    }
    if(State.SequenceRunning)SequenceStep(NowMs);
    else if(State.Active && (uint32_t)(NowMs-Started)>=Duration)
    {Arm_Stop();State.Fault=ARM_EXPIRED;}
    if(BasePosition.Enabled)
    {
        float PositionRate=BasePosition_Step(&BasePosition,State.BaseRelativeAngleDeg,BaseControl.FilteredRpm,DtS);
        BaseRate=PositionRate*BasePosition.Config.MaxRpm/BaseControl.Config.MaxRpm;
        if(BasePosition.Invalid){Arm_Stop();State.Fault=ARM_BAD_SAMPLE;}
    }
    PublishBasePosition();
    State.BasePWM=(int16_t)(BaseMotor_Step(&BaseControl,BaseRate,State.BaseRawRpm,DtS)*ROBOT_ARM_BASE_MOTOR_SIGN);
    State.BaseRpm=BaseControl.FilteredRpm;State.BaseTargetRpm=BaseControl.TargetRpm;
    if(BaseControl.Fault && (BaseRate!=0 || BasePosition.Enabled))
    {Arm_Stop();State.Fault=ARM_BASE_FEEDBACK_ERROR;}
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
    {
        if(Servos[i].Mode!=SERVO_POSITION)continue;
        if(State.Active && (State.Action==ARM_RATE || State.Action==ARM_REMOTE) && State.SelectedJoint==i)
        {
            Target[i]=State.CommandedAngleDeg[i]+JogDirection[i]*State.Rate[i]*ROBOT_ARM_POSITION_RATE_DPS*DtS;
            if(Target[i]<PositionMin[i])Target[i]=PositionMin[i];
            if(Target[i]>PositionLimit(i))Target[i]=PositionLimit(i);
        }
        float Error=Target[i]-State.CommandedAngleDeg[i],Step=ROBOT_ARM_POSITION_RATE_DPS*DtS;
        if(Error>Step)Error=Step;
        if(Error<-Step)Error=-Step;
        State.CommandedAngleDeg[i]+=Error;
    }
    if(!Apply())HardwareFault();
}
Arm_Status Arm_GetStatus(void) {return State;}
