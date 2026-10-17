#include "components/arm/arm.h"
#include "robot/robot_config.h"
#include "actuator/servo.h"
#include "arm/bsp_arm.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static Servo_Config Servos[ARM_SERVO_COUNT]=ROBOT_ARM_SERVO_CONFIG;
static const Servo_Config DefaultServos[ARM_SERVO_COUNT]=ROBOT_ARM_SERVO_CONFIG;
static const float Preset[ARM_SERVO_COUNT]=ROBOT_ARM_PRESET_DEG;
static Arm_Status State;
static float Target[ARM_SERVO_COUNT];
static uint32_t Started,Duration;
static uint16_t LastCount;
static uint32_t SequenceStarted,StageStarted,JointDuration[4];
static float SequenceValues[4];
static float SequenceAngleTargets[2];
static uint8_t RemoteManual;
static const Arm_Joint SequenceJoints[]={ARM_SHOULDER,ARM_ELBOW,ARM_WRIST_PITCH,ARM_GRIPPER};
_Static_assert(ARM_SERVO_COUNT==BSP_ARM_SERVO_COUNT,"Arm servo mapping mismatch");
_Static_assert(ROBOT_ARM_BASE_PWM_LIMIT>0 && ROBOT_ARM_BASE_PWM_LIMIT<=BSP_ARM_BASE_PWM_MAX,"Arm base PWM limit invalid");
_Static_assert(ROBOT_ARM_BASE_MOTOR_SIGN==1 || ROBOT_ARM_BASE_MOTOR_SIGN==-1,"Arm motor polarity invalid");
_Static_assert(ROBOT_ARM_BASE_ENCODER_SIGN==1 || ROBOT_ARM_BASE_ENCODER_SIGN==-1,"Arm encoder polarity invalid");
static float PositionLimit(uint8_t Joint)
{
    if(Joint==ARM_GRIPPER)return ROBOT_ARM_GRIPPER_OPEN_DEG;
    return Joint==ARM_SHOULDER?ROBOT_ARM_SHOULDER_POSITION_MAX_DEG:Servos[Joint].MaxAngleDeg;
}
static uint8_t PositionPulse(uint8_t Joint,float PositionDeg,uint16_t *PulseUs)
{
    if(!isfinite(PositionDeg) || PositionDeg<0 || PositionDeg>PositionLimit(Joint))return 0;
    float ServoDeg=Joint==ARM_SHOULDER?ROBOT_ARM_SHOULDER_ZERO_DEG+PositionDeg:PositionDeg;
    return Servo_Pulse(&Servos[Joint],ServoDeg,PulseUs);
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
    RemoteManual=0;
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++) {State.Rate[i]=0;Target[i]=State.CommandedAngleDeg[i];}
    State.BasePWM=0;State.Active=0;State.Action=ARM_STOP;
}
static void HardwareFault(void)
{
    Arm_Disable();State.Fault=ARM_HARDWARE_ERROR;
}
static void SetZero(void)
{
    LastCount=BSP_Arm_EncoderCount();State.BaseCounts=0;
    State.BaseRelativeAngleDeg=State.BaseRpm=0;
    State.ZeroGeneration++;
    State.EstimatedContinuousDeg[0]=State.EstimatedContinuousDeg[1]=0;
    State.SequencePose=State.CommandedAngleDeg[ARM_SHOULDER]==0 &&
                       State.CommandedAngleDeg[ARM_GRIPPER]==ROBOT_ARM_GRIPPER_CLOSED_DEG?
                       ARM_SEQUENCE_ZERO:ARM_SEQUENCE_UNKNOWN;
}
void Arm_Disable(void) {Halt();State.Ready=0;BSP_Arm_Disable();}
void Arm_Stop(void)
{
    Halt();
    if(State.Ready && !Apply())HardwareFault();
}
uint8_t Arm_Init(void)
{
    Arm_Disable();
    memset(&State,0,sizeof(State));State.SelectedJoint=ARM_SHOULDER;
    memcpy(Servos,DefaultServos,sizeof(Servos));
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
    {
        State.CommandedAngleDeg[i]=Target[i]=Preset[i];
        if(!(Servos[i].Mode==SERVO_POSITION?PositionPulse(i,Preset[i],&State.PulseUs[i]):
             Servo_Pulse(&Servos[i],0,&State.PulseUs[i])))
        {HardwareFault();return 0;}
        if(Servos[i].Mode==SERVO_POSITION)State.PulseUs[i]=0;
    }
    if(!isfinite(ROBOT_ARM_POSITION_RATE_DPS) || ROBOT_ARM_POSITION_RATE_DPS<=0 ||
       !isfinite(ROBOT_ARM_SHOULDER_ZERO_DEG) || !isfinite(ROBOT_ARM_SHOULDER_POSITION_MAX_DEG) ||
       ROBOT_ARM_SHOULDER_POSITION_MAX_DEG<=0 || ROBOT_ARM_SHOULDER_ZERO_DEG<0 ||
       ROBOT_ARM_SHOULDER_ZERO_DEG+ROBOT_ARM_SHOULDER_POSITION_MAX_DEG>Servos[ARM_SHOULDER].MaxAngleDeg ||
       !isfinite(ROBOT_ARM_ELBOW_RATE_LIMIT) || ROBOT_ARM_ELBOW_RATE_LIMIT<=0 || ROBOT_ARM_ELBOW_RATE_LIMIT>1 ||
       !isfinite(ROBOT_ARM_WRIST_PITCH_RATE_LIMIT) || ROBOT_ARM_WRIST_PITCH_RATE_LIMIT<=0 || ROBOT_ARM_WRIST_PITCH_RATE_LIMIT>1 ||
       !isfinite(ROBOT_ARM_MAX_SAMPLE_S) || ROBOT_ARM_MAX_SAMPLE_S<=0 ||
       !isfinite(ROBOT_ARM_BASE_CPR) || ROBOT_ARM_BASE_CPR<=0 ||
       !BSP_Arm_Init(State.PulseUs)) {HardwareFault();return 0;}
    SetZero();State.Ready=1;State.Fault=ARM_OK;return 1;
}
uint8_t Arm_CommandValid(const Arm_Command *Command)
{
    if(Command==NULL)return 0;
    if(Command->Action==ARM_STOP)return 1;
    if(Command->DurationMs==0 || Command->DurationMs>ROBOT_ARM_COMMAND_MAX_MS)return 0;
    if(Command->Action==ARM_PRESET || Command->Action==ARM_ZERO || Command->Action==ARM_INITIALIZE)return 1;
    if(Command->Joint>=ARM_JOINT_COUNT || (int)Command->Joint<0 || !isfinite(Command->Value))return 0;
    if(Command->Action==ARM_RATE)
        return Command->Joint!=ARM_GRIPPER && Command->Value>=-1 && Command->Value<=1 &&
               (Command->Joint==ARM_BASE || Servos[Command->Joint].Mode==SERVO_CONTINUOUS ||
                (State.PositionEnabledMask&(1U<<Command->Joint)));
    if(Command->Action==ARM_POSITION)
        return Command->Joint<ARM_SERVO_COUNT && Servos[Command->Joint].Mode==SERVO_POSITION &&
               Command->Value>=0 && Command->Value<=PositionLimit((uint8_t)Command->Joint) &&
               (Command->Joint!=ARM_GRIPPER || Command->Value==ROBOT_ARM_GRIPPER_CLOSED_DEG ||
                Command->Value==ROBOT_ARM_GRIPPER_OPEN_DEG);
    if(Command->Action==ARM_NEUTRAL)
        return Command->Joint<ARM_SERVO_COUNT && Servos[Command->Joint].Mode==SERVO_CONTINUOUS &&
               Command->Value>=Servos[Command->Joint].MinUs && Command->Value<=Servos[Command->Joint].MaxUs &&
               floorf(Command->Value)==Command->Value;
    return 0;
}
static void BeginSequenceStage(uint32_t NowMs)
{
    unsigned Stage=State.SequenceStage;
    Arm_Joint Joint=SequenceJoints[Stage];
    for(unsigned i=0;i<ARM_SERVO_COUNT;i++)State.Rate[i]=0;
    if(Joint==ARM_SHOULDER || Joint==ARM_GRIPPER)
    {
        Target[Joint]=SequenceValues[Stage];State.PositionEnabledMask|=(uint8_t)(1U<<Joint);
    }
    else State.Rate[Joint]=SequenceValues[Stage];
    State.SelectedJoint=Joint;StageStarted=NowMs;
}
static uint8_t BeginSequence(uint32_t NowMs)
{
    float Values[4];uint32_t Durations[4];
    for(unsigned i=0;i<4;i++)
    {
        Arm_Joint Joint=SequenceJoints[i];float Ms;Arm_Action Action;
        if(Joint==ARM_SHOULDER || Joint==ARM_GRIPPER)
        {
            Action=ARM_POSITION;
            Values[i]=State.SequenceReturning?0:Joint==ARM_SHOULDER?
                      ROBOT_ARM_INIT_SHOULDER_DEG:ROBOT_ARM_GRIPPER_OPEN_DEG;
            Ms=ceilf(fabsf(Values[i]-State.CommandedAngleDeg[Joint])*1000.0f/ROBOT_ARM_POSITION_RATE_DPS)+
               ROBOT_ARM_SEQUENCE_SETTLE_MS;
        }
        else
        {
            Action=ARM_RATE;
            unsigned Index=Joint==ARM_ELBOW?0U:1U;
            float Goal=State.SequenceReturning?0:Joint==ARM_ELBOW?ROBOT_ARM_INIT_ELBOW_DEG:-ROBOT_ARM_INIT_WRIST_PITCH_DEG;
            float Degrees=Goal-State.EstimatedContinuousDeg[Index];
            SequenceAngleTargets[Index]=Goal;
            float Dps=Joint==ARM_ELBOW?ROBOT_ARM_ELBOW_REFERENCE_DPS:ROBOT_ARM_WRIST_PITCH_REFERENCE_DPS;
            float Limit=Joint==ARM_ELBOW?ROBOT_ARM_ELBOW_RATE_LIMIT:ROBOT_ARM_WRIST_PITCH_RATE_LIMIT;
            Values[i]=Degrees>0?Limit:Degrees<0?-Limit:0;
            Ms=fmaxf(1,ceilf(fabsf(Degrees)*1000.0f/Dps));
        }
        if(!isfinite(Ms) || Ms<=0 || Ms>ROBOT_ARM_COMMAND_MAX_MS)return 0;
        Arm_Command Check={Action,Joint,Values[i],(uint32_t)Ms};
        if(!Arm_CommandValid(&Check))return 0;
        Durations[i]=(uint32_t)Ms;
    }
    // Validate all four stages before activating the first joint.
    for(unsigned i=0;i<4;i++)
    {
        JointDuration[i]=Durations[i];SequenceValues[i]=Values[i];
    }
    SequenceStarted=NowMs;
    State.SequenceStage=State.SequenceReturning?3U:0U;
    BeginSequenceStage(NowMs);
    return 1;
}
static uint8_t StartSequence(uint32_t NowMs)
{
    if(State.SequenceRunning)return 1; // A held/repeated button cannot restart the timer.
    if(State.SequencePose==ARM_SEQUENCE_UNKNOWN)
    {Arm_Stop();State.Fault=ARM_NEEDS_ZERO;return 0;}
    Arm_Stop();if(!State.Ready)return 0;
    State.SequenceReturning=State.SequencePose==ARM_SEQUENCE_DEPLOYED;
    State.SequenceRunning=State.Active=1;State.SequenceDoneMask=0;
    State.Action=ARM_INITIALIZE;State.Fault=ARM_OK;State.SelectedJoint=ARM_SHOULDER;
    if(!BeginSequence(NowMs))
    {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    return 1;
}
static void SequenceStep(uint32_t NowMs)
{
    if((uint32_t)(NowMs-SequenceStarted)>=ROBOT_ARM_SEQUENCE_MAX_MS)
    {Arm_Stop();State.Fault=ARM_EXPIRED;return;}
    unsigned Stage=State.SequenceStage;
    if((uint32_t)(NowMs-StageStarted)<JointDuration[Stage])return;
    Arm_Joint Joint=SequenceJoints[Stage];
    if(Joint==ARM_SHOULDER || Joint==ARM_GRIPPER)
    {if(fabsf(Target[Joint]-State.CommandedAngleDeg[Joint])>0.001f)return;}
    else
    {
        State.Rate[Joint]=0;
        State.EstimatedContinuousDeg[Joint==ARM_ELBOW?0:1]=SequenceAngleTargets[Joint==ARM_ELBOW?0:1];
    }
    State.SequenceDoneMask|=(uint8_t)(1U<<Stage);
    if(State.SequenceDoneMask==0x0fU)
    {
        State.SequenceRunning=0;Halt();
        State.SequencePose=State.SequenceReturning?ARM_SEQUENCE_ZERO:ARM_SEQUENCE_DEPLOYED;
        return;
    }
    if(State.SequenceReturning)State.SequenceStage--;
    else State.SequenceStage++;
    BeginSequenceStage(NowMs);
}
uint8_t Arm_Request(const Arm_Command *Command,uint32_t CreatedMs,uint32_t NowMs)
{
    if(!State.Ready)return 0;
    if(!Arm_CommandValid(Command)) {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    if(Command->Action!=ARM_STOP && (uint32_t)(NowMs-CreatedMs)>=Command->DurationMs)
    {Arm_Stop();State.Fault=ARM_EXPIRED;return 0;}
    if(Command->Action==ARM_INITIALIZE)return StartSequence(NowMs);
    Arm_Stop();if(!State.Ready)return 0;
    State.Fault=ARM_OK;
    if(Command->Action==ARM_STOP)return 1;
    if(Command->Action==ARM_ZERO)
    {
        SetZero();State.Action=ARM_ZERO;return 1;
    }
    if(Command->Action==ARM_NEUTRAL)
    {
        Servos[Command->Joint].NeutralUs=(uint16_t)Command->Value;
        State.SelectedJoint=Command->Joint;State.Action=ARM_NEUTRAL;
        if(!Apply()) {HardwareFault();return 0;}
        return 1; // RAM calibration persists through stop/timeout, not through reboot.
    }
    if(Command->Action==ARM_PRESET || Command->Joint==ARM_SHOULDER || Command->Joint==ARM_ELBOW ||
       Command->Joint==ARM_WRIST_PITCH || Command->Joint==ARM_GRIPPER)
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
    }
    else
    {
        State.SelectedJoint=Command->Joint;
        if(Command->Action==ARM_POSITION)
        {
            if(!(State.PositionEnabledMask&(1U<<Command->Joint)))State.CommandedAngleDeg[Command->Joint]=Command->Value;
            State.PositionEnabledMask|=(uint8_t)(1U<<Command->Joint);
            Target[Command->Joint]=Command->Value;
        }
        else if(Command->Joint==ARM_BASE)State.BasePWM=(int16_t)(Command->Value*ROBOT_ARM_BASE_PWM_LIMIT*ROBOT_ARM_BASE_MOTOR_SIGN);
        else
        {
            float Rate=Command->Value;
            float Limit=Command->Joint==ARM_ELBOW?ROBOT_ARM_ELBOW_RATE_LIMIT:
                        Command->Joint==ARM_WRIST_PITCH?ROBOT_ARM_WRIST_PITCH_RATE_LIMIT:1.0f;
            if(Rate>Limit)Rate=Limit;
            if(Rate<-Limit)Rate=-Limit;
            State.Rate[Command->Joint]=Rate;
        }
    }
    return 1;
}
void Arm_ResetSampling(void) {LastCount=BSP_Arm_EncoderCount();}
void Arm_RemoteInput(float Base,Arm_Joint Joint,float Rate,uint8_t Initialize,uint8_t Gripper,uint32_t NowMs)
{
    if(!State.Ready)return;
    if(!isfinite(Base) || !isfinite(Rate) || fabsf(Base)>1 || fabsf(Rate)>1 || Joint>ARM_WRIST_ROLL || (int)Joint<0)
    {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return;}
    if(Initialize && Base==0 && Rate==0){(void)StartSequence(NowMs);return;}
    if(State.SequenceRunning)
    {
        if(Base==0 && Rate==0)return; // Ignore gripper presses while the sequence owns it.
        Arm_Stop(); // Manual override cancels the sequence; reference is now unknown.
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
    State.BasePWM=(int16_t)(Base*ROBOT_ARM_BASE_PWM_LIMIT*ROBOT_ARM_BASE_MOTOR_SIGN);
    State.Rate[Joint]=Rate*(Joint==ARM_ELBOW?ROBOT_ARM_ELBOW_RATE_LIMIT:
                           Joint==ARM_WRIST_PITCH?ROBOT_ARM_WRIST_PITCH_RATE_LIMIT:1.0f);
    if(Rate!=0 && Servos[Joint].Mode==SERVO_POSITION)State.PositionEnabledMask|=(uint8_t)(1U<<Joint);
    if(Gripper)
    {
        Target[ARM_GRIPPER]=Target[ARM_GRIPPER]==ROBOT_ARM_GRIPPER_OPEN_DEG?
                           ROBOT_ARM_GRIPPER_CLOSED_DEG:ROBOT_ARM_GRIPPER_OPEN_DEG;
        State.PositionEnabledMask|=(uint8_t)(1U<<ARM_GRIPPER);
    }
}
void Arm_Step(float DtS,uint32_t NowMs)
{
    if(!State.Ready)return;
    State.TickMs=NowMs;
    uint16_t Count=BSP_Arm_EncoderCount();
    uint16_t Raw=(uint16_t)(Count-LastCount);LastCount=Count;
    int32_t Delta=Raw>=32768?(int32_t)Raw-65536:Raw;
    Delta*=ROBOT_ARM_BASE_ENCODER_SIGN;
    State.BaseCounts+=Delta;
    State.BaseRelativeAngleDeg=(float)State.BaseCounts*360.0f/ROBOT_ARM_BASE_CPR;
    if(!isfinite(DtS) || DtS<=0 || DtS>ROBOT_ARM_MAX_SAMPLE_S)
    {State.BaseRpm=0;Arm_Stop();State.Fault=ARM_BAD_SAMPLE;return;}
    State.BaseRpm=(float)Delta*60.0f/(ROBOT_ARM_BASE_CPR*DtS);
    State.EstimatedContinuousDeg[0]+=State.Rate[ARM_ELBOW]*ROBOT_ARM_ELBOW_REFERENCE_DPS/ROBOT_ARM_ELBOW_RATE_LIMIT*DtS;
    State.EstimatedContinuousDeg[1]+=State.Rate[ARM_WRIST_PITCH]*ROBOT_ARM_WRIST_PITCH_REFERENCE_DPS/ROBOT_ARM_WRIST_PITCH_RATE_LIMIT*DtS;
    if(State.SequenceRunning)SequenceStep(NowMs);
    else if(State.Active && (uint32_t)(NowMs-Started)>=Duration)
    {Arm_Stop();State.Fault=ARM_EXPIRED;}
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
    {
        if(Servos[i].Mode!=SERVO_POSITION)continue;
        if(State.Active && (State.Action==ARM_RATE || State.Action==ARM_REMOTE) && State.SelectedJoint==i)
        {
            Target[i]=State.CommandedAngleDeg[i]+State.Rate[i]*ROBOT_ARM_POSITION_RATE_DPS*DtS;
            if(Target[i]<0)Target[i]=0;
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
