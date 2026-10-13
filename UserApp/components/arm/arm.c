#include "components/arm/arm.h"
#include "robot/robot_config.h"
#include "actuator/servo.h"
#include "arm/bsp_arm.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static const Servo_Config Servos[ARM_SERVO_COUNT]=ROBOT_ARM_SERVO_CONFIG;
static const float Preset[ARM_SERVO_COUNT]=ROBOT_ARM_PRESET_DEG;
static Arm_Status State;
static float Target[ARM_SERVO_COUNT];
static uint32_t Started,Duration;
static uint16_t LastCount;
_Static_assert(ARM_SERVO_COUNT==BSP_ARM_SERVO_COUNT,"Arm servo mapping mismatch");
_Static_assert(ROBOT_ARM_BASE_PWM_LIMIT>0 && ROBOT_ARM_BASE_PWM_LIMIT<=BSP_ARM_BASE_PWM_MAX,"Arm base PWM limit invalid");
_Static_assert(ROBOT_ARM_BASE_MOTOR_SIGN==1 || ROBOT_ARM_BASE_MOTOR_SIGN==-1,"Arm motor polarity invalid");
_Static_assert(ROBOT_ARM_BASE_ENCODER_SIGN==1 || ROBOT_ARM_BASE_ENCODER_SIGN==-1,"Arm encoder polarity invalid");
static uint8_t Apply(void)
{
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
        if(!Servo_Pulse(&Servos[i],Servos[i].Mode==SERVO_POSITION?State.CommandedAngleDeg[i]:State.Rate[i],&State.PulseUs[i]))return 0;
    return BSP_Arm_Apply(State.PulseUs,State.BasePWM);
}
static void Halt(void)
{
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++) {State.Rate[i]=0;Target[i]=State.CommandedAngleDeg[i];}
    State.BasePWM=0;State.Active=0;State.Action=ARM_STOP;
}
static void HardwareFault(void)
{
    Arm_Disable();State.Fault=ARM_HARDWARE_ERROR;
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
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
    {
        State.CommandedAngleDeg[i]=Target[i]=Preset[i];
        if(!Servo_Pulse(&Servos[i],Servos[i].Mode==SERVO_POSITION?Preset[i]:0,&State.PulseUs[i]))
        {HardwareFault();return 0;}
    }
    if(!isfinite(ROBOT_ARM_POSITION_RATE_DPS) || ROBOT_ARM_POSITION_RATE_DPS<=0 ||
       !isfinite(ROBOT_ARM_MAX_SAMPLE_S) || ROBOT_ARM_MAX_SAMPLE_S<=0 ||
       !isfinite(ROBOT_ARM_BASE_CPR) || ROBOT_ARM_BASE_CPR<=0 ||
       !BSP_Arm_Init(State.PulseUs)) {HardwareFault();return 0;}
    LastCount=BSP_Arm_EncoderCount();State.Ready=1;State.Fault=ARM_OK;return 1;
}
uint8_t Arm_CommandValid(const Arm_Command *Command)
{
    if(Command==NULL)return 0;
    if(Command->Action==ARM_STOP)return 1;
    if(Command->DurationMs==0 || Command->DurationMs>ROBOT_ARM_COMMAND_MAX_MS)return 0;
    if(Command->Action==ARM_PRESET)return 1;
    if(Command->Joint>=ARM_JOINT_COUNT || (int)Command->Joint<0 || !isfinite(Command->Value))return 0;
    if(Command->Action==ARM_RATE)return Command->Value>=-1 && Command->Value<=1;
    if(Command->Action==ARM_POSITION)
        return Command->Joint<ARM_SERVO_COUNT && Servos[Command->Joint].Mode==SERVO_POSITION &&
               Command->Value>=0 && Command->Value<=Servos[Command->Joint].MaxAngleDeg;
    return 0;
}
uint8_t Arm_Request(const Arm_Command *Command,uint32_t CreatedMs,uint32_t NowMs)
{
    if(!State.Ready)return 0;
    if(!Arm_CommandValid(Command)) {Arm_Stop();State.Fault=ARM_INVALID_COMMAND;return 0;}
    if(Command->Action!=ARM_STOP && (uint32_t)(NowMs-CreatedMs)>=Command->DurationMs)
    {Arm_Stop();State.Fault=ARM_EXPIRED;return 0;}
    Arm_Stop();if(!State.Ready)return 0;
    State.Fault=ARM_OK;
    if(Command->Action==ARM_STOP)return 1;
    Started=CreatedMs;Duration=Command->DurationMs;
    State.Action=Command->Action;State.Active=1;
    if(Command->Action==ARM_PRESET)memcpy(Target,Preset,sizeof(Target));
    else
    {
        State.SelectedJoint=Command->Joint;
        if(Command->Action==ARM_POSITION)Target[Command->Joint]=Command->Value;
        else if(Command->Joint==ARM_BASE)State.BasePWM=(int16_t)(Command->Value*ROBOT_ARM_BASE_PWM_LIMIT*ROBOT_ARM_BASE_MOTOR_SIGN);
        else State.Rate[Command->Joint]=Command->Value;
    }
    return 1;
}
void Arm_ResetSampling(void) {LastCount=BSP_Arm_EncoderCount();}
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
    if(State.Active && (uint32_t)(NowMs-Started)>=Duration)
    {Arm_Stop();State.Fault=ARM_EXPIRED;}
    for(uint8_t i=0;i<ARM_SERVO_COUNT;i++)
    {
        if(Servos[i].Mode!=SERVO_POSITION)continue;
        if(State.Active && State.Action==ARM_RATE && State.SelectedJoint==i)
        {
            Target[i]=State.CommandedAngleDeg[i]+State.Rate[i]*ROBOT_ARM_POSITION_RATE_DPS*DtS;
            if(Target[i]<0)Target[i]=0;
            if(Target[i]>Servos[i].MaxAngleDeg)Target[i]=Servos[i].MaxAngleDeg;
        }
        float Error=Target[i]-State.CommandedAngleDeg[i],Step=ROBOT_ARM_POSITION_RATE_DPS*DtS;
        if(Error>Step)Error=Step;
        if(Error<-Step)Error=-Step;
        State.CommandedAngleDeg[i]+=Error;
    }
    if(!Apply())HardwareFault();
}
Arm_Status Arm_GetStatus(void) {return State;}
