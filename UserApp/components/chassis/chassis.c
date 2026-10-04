#include "components/chassis/chassis.h"
#include "robot/robot_config.h"
#include "wheel/speed.h"
#include "wheel/motor_control.h"
#include "motor/bsp_motor.h"
#include "algorithm/mecanum.h"
#include <string.h>
#include <stddef.h>
_Static_assert(ROBOT_WHEEL_COUNT==WHEEL_COUNT, "Command and wheel count must match");
_Static_assert(ROBOT_CHANNEL_FL<4 && ROBOT_CHANNEL_FR<4 && ROBOT_CHANNEL_RL<4 && ROBOT_CHANNEL_RR<4,
               "Wheel channel out of range");
_Static_assert(((1U<<ROBOT_CHANNEL_FL)|(1U<<ROBOT_CHANNEL_FR)|(1U<<ROBOT_CHANNEL_RL)|(1U<<ROBOT_CHANNEL_RR))==0x0fU,
               "Wheel channel mapping must be a permutation");
static const uint8_t WheelChannels[ROBOT_WHEEL_COUNT]=ROBOT_WHEEL_CHANNEL_MAP;
static Robot_CommandMode ActiveMode;
static uint8_t Running;
_Static_assert(ROBOT_WHEEL_FL==0 && ROBOT_WHEEL_FR==1 && ROBOT_WHEEL_RL==2 && ROBOT_WHEEL_RR==3 &&
               ROBOT_WHEEL_COUNT==MECANUM_WHEEL_COUNT, "Mecanum wheel order must match");
uint8_t Chassis_Init(void)
{
    ActiveMode=ROBOT_COMMAND_WHEELS;
    Running=0;
    float Wheels[ROBOT_WHEEL_COUNT];
    if(!Mecanum_Inverse(0,0,0,ROBOT_CHASSIS_WHEELBASE_M,ROBOT_CHASSIS_TRACK_M,
                        ROBOT_WHEEL_SPEED_LIMIT_MPS,Wheels)) {Chassis_Stop();return 0;}
    static const int8_t MotorSign[4]=ROBOT_MOTOR_POLARITY;
    static const int8_t EncoderSign[4]=ROBOT_ENCODER_POLARITY;
    return Speed_Init(ROBOT_WHEEL_DIAMETER_M, ROBOT_ENCODER_CPR) &&
           MotorControl_Init(ROBOT_SPEED_KP, ROBOT_SPEED_KI,
                             BSP_MOTOR_PWM_MAX, ROBOT_WHEEL_SPEED_LIMIT_MPS) &&
           MotorControl_SetPolarity(MotorSign, EncoderSign);
}
void Chassis_ResetSampling(void)
{
    Speed_Reset();
}
uint8_t Chassis_Measure(float DtS)
{
    return Speed_Update(DtS);
}
void Chassis_Stop(void)
{
    MotorControl_Stop();
    Running=0;
}
static void SelectMode(Robot_CommandMode Mode)
{
    // Avoid carrying integral state between direct wheel testing and body control.
    if(Running && ActiveMode!=Mode)MotorControl_Stop();
    ActiveMode=Mode;
}
static void ApplyWheels(const float Wheels[ROBOT_WHEEL_COUNT], float DtS)
{
    MotorControl_SetTarget(Wheels[0],Wheels[1],Wheels[2],Wheels[3]);
    MotorControl_Update(DtS);
    Running=MotorControl_IsEnabled();
}
void Chassis_ControlWheels(const Robot_WheelCommand *Command, float DtS)
{
    if(Command==NULL || !Command->Enabled) {Chassis_Stop();return;}
    SelectMode(ROBOT_COMMAND_WHEELS);
    ApplyWheels(Command->WheelSpeed, DtS);
}
void Chassis_ControlVelocity(const Robot_ChassisVelocity *Velocity, uint8_t Enabled, float DtS)
{
    if(Velocity==NULL || !Enabled) {Chassis_Stop();return;}
    float Wheels[ROBOT_WHEEL_COUNT],Channels[ROBOT_WHEEL_COUNT];
    if(!Mecanum_Inverse(Velocity->Vx,Velocity->Vy,Velocity->Wz,
                        ROBOT_CHASSIS_WHEELBASE_M,ROBOT_CHASSIS_TRACK_M,
                        ROBOT_WHEEL_SPEED_LIMIT_MPS,Wheels)) {Chassis_Stop();return;}
    for(uint8_t i=0;i<ROBOT_WHEEL_COUNT;i++)Channels[WheelChannels[i]]=Wheels[i];
    SelectMode(ROBOT_COMMAND_VELOCITY);
    ApplyWheels(Channels, DtS);
}
void Chassis_Control(const Robot_ChassisCommand *Command, float DtS)
{
    if(Command==NULL || !Command->Enabled) {Chassis_Stop();return;}
    if(Command->Mode==ROBOT_COMMAND_VELOCITY)
        Chassis_ControlVelocity(&Command->Target.Velocity, 1, DtS);
    else if(Command->Mode==ROBOT_COMMAND_WHEELS)
    {
        Robot_WheelCommand Wheels={.Enabled=1};
        memcpy(Wheels.WheelSpeed,Command->Target.Wheels,sizeof(Wheels.WheelSpeed));
        Chassis_ControlWheels(&Wheels, DtS);
    }
    else Chassis_Stop();
}
void Chassis_GetFeedback(Robot_Status *Status)
{
    if(Status==NULL)return;
    Status->CommandMode=ActiveMode;
    Status->MotorsEnabled=MotorControl_IsEnabled();
    for(uint8_t i=0;i<ROBOT_WHEEL_COUNT;i++)
    {
        MotorControl_HandleTypeDef Wheel=MotorControl_GetState(i);
        Status->MeasuredSpeed[i]=MotorControl_GetMeasuredSpeed(i);
        Status->TargetSpeed[i]=Wheel.MotorControl_Target;
        Status->PWM[i]=Wheel.MotorControl_Out;
    }
}
