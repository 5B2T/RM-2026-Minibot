#include "components/chassis/chassis.h"
#include "robot/robot_config.h"
#include "wheel/speed.h"
#include "wheel/motor_control.h"
#include "motor/bsp_motor.h"
#include <stddef.h>
_Static_assert(ROBOT_WHEEL_COUNT==WHEEL_COUNT, "Command and wheel count must match");
_Static_assert(ROBOT_CHANNEL_FL<4 && ROBOT_CHANNEL_FR<4 && ROBOT_CHANNEL_RL<4 && ROBOT_CHANNEL_RR<4,
               "Wheel channel out of range");
_Static_assert(((1U<<ROBOT_CHANNEL_FL)|(1U<<ROBOT_CHANNEL_FR)|(1U<<ROBOT_CHANNEL_RL)|(1U<<ROBOT_CHANNEL_RR))==0x0fU,
               "Wheel channel mapping must be a permutation");
uint8_t Chassis_Init(void)
{
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
}
void Chassis_Control(const Robot_WheelCommand *Command, float DtS)
{
    if(Command==NULL || !Command->Enabled) {Chassis_Stop();return;}
    MotorControl_SetTarget(Command->WheelSpeed[0], Command->WheelSpeed[1],
                           Command->WheelSpeed[2], Command->WheelSpeed[3]);
    MotorControl_Update(DtS);
}
void Chassis_GetFeedback(Robot_Status *Status)
{
    if(Status==NULL)return;
    Status->MotorsEnabled=MotorControl_IsEnabled();
    for(uint8_t i=0;i<ROBOT_WHEEL_COUNT;i++)
    {
        MotorControl_HandleTypeDef Wheel=MotorControl_GetState(i);
        Status->MeasuredSpeed[i]=MotorControl_GetMeasuredSpeed(i);
        Status->TargetSpeed[i]=Wheel.MotorControl_Target;
        Status->PWM[i]=Wheel.MotorControl_Out;
    }
}
