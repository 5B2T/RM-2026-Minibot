#include "components/chassis/chassis.h"
#include "robot/robot_config.h"
#include "wheel/speed.h"
#include "wheel/motor_control.h"
#include "motor/bsp_motor.h"
#include <stddef.h>
_Static_assert(ROBOT_WHEEL_COUNT==WHEEL_COUNT, "Command and wheel count must match");
uint8_t Chassis_Init(void)
{
    return Speed_Init(ROBOT_WHEEL_DIAMETER_M, ROBOT_ENCODER_CPR) &&
           MotorControl_Init(ROBOT_SPEED_KP, ROBOT_SPEED_KI,
                             BSP_MOTOR_PWM_MAX, ROBOT_WHEEL_SPEED_LIMIT_MPS);
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
        Status->MeasuredSpeed[i]=Speed_Get(i);
        Status->TargetSpeed[i]=Wheel.MotorControl_Target;
        Status->PWM[i]=Wheel.MotorControl_Out;
    }
}
