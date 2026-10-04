#ifndef WHEEL_MOTOR_CONTROL_H
#define WHEEL_MOTOR_CONTROL_H
#include <stdint.h>

typedef struct
{
    float MotorControl_Target;
    float MotorControl_Kp;
    float MotorControl_Ki;
    float MotorControl_I;
    float MotorControl_Out;
}MotorControl_HandleTypeDef;

uint8_t MotorControl_Init(float Kp, float Ki, float OutputLimit, float TargetLimitMps);
void MotorControl_SetTarget(float M1_Target, float M2_Target, float M3_Target, float M4_Target);
void MotorControl_Update(float SampleTime);
void MotorControl_Stop(void);
uint8_t MotorControl_IsEnabled(void);

// Control-task only; copied state for publication by the robot layer.
MotorControl_HandleTypeDef MotorControl_GetState(uint8_t Wheel);

#endif //WHEEL_MOTOR_CONTROL_H
