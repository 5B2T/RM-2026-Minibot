//
// Created by Admin on 2026/10/2.
//

#ifndef RM_2026_MINIBOT_MOTORCONTROL_H
#define RM_2026_MINIBOT_MOTORCONTROL_H
#include <stdint.h>
#include "../Driver/Motor.h"
#include "Speed.h"

#define MOTOR_CONTROL_KP 600.0f
#define MOTOR_CONTROL_KI 80.0f
#define MOTOR_CONTROL_SAMPLE_TIME 0.01f
#define MOTOR_CONTROL_OUTPUT_MAX 999.0f

typedef struct
{
    float MotorControl_Target;
    float MotorControl_Kp;
    float MotorControl_Ki;
    float MotorControl_I;
    float MotorControl_Out;
}MotorControl_HandleTypeDef;

extern MotorControl_HandleTypeDef MotorControl[4];

void MotorControl_Init(void);
void MotorControl_SetTarget(float M1_Target, float M2_Target, float M3_Target, float M4_Target);
void MotorControl_Update(void);
void MotorControl_Stop(void);

#endif //RM_2026_MINIBOT_MOTORCONTROL_H
