//
// Created by Admin on 2026/10/2.
//

#include "MotorControl.h"
#include <math.h>

MotorControl_HandleTypeDef MotorControl[4];
static uint8_t MotorControl_Enabled;

static float MotorControl_Limit(float Value)
{
    if(Value>MOTOR_CONTROL_OUTPUT_MAX)Value=MOTOR_CONTROL_OUTPUT_MAX;
    if(Value<-MOTOR_CONTROL_OUTPUT_MAX)Value=-MOTOR_CONTROL_OUTPUT_MAX;
    return Value;
}

void MotorControl_Init(void)
{
    MotorControl_Enabled=0;
    for(uint8_t t=0;t<4;t++)
    {
        MotorControl[t]=(MotorControl_HandleTypeDef){
            0,
            MOTOR_CONTROL_KP,
            MOTOR_CONTROL_KI,
            0,
            0
        };
    }
}

void MotorControl_SetTarget(float M1_Target, float M2_Target, float M3_Target, float M4_Target)
{
    float Target[4]={M1_Target, M2_Target, M3_Target, M4_Target};
    float Peak=0;
    for(uint8_t t=0;t<4;t++)
    {
        if(!isfinite(Target[t]))
        {
            MotorControl_Stop();
            return;
        }
        if(fabsf(Target[t])>Peak)Peak=fabsf(Target[t]);
    }
    float Scale=Peak>MOTOR_CONTROL_TARGET_MAX ? MOTOR_CONTROL_TARGET_MAX/Peak : 1.0f;
    for(uint8_t t=0;t<4;t++)
    {
        MotorControl[t].MotorControl_Target=Target[t]*Scale;
    }
    MotorControl_Enabled=1;
}

void MotorControl_Update(float SampleTime)
{
    if(!MotorControl_Enabled || !isfinite(SampleTime) || SampleTime<=0)
    {
        MotorControl_Stop();
        return;
    }
    int MotorPWM[4];
    for(uint8_t t=0;t<4;t++)
    {
        if(!isfinite(Speed_Get(&Speed[t])))
        {
            MotorControl_Stop();
            return;
        }
    }

    for(uint8_t t=0;t<4;t++)
    {
        float Err=MotorControl[t].MotorControl_Target-Speed_Get(&Speed[t]);
        float I=MotorControl[t].MotorControl_I+Err*SampleTime;
        float Output=MotorControl[t].MotorControl_Kp*Err+MotorControl[t].MotorControl_Ki*I;

        Output=MotorControl_Limit(Output);

        if((Output>=MOTOR_CONTROL_OUTPUT_MAX&&Err>0)||(Output<=-MOTOR_CONTROL_OUTPUT_MAX&&Err<0))
        {
            I=MotorControl[t].MotorControl_I;
        }

        MotorControl[t].MotorControl_I=I;
        MotorControl[t].MotorControl_Out=Output;
        MotorPWM[t]=(int)Output;
    }

    Motor_Load(MotorPWM);
    Motor_Enable();
}

void MotorControl_Stop(void)
{
    MotorControl_Enabled=0;
    for(uint8_t t=0;t<4;t++)
    {
        MotorControl[t].MotorControl_Target=0;
        MotorControl[t].MotorControl_I=0;
        MotorControl[t].MotorControl_Out=0;
    }

    Motor_Disable();
}
