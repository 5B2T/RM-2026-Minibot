//
// Created by Admin on 2026/10/2.
//

#include "MotorControl.h"

MotorControl_HandleTypeDef MotorControl[4];

static float MotorControl_Limit(float Value)
{
    if(Value>MOTOR_CONTROL_OUTPUT_MAX)Value=MOTOR_CONTROL_OUTPUT_MAX;
    if(Value<-MOTOR_CONTROL_OUTPUT_MAX)Value=-MOTOR_CONTROL_OUTPUT_MAX;
    return Value;
}

void MotorControl_Init(void)
{
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

    for(uint8_t t=0;t<4;t++)
    {
        MotorControl[t].MotorControl_Target=Target[t];
    }
}

void MotorControl_Update(void)
{
    int MotorPWM[4];

    for(uint8_t t=0;t<4;t++)
    {
        float Err=MotorControl[t].MotorControl_Target-Speed_Get(&Speed[t]);
        float I=MotorControl[t].MotorControl_I+Err*MOTOR_CONTROL_SAMPLE_TIME;
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
}

void MotorControl_Stop(void)
{
    for(uint8_t t=0;t<4;t++)
    {
        MotorControl[t].MotorControl_Target=0;
        MotorControl[t].MotorControl_I=0;
        MotorControl[t].MotorControl_Out=0;
    }

    int Motor_StopPWM[4]={0,0,0,0};

    Motor_Load(Motor_StopPWM);
}
