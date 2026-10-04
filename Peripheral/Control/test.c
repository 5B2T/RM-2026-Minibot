//
// Created by Admin on 2026/10/3.
//

#include "test.h"
#include "../Driver/Motor.h"
#include "../Driver/Encoder.h"
#include "../Driver/NRF24L01.h"
#include "Speed.h"
#include "MotorControl.h"

uint8_t Test_Motor(void)
{
    MotorControl_Stop();
    for(uint8_t t=0;t<4;t++)
    {
        if(Motor[t].Motor_PWM != 0)return 0;
    }
    return 1;
}

uint8_t Test_Encoder(void)
{
    Encoder_Reset(&Encoder1);
    Encoder_Reset(&Encoder2);
    Encoder_Reset(&Encoder3);
    Encoder_Reset(&Encoder4);
    return Encoder_GetCount(&Encoder1) == 0 && Encoder_GetCount(&Encoder2) == 0 &&
           Encoder_GetCount(&Encoder3) == 0 && Encoder_GetCount(&Encoder4) == 0;
}

uint8_t Test_Speed(void)
{
    if(!Speed_Update(SPEED_SAMPLE_TIME))return 0;
    for(uint8_t t=0;t<4;t++)
    {
        if(Speed[t].Speed_Encoderx == 0)return 0;
    }
    return 1;
}

uint8_t Test_MotorControl(void)
{
    MotorControl_SetTarget(0, 0, 0, 0);
    MotorControl_Update(SPEED_SAMPLE_TIME);
    for(uint8_t t=0;t<4;t++)
    {
        if(MotorControl[t].MotorControl_Target != 0)return 0;
    }
    return 1;
}

uint8_t Test_NRF24L01(void)
{
    return NRF24L01_Check() == 0;
}

uint8_t Test_Run(void)
{
    uint8_t result = 0;
    if(Test_Motor())result |= TEST_MOTOR;
    if(Test_Encoder())result |= TEST_ENCODER;
    if(Test_Speed())result |= TEST_SPEED;
    if(Test_MotorControl())result |= TEST_MOTOR_CONTROL;
    if(Test_NRF24L01())result |= TEST_NRF24L01;
    return result;
}
