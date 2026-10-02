//
// Created by Admin on 2026/10/2.
//

#include "Speed.h"

Speed_HandleTypeDef Speed[4];

void Speed_Init(void)
{
    Encoder_HandleTypeDef* Encoder[4]={&Encoder1,&Encoder2,&Encoder3,&Encoder4};

    for(uint8_t t=0;t<4;t++)
    {
        Speed[t]=(Speed_HandleTypeDef){
            Encoder[t],
            SPEED_CPR,
            SPEED_RATIO,
            SPEED_DIA,
            SPEED_SAMPLE_TIME,
            0,
            0
        };
    }
}

static float Speed_Calculate(Speed_HandleTypeDef* Speedx, int16_t Delta)
{
    return ((float)Delta * 3.1415926f*Speedx->Speed_WheelDia)
        / (Speedx->Speed_EncoderCPR*Speedx->Speed_Ratio*Speedx->Speed_SampleTime);
}

void Speed_Update(void)
{
    for(uint8_t t=0;t<4;t++)
    {
        Speed[t].Speed_Delta=Encoder_GetDelta(Speed[t].Speed_Encoderx);
        Speed[t].Speed_Value=Speed_Calculate(&Speed[t],Speed[t].Speed_Delta);
    }
}

void Speed_SetEncoderCPR(float EncoderCPR)
{
    for(uint8_t t=0;t<4;t++)
    {
        Speed[t].Speed_EncoderCPR=EncoderCPR;
    }
}

float Speed_Get(Speed_HandleTypeDef* Speedx)
{
    return Speedx->Speed_Value;
}

int16_t Speed_GetDelta(Speed_HandleTypeDef* Speedx)
{
    return Speedx->Speed_Delta;
}
