//
// Created by Admin on 2026/10/2.
//

#include "Speed.h"
#include <math.h>

Speed_HandleTypeDef Speed[4];

void Speed_Init(void)
{
    Encoder_HandleTypeDef* Encoder[4]={&Encoder1,&Encoder2,&Encoder3,&Encoder4};

    for(uint8_t t=0;t<4;t++)
    {
        Speed[t]=(Speed_HandleTypeDef){
            Encoder[t],
            SPEED_CPR,
            SPEED_DIA,
            0,
            0,
            0
        };
    }
    Speed_Reset();
}

void Speed_Reset(void)
{
    for(uint8_t t=0;t<4;t++)
    {
        Encoder_HandleTypeDef* Encoderx=Speed[t].Speed_Encoderx;
        Encoderx->Encoder_LastCount=Encoder_GetCount(Encoderx);
        Encoderx->Encoder_Delta=0;
        Speed[t].Speed_SampleTime=0;
        Speed[t].Speed_Value=0;
        Speed[t].Speed_Delta=0;
    }
}

static float Speed_Calculate(Speed_HandleTypeDef* Speedx, int16_t Delta)
{
    return ((float)Delta * 3.1415926f*Speedx->Speed_WheelDia)
        / (Speedx->Speed_EncoderCPR*Speedx->Speed_SampleTime);
}

uint8_t Speed_Update(float SampleTime)
{
    if(!isfinite(SampleTime) || SampleTime<=0)return 0;
    for(uint8_t t=0;t<4;t++)
    {
        Speed[t].Speed_SampleTime=SampleTime;
        Speed[t].Speed_Delta=Encoder_GetDelta(Speed[t].Speed_Encoderx);
        Speed[t].Speed_Value=Speed_Calculate(&Speed[t],Speed[t].Speed_Delta);
    }
    return 1;
}

void Speed_SetEncoderCPR(float EncoderCPR)
{
    if(!isfinite(EncoderCPR) || EncoderCPR<=0)return;
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
