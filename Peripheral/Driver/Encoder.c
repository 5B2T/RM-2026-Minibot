//
// Created by Admin on 2026/9/30.
//

#include "Encoder.h"

Encoder_HandleTypeDef Encoder1;
Encoder_HandleTypeDef Encoder2;
Encoder_HandleTypeDef Encoder3;
Encoder_HandleTypeDef Encoder4;

void Encoder_Init(void)
{
    Encoder1 = (Encoder_HandleTypeDef){
        &htim5,
        0,
        0
    };
    Encoder2 = (Encoder_HandleTypeDef){
        &htim3,
        0,
        0
    };
    Encoder3 = (Encoder_HandleTypeDef){
        &htim2,
        0,
        0
    };
    Encoder4 = (Encoder_HandleTypeDef){
        &htim8,
        0,
        0
    };

    HAL_TIM_Encoder_Start(Encoder1.Encoder_TIMx, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(Encoder2.Encoder_TIMx, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(Encoder3.Encoder_TIMx, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(Encoder4.Encoder_TIMx, TIM_CHANNEL_ALL);
}

uint16_t Encoder_GetCount(Encoder_HandleTypeDef* Encoderx)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(Encoderx->Encoder_TIMx);
}

int16_t Encoder_GetDelta(Encoder_HandleTypeDef* Encoderx)
{
    uint16_t Count=Encoder_GetCount(Encoderx);
    uint16_t RawDelta=(uint16_t)(Count-Encoderx->Encoder_LastCount);
    int32_t Delta=RawDelta;
    // All four timers use ARR=65535. Motion must be less than 32768 counts/sample.
    if(RawDelta>=0x8000U)Delta-=0x10000L;

    Encoderx->Encoder_LastCount = Count;
    Encoderx->Encoder_Delta = (int16_t)Delta;

    return Encoderx->Encoder_Delta;
}

void Encoder_Reset(Encoder_HandleTypeDef* Encoderx)
{
    __HAL_TIM_SET_COUNTER(Encoderx->Encoder_TIMx, 0);
    Encoderx->Encoder_LastCount = 0;
    Encoderx->Encoder_Delta = 0;
}
