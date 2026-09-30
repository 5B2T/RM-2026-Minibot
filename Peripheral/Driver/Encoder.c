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

int16_t Encoder_GetCount(Encoder_HandleTypeDef* Encoderx)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(Encoderx->Encoder_TIMx);
}

int16_t Encoder_GetDelta(Encoder_HandleTypeDef* Encoderx)
{
    int16_t Count=Encoder_GetCount(Encoderx); 
    int16_t Delta=Count-Encoderx->Encoder_LastCount;

    Encoderx->Encoder_LastCount = Count;
    Encoderx->Encoder_Delta = Delta;

    return Delta;
}

void Encoder_Reset(Encoder_HandleTypeDef* Encoderx)
{
    __HAL_TIM_SET_COUNTER(Encoderx->Encoder_TIMx, 0);
    Encoderx->Encoder_LastCount = 0;
    Encoderx->Encoder_Delta = 0;
}
