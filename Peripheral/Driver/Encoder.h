//
// Created by Admin on 2026/9/30.
//

#ifndef RM_2026_MINIBOT_ENCODER_H
#define RM_2026_MINIBOT_ENCODER_H

//编码器AB相：PA0/1 PA6/7 PA5/PB3 PC6/7
#include <stdint.h>
#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef* Encoder_TIMx;
    int16_t Encoder_LastCount;
    int16_t Encoder_Delta;
}Encoder_HandleTypeDef;

extern Encoder_HandleTypeDef Encoder1;
extern Encoder_HandleTypeDef Encoder2;
extern Encoder_HandleTypeDef Encoder3;
extern Encoder_HandleTypeDef Encoder4;

void Encoder_Init(void);
int16_t Encoder_GetCount(Encoder_HandleTypeDef* Encoderx);
int16_t Encoder_GetDelta(Encoder_HandleTypeDef* Encoderx);
void Encoder_Reset(Encoder_HandleTypeDef* Encoderx);

#endif //RM_2026_MINIBOT_ENCODER_H
