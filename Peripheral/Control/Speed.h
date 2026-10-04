//
// Created by Admin on 2026/10/2.
//

#ifndef RM_2026_MINIBOT_SPEED_H
#define RM_2026_MINIBOT_SPEED_H
#include <stdint.h>
#include "../Driver/Encoder.h"


#define SPEED_DIA 0.060f
#define SPEED_SAMPLE_PERIOD_MS 10U
#define SPEED_SAMPLE_TIME ((float)SPEED_SAMPLE_PERIOD_MS/1000.0f)
// Counts per output-shaft revolution, already including reduction and x4 decoding.
#define SPEED_CPR 1560.0f

typedef struct
{
    Encoder_HandleTypeDef* Speed_Encoderx;
    float Speed_EncoderCPR;
    float Speed_WheelDia;
    float Speed_SampleTime;
    float Speed_Value;
    int16_t Speed_Delta;
}Speed_HandleTypeDef;

extern Speed_HandleTypeDef Speed[4];

void Speed_Init(void);
void Speed_Reset(void);
uint8_t Speed_Update(float SampleTime);
void Speed_SetEncoderCPR(float EncoderCPR);
float Speed_Get(Speed_HandleTypeDef* Speedx);
int16_t Speed_GetDelta(Speed_HandleTypeDef* Speedx);

#endif //RM_2026_MINIBOT_SPEED_H
