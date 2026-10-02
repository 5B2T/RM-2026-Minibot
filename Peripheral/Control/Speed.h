//
// Created by Admin on 2026/10/2.
//

#ifndef RM_2026_MINIBOT_SPEED_H
#define RM_2026_MINIBOT_SPEED_H
#include <stdint.h>
#include "../Driver/Encoder.h"


#define SPEED_RATIO 30.0f
#define SPEED_DIA 0.075f
#define SPEED_SAMPLE_TIME 0.01f
#define SPEED_CPR 1560f

typedef struct
{
    Encoder_HandleTypeDef* Speed_Encoderx;
    float Speed_EncoderCPR;
    float Speed_Ratio;
    float Speed_WheelDia;
    float Speed_SampleTime;
    float Speed_Value;
    int16_t Speed_Delta;
}Speed_HandleTypeDef;

extern Speed_HandleTypeDef Speed[4];

void Speed_Init(void);
void Speed_Update(void);
void Speed_SetEncoderCPR(float EncoderCPR);
float Speed_Get(Speed_HandleTypeDef* Speedx);
int16_t Speed_GetDelta(Speed_HandleTypeDef* Speedx);

#endif //RM_2026_MINIBOT_SPEED_H
