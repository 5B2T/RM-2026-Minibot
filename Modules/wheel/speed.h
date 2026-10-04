#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H
#include <stdint.h>
#define WHEEL_COUNT 4U
typedef struct
{
    float Speed_EncoderCPR;
    float Speed_WheelDia;
    float Speed_SampleTime;
    float Speed_Value; // Wheel-rim m/s, with original encoder polarity.
    int16_t Speed_Delta;
} Speed_HandleTypeDef;
// Control-task owner. State getters return values, never writable global arrays.
uint8_t Speed_Init(float WheelDiameterM, float CountsPerRevolution);
void Speed_Reset(void);
uint8_t Speed_Update(float SampleTimeS);
void Speed_SetEncoderCPR(float CountsPerRevolution);
float Speed_Get(uint8_t Wheel);
int16_t Speed_GetDelta(uint8_t Wheel);
Speed_HandleTypeDef Speed_GetState(uint8_t Wheel);
#endif
