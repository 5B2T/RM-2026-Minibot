#ifndef TEST_TIM_H
#define TEST_TIM_H
#include "main.h"
typedef struct {uint32_t Ccr[4],Count,Running;} TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim1,htim2,htim3,htim4,htim5,htim8,htim9,htim10,htim11,htim12,htim13;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
#define TIM_CHANNEL_ALL 0xfU
#define HAL_OK 0
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Ccr[c]=(v))
#define __HAL_TIM_GET_COMPARE(t,c) ((t)->Ccr[c])
#define __HAL_TIM_GET_COUNTER(t) ((t)->Count)
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *,uint32_t);
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *,uint32_t);
int HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *,uint32_t);
#endif
