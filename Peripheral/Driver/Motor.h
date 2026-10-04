//
// Created by Admin on 2026/9/30.
//

#ifndef RM_2026_MINIBOT_MOTOR_H
#define RM_2026_MINIBOT_MOTOR_H
#include <stdint.h>
#include "tim.h"

//电机PWM M1-4分别为PE9 11 13 14
//AB相：M1为PB0/1 M2为PB2/10 M3为PD0/1 M4为PD2/3
//stby：PB11 PD4
//编码器AB相：PA0/1 PA6/7 PA5/PB3 PC6/7

typedef struct
{
    GPIO_TypeDef* Motor_A_GPIOx;
    uint16_t Motor_A_GPIO_Pin;
    GPIO_TypeDef* Motor_B_GPIOx;
    uint16_t Motor_B_GPIO_Pin;

    TIM_HandleTypeDef* Motor_PWM_TIMx;
    uint32_t Motor_PWM_Channel;

    int Motor_PWM;
}Motor_HandleTypeDef;

extern Motor_HandleTypeDef Motor[4];

void Motor_Init(void);
void Motor_Enable(void);
void Motor_Disable(void);
void Motor_Load(int* Motor_PWM);
void Motor_Stop(Motor_HandleTypeDef* Motorx);

#endif //RM_2026_MINIBOT_MOTOR_H
