//
// Created by Admin on 2026/9/30.
//

#include "Motor.h"

#include "stm32f4xx_hal_gpio.h"
#include <stdlib.h>

Motor_HandleTypeDef Motor1;
Motor_HandleTypeDef Motor2;
Motor_HandleTypeDef Motor3;
Motor_HandleTypeDef Motor4;

static void Motor_InitOne(Motor_HandleTypeDef* Motorx)
{
    HAL_GPIO_WritePin(Motorx->Motor_A_GPIOx, Motorx->Motor_A_GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motorx->Motor_B_GPIOx, Motorx->Motor_B_GPIO_Pin, GPIO_PIN_RESET);
}

static void Motor_Forward(Motor_HandleTypeDef* Motorx)
{
    HAL_GPIO_WritePin(Motorx->Motor_A_GPIOx, Motorx->Motor_A_GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motorx->Motor_B_GPIOx, Motorx->Motor_B_GPIO_Pin, GPIO_PIN_RESET);
}

static void Motor_Reverse(Motor_HandleTypeDef* Motorx)
{
    HAL_GPIO_WritePin(Motorx->Motor_A_GPIOx, Motorx->Motor_A_GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motorx->Motor_B_GPIOx, Motorx->Motor_B_GPIO_Pin, GPIO_PIN_SET);
}

static int Motor_Limit(int PWM)
{
    if(PWM<0)PWM=-PWM;
    if(PWM>999)PWM=999;
    return PWM;
}

void Motor_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

    Motor1=(Motor_HandleTypeDef){
        GPIOB,
        GPIO_PIN_0,
        GPIOB,
        GPIO_PIN_1,
        &htim1,
        TIM_CHANNEL_1,
        0
    };
    Motor2=(Motor_HandleTypeDef){
        GPIOB,
        GPIO_PIN_2,
        GPIOB,
        GPIO_PIN_10,
        &htim1,
        TIM_CHANNEL_2,
        0
    };
    Motor3=(Motor_HandleTypeDef){
        GPIOD,
        GPIO_PIN_0,
        GPIOD,
        GPIO_PIN_1,
        &htim1,
        TIM_CHANNEL_3,
        0
    };
    Motor4=(Motor_HandleTypeDef){
        GPIOD,
        GPIO_PIN_2,
        GPIOD,
        GPIO_PIN_3,
        &htim1,
        TIM_CHANNEL_4,
        0
    };

    Motor_InitOne(&Motor1);
    Motor_InitOne(&Motor2);
    Motor_InitOne(&Motor3);
    Motor_InitOne(&Motor4);
}

void Motor_Load(int Motor1_PWM, int Motor2_PWM, int Motor3_PWM, int Motor4_PWM)
{
    Motor1.Motor_PWM=Motor_Limit(Motor1_PWM);
    Motor2.Motor_PWM=Motor_Limit(Motor2_PWM);
    Motor3.Motor_PWM=Motor_Limit(Motor3_PWM);
    Motor4.Motor_PWM=Motor_Limit(Motor4_PWM);

    if(Motor1_PWM>0)Motor_Forward(&Motor1);
    if(Motor1_PWM<0)Motor_Reverse(&Motor1);
    if(Motor2_PWM>0)Motor_Forward(&Motor2);
    if(Motor2_PWM<0)Motor_Reverse(&Motor2);
    if(Motor3_PWM>0)Motor_Forward(&Motor3);
    if(Motor3_PWM<0)Motor_Reverse(&Motor3);
    if(Motor4_PWM>0)Motor_Forward(&Motor4);
    if(Motor4_PWM<0)Motor_Reverse(&Motor4);

    __HAL_TIM_SET_COMPARE(Motor1.Motor_PWM_TIMx, Motor1.Motor_PWM_Channel, Motor1.Motor_PWM);
    __HAL_TIM_SET_COMPARE(Motor2.Motor_PWM_TIMx, Motor2.Motor_PWM_Channel, Motor2.Motor_PWM);
    __HAL_TIM_SET_COMPARE(Motor3.Motor_PWM_TIMx, Motor3.Motor_PWM_Channel, Motor3.Motor_PWM);
    __HAL_TIM_SET_COMPARE(Motor4.Motor_PWM_TIMx, Motor4.Motor_PWM_Channel, Motor4.Motor_PWM);
}

void Motor_Stop(Motor_HandleTypeDef* Motorx)
{
    if(Motorx==NULL)return;
    Motorx->Motor_PWM=0;
    __HAL_TIM_SET_COMPARE(Motorx->Motor_PWM_TIMx, Motorx->Motor_PWM_Channel, 0);
    HAL_GPIO_WritePin(Motorx->Motor_A_GPIOx, Motorx->Motor_A_GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motorx->Motor_B_GPIOx, Motorx->Motor_B_GPIO_Pin, GPIO_PIN_RESET);
}
