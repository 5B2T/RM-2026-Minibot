//
// Created by Admin on 2026/9/30.
//

#include "Motor.h"

#include <stdlib.h>

Motor_HandleTypeDef Motor[4];

static void Motor_InitOne(Motor_HandleTypeDef* Motorx)
{
    HAL_GPIO_WritePin(Motorx->Motor_A_GPIOx, Motorx->Motor_A_GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motorx->Motor_B_GPIOx, Motorx->Motor_B_GPIO_Pin, GPIO_PIN_RESET);
}

static int Motor_Limit(int PWM)
{
    if(PWM<0)PWM=-PWM;
    if(PWM>999)PWM=999;
    return PWM;
}

void Motor_Init(void)
{
    // PWM start accepts one channel per call; TIM_CHANNEL_ALL is encoder-only here.
    const uint32_t Channels[4]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};
    for(uint8_t t=0;t<4;t++)
    {
        if(HAL_TIM_PWM_Start(&htim1, Channels[t])!=HAL_OK)
        {
            Error_Handler();
            return;
        }
    }

    Motor[0]=(Motor_HandleTypeDef){
        GPIOB,
        GPIO_PIN_0,
        GPIOB,
        GPIO_PIN_1,
        &htim1,
        TIM_CHANNEL_1,
        0};
    Motor[1]=(Motor_HandleTypeDef){
        GPIOB,
        GPIO_PIN_2,
        GPIOB,
        GPIO_PIN_10,
        &htim1,
        TIM_CHANNEL_2,
        0};
    Motor[2]=(Motor_HandleTypeDef){
        GPIOD,
        GPIO_PIN_0,
        GPIOD,
        GPIO_PIN_1,
        &htim1,
        TIM_CHANNEL_3,
        0};
    Motor[3]=(Motor_HandleTypeDef){
        GPIOD,
        GPIO_PIN_2,
        GPIOD,
        GPIO_PIN_3,
        &htim1,
        TIM_CHANNEL_4,
        0};

    for(uint8_t t=0;t<4;t++)
    {
        Motor_InitOne(&Motor[t]);
    }
    Motor_Disable();
}

void Motor_Enable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    // STBY low puts both bridges into high impedance; this is not active braking.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    for(uint8_t t=0;t<4;t++)
    {
        Motor[t].Motor_PWM=0;
        __HAL_TIM_SET_COMPARE(Motor[t].Motor_PWM_TIMx, Motor[t].Motor_PWM_Channel, 0);
    }
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

void Motor_Load(int* Motor_PWM)
{

    for(uint8_t t=0;t<4;t++)
    {
        Motor[t].Motor_PWM=Motor_Limit(Motor_PWM[t]);

        if(Motor_PWM[t]>0)Motor_Forward(&Motor[t]);
        if(Motor_PWM[t]<0)Motor_Reverse(&Motor[t]);

        __HAL_TIM_SET_COMPARE(Motor[t].Motor_PWM_TIMx, Motor[t].Motor_PWM_Channel, Motor[t].Motor_PWM);
    }
}

void Motor_Stop(Motor_HandleTypeDef* Motorx)
{
    if(Motorx==NULL)return;
    Motorx->Motor_PWM=0;
    __HAL_TIM_SET_COMPARE(Motorx->Motor_PWM_TIMx, Motorx->Motor_PWM_Channel, 0);
    HAL_GPIO_WritePin(Motorx->Motor_A_GPIOx, Motorx->Motor_A_GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motorx->Motor_B_GPIOx, Motorx->Motor_B_GPIO_Pin, GPIO_PIN_RESET);
}
