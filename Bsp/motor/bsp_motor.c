#include "motor/bsp_motor.h"
#include "board_config.h"
#include <stddef.h>

typedef struct
{
    GPIO_TypeDef *A_Port;
    uint16_t A_Pin;
    GPIO_TypeDef *B_Port;
    uint16_t B_Pin;
    TIM_HandleTypeDef *Timer;
    uint32_t Channel;
} MotorChannel;
static const MotorChannel Channels[BSP_MOTOR_COUNT]=BOARD_MOTOR_MAP;

void BSP_Motor_Disable(void)
{
    HAL_GPIO_WritePin(BOARD_STBY_1_PORT, BOARD_STBY_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_STBY_2_PORT, BOARD_STBY_2_PIN, GPIO_PIN_RESET);
    for(uint8_t i=0;i<BSP_MOTOR_COUNT;i++)
        __HAL_TIM_SET_COMPARE(Channels[i].Timer, Channels[i].Channel, 0);
}

uint8_t BSP_Motor_Init(void)
{
    BSP_Motor_Disable();
    for(uint8_t i=0;i<BSP_MOTOR_COUNT;i++)
    {
        const MotorChannel *Motor=&Channels[i];
        HAL_GPIO_WritePin(Motor->A_Port, Motor->A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor->B_Port, Motor->B_Pin, GPIO_PIN_RESET);
        if(HAL_TIM_PWM_Start(Motor->Timer, Motor->Channel)!=HAL_OK)
        {
            BSP_Motor_Disable();
            return 0;
        }
    }
    return 1;
}

void BSP_Motor_Enable(void)
{
    HAL_GPIO_WritePin(BOARD_STBY_1_PORT, BOARD_STBY_1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BOARD_STBY_2_PORT, BOARD_STBY_2_PIN, GPIO_PIN_SET);
}

void BSP_Motor_Load(const int PWM[BSP_MOTOR_COUNT])
{
    if(PWM==NULL)
    {
        BSP_Motor_Disable();
        return;
    }
    for(uint8_t i=0;i<BSP_MOTOR_COUNT;i++)
    {
        const MotorChannel *Motor=&Channels[i];
        // Clamp before negation, including INT_MIN.
        int Value=PWM[i];
        if(Value>BSP_MOTOR_PWM_MAX)Value=BSP_MOTOR_PWM_MAX;
        if(Value<-BSP_MOTOR_PWM_MAX)Value=-BSP_MOTOR_PWM_MAX;
        if(Value!=0)
        {
            HAL_GPIO_WritePin(Motor->A_Port, Motor->A_Pin,
                              Value>0?GPIO_PIN_SET:GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Motor->B_Port, Motor->B_Pin,
                              Value>0?GPIO_PIN_RESET:GPIO_PIN_SET);
        }
        __HAL_TIM_SET_COMPARE(Motor->Timer, Motor->Channel, Value<0?-Value:Value);
    }
}
