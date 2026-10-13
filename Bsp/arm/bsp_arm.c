#include "arm/bsp_arm.h"
#include "board_config.h"
#include <stddef.h>
typedef struct {TIM_HandleTypeDef *Timer;uint32_t Channel;} ServoChannel;
static const ServoChannel Servos[BSP_ARM_SERVO_COUNT]=BOARD_ARM_SERVO_MAP;
static uint8_t Ready;
static void BaseStop(void)
{
    HAL_GPIO_WritePin(BOARD_ARM_STBY_PORT,BOARD_ARM_STBY_PIN,GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&BOARD_ARM_BASE_TIMER,BOARD_ARM_BASE_CHANNEL,0);
}
static uint8_t PulsesValid(const uint16_t *Pulse)
{
    if(Pulse==NULL)return 0;
    for(uint8_t i=0;i<BSP_ARM_SERVO_COUNT;i++)if(Pulse[i]<500 || Pulse[i]>2500)return 0;
    return 1;
}
void BSP_Arm_Disable(void)
{
    Ready=0;
    BaseStop();
    (void)HAL_TIM_PWM_Stop(&BOARD_ARM_BASE_TIMER,BOARD_ARM_BASE_CHANNEL);
    for(uint8_t i=0;i<BSP_ARM_SERVO_COUNT;i++)
    {
        (void)HAL_TIM_PWM_Stop(Servos[i].Timer,Servos[i].Channel);
        __HAL_TIM_SET_COMPARE(Servos[i].Timer,Servos[i].Channel,0);
    }
    (void)HAL_TIM_Encoder_Stop(&BOARD_ARM_ENCODER_TIMER,TIM_CHANNEL_ALL);
}
uint8_t BSP_Arm_Init(const uint16_t InitialPulseUs[BSP_ARM_SERVO_COUNT])
{
    Ready=0;BaseStop();
    if(!PulsesValid(InitialPulseUs)) {BSP_Arm_Disable();return 0;}
    if(HAL_TIM_Encoder_Start(&BOARD_ARM_ENCODER_TIMER,TIM_CHANNEL_ALL)!=HAL_OK ||
       HAL_TIM_PWM_Start(&BOARD_ARM_BASE_TIMER,BOARD_ARM_BASE_CHANNEL)!=HAL_OK)
    {BSP_Arm_Disable();return 0;}
    for(uint8_t i=0;i<BSP_ARM_SERVO_COUNT;i++)
    {
        __HAL_TIM_SET_COMPARE(Servos[i].Timer,Servos[i].Channel,InitialPulseUs[i]);
        if(HAL_TIM_PWM_Start(Servos[i].Timer,Servos[i].Channel)!=HAL_OK)
        {BSP_Arm_Disable();return 0;}
    }
    Ready=1;return 1;
}
uint8_t BSP_Arm_Apply(const uint16_t PulseUs[BSP_ARM_SERVO_COUNT], int BasePWM)
{
    if(!Ready || !PulsesValid(PulseUs) || BasePWM>BSP_ARM_BASE_PWM_MAX || BasePWM<-BSP_ARM_BASE_PWM_MAX)
    {BSP_Arm_Disable();return 0;}
    for(uint8_t i=0;i<BSP_ARM_SERVO_COUNT;i++)
        __HAL_TIM_SET_COMPARE(Servos[i].Timer,Servos[i].Channel,PulseUs[i]);
    if(BasePWM==0)BaseStop();
    else
    {
        HAL_GPIO_WritePin(BOARD_ARM_IN1_PORT,BOARD_ARM_IN1_PIN,BasePWM>0?GPIO_PIN_SET:GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BOARD_ARM_IN2_PORT,BOARD_ARM_IN2_PIN,BasePWM>0?GPIO_PIN_RESET:GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&BOARD_ARM_BASE_TIMER,BOARD_ARM_BASE_CHANNEL,BasePWM<0?-BasePWM:BasePWM);
        HAL_GPIO_WritePin(BOARD_ARM_STBY_PORT,BOARD_ARM_STBY_PIN,GPIO_PIN_SET);
    }
    return 1;
}
uint16_t BSP_Arm_EncoderCount(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&BOARD_ARM_ENCODER_TIMER);
}
