#ifndef ARM_BSP_BOARD_STUB_H
#define ARM_BSP_BOARD_STUB_H
#include <stdint.h>
typedef struct {uint32_t Channel[4],Counter;uint8_t Running[4],Encoder;} TIM_HandleTypeDef;
typedef enum {HAL_OK=0,HAL_ERROR} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET=0,GPIO_PIN_SET} GPIO_PinState;
extern TIM_HandleTypeDef FakeTimers[6];
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_ALL 255U
#define BOARD_ARM_SERVO_MAP {{&FakeTimers[0],0},{&FakeTimers[1],0},{&FakeTimers[2],0},{&FakeTimers[3],0},{&FakeTimers[0],1}}
#define BOARD_ARM_BASE_TIMER FakeTimers[4]
#define BOARD_ARM_BASE_CHANNEL 0U
#define BOARD_ARM_ENCODER_TIMER FakeTimers[5]
#define BOARD_ARM_IN1_PORT 0U
#define BOARD_ARM_IN1_PIN 0U
#define BOARD_ARM_IN2_PORT 0U
#define BOARD_ARM_IN2_PIN 1U
#define BOARD_ARM_STBY_PORT 0U
#define BOARD_ARM_STBY_PIN 2U
#define BOARD_ARM_PWM_PORT 0U
#define BOARD_ARM_PWM_PIN 3U
#define __HAL_TIM_SET_COMPARE(T,C,V) ((T)->Channel[(C)]=(V))
#define __HAL_TIM_GET_COMPARE(T,C) ((T)->Channel[(C)])
#define __HAL_TIM_GET_COUNTER(T) ((T)->Counter)
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *T,uint32_t C);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *T,uint32_t C);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *T,uint32_t C);
GPIO_PinState HAL_GPIO_ReadPin(unsigned P,unsigned N);
void HAL_GPIO_WritePin(unsigned P,unsigned N,GPIO_PinState S);
#endif
