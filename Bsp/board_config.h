#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"
#include "tim.h"

// Physical channels M1..M4. Wheel position/polarity still needs vehicle verification.
#define BOARD_MOTOR_MAP { \
    {GPIOB,GPIO_PIN_0,GPIOB,GPIO_PIN_1,&htim1,TIM_CHANNEL_1}, \
    {GPIOB,GPIO_PIN_2,GPIOB,GPIO_PIN_10,&htim1,TIM_CHANNEL_2}, \
    {GPIOD,GPIO_PIN_0,GPIOD,GPIO_PIN_1,&htim1,TIM_CHANNEL_3}, \
    {GPIOD,GPIO_PIN_2,GPIOD,GPIO_PIN_3,&htim1,TIM_CHANNEL_4} }
// AB: PA0/1, PA6/7, PA5/PB3, PC6/7. All timer ARR values are 65535.
#define BOARD_ENCODER_MAP {&htim5,&htim3,&htim2,&htim8}
#define BOARD_STBY_1_PORT GPIOB
#define BOARD_STBY_1_PIN GPIO_PIN_11
#define BOARD_STBY_2_PORT GPIOD
#define BOARD_STBY_2_PIN GPIO_PIN_4
#define BOARD_RADIO_CE_PORT GPIOC
#define BOARD_RADIO_CE_PIN GPIO_PIN_5
#define BOARD_RADIO_CSN_PORT GPIOD
#define BOARD_RADIO_CSN_PIN GPIO_PIN_8
#define BOARD_RADIO_SPI hspi2
#define BOARD_RADIO_SPI_TIMEOUT_MS 2U

#endif
