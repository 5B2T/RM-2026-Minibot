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

// Shared OLED / future sensor bus: I2C1, PB6 SCL / PB7 SDA, 100 kHz.
#define BOARD_DEVICE_I2C hi2c1
#define BOARD_I2C_MAX_WRITE_SIZE 33U
#define BOARD_I2C_LOCK_TIMEOUT_MS 5U
#define BOARD_I2C_WRITE_TIMEOUT_MS 10U
// Core-board SW2 / SW4, pressed connects to GND; internal pull-ups enabled.
#define BOARD_BUTTON_NEXT_PORT GPIOA
#define BOARD_BUTTON_NEXT_PIN GPIO_PIN_15
#define BOARD_BUTTON_BACK_PORT GPIOE
#define BOARD_BUTTON_BACK_PIN GPIO_PIN_6

// J1 shoulder, J2 elbow, J3 wrist pitch, J4 wrist roll, J5 gripper.
#define BOARD_ARM_SERVO_MAP { \
    {&htim9,TIM_CHANNEL_1},{&htim10,TIM_CHANNEL_1}, \
    {&htim11,TIM_CHANNEL_1},{&htim12,TIM_CHANNEL_1},{&htim12,TIM_CHANNEL_2} }
#define BOARD_ARM_BASE_TIMER htim13
#define BOARD_ARM_BASE_CHANNEL TIM_CHANNEL_1
#define BOARD_ARM_ENCODER_TIMER htim4
#define BOARD_ARM_IN1_PORT GPIOD
#define BOARD_ARM_IN1_PIN GPIO_PIN_5
#define BOARD_ARM_IN2_PORT GPIOD
#define BOARD_ARM_IN2_PIN GPIO_PIN_6
#define BOARD_ARM_STBY_PORT GPIOD
#define BOARD_ARM_STBY_PIN GPIO_PIN_7

#endif
