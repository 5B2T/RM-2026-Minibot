//
// Created by Admin on 2026/10/3.
//

#ifndef RM_2026_MINIBOT_TEST_H
#define RM_2026_MINIBOT_TEST_H

#include <stdint.h>

#define TEST_MOTOR 0x01U
#define TEST_ENCODER 0x02U
#define TEST_SPEED 0x04U
#define TEST_MOTOR_CONTROL 0x08U
#define TEST_NRF24L01 0x10U

uint8_t Test_Run(void);
uint8_t Test_NRF24L01(void);
uint8_t Test_Motor(void);
uint8_t Test_Encoder(void);
uint8_t Test_Speed(void);
uint8_t Test_MotorControl(void);

#endif //RM_2026_MINIBOT_TEST_H
