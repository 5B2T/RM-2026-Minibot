/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RADIO_CSN_Pin|LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_CE_GPIO_Port, RADIO_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_1_Pin KEY_RIGHTROCKER_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_RIGHTROCKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : WAUP_Pin */
  GPIO_InitStruct.Pin = WAUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(WAUP_GPIO_Port, &GPIO_InitStruct);

  /* Chip-to-chip software I2C, former OLED pins. */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5|GPIO_PIN_7,GPIO_PIN_SET);

  /*Configure GPIO pins : RADIO_CSN_Pin RADIO_CE_Pin LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = RADIO_CSN_Pin|RADIO_CE_Pin|LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_IRQ_Pin KEY_RIGHT_Pin KEY_A_Pin KEY_A_PLUS_Pin
                           KEY_B_Pin KEY_B_PLUS_Pin */
  GPIO_InitStruct.Pin = RADIO_IRQ_Pin|KEY_RIGHT_Pin|KEY_A_Pin|KEY_A_PLUS_Pin
                          |KEY_B_Pin|KEY_B_PLUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_LEFTROCKER_Pin KEY_DOWN_Pin KEY_UP_Pin KEY_LEFT_Pin */
  GPIO_InitStruct.Pin = KEY_LEFTROCKER_Pin|KEY_DOWN_Pin|KEY_UP_Pin|KEY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
