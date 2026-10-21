#include "test.h"
#include "main.h"
#include "input/bsp_input.h"
#include "remote_config.h"
#include <stdio.h>
static uint16_t RawKeys,StableKeys;
static uint32_t KeyTick,TxFaults;
static uint8_t Validity=2;
static const char *KeyNames[KEY_COUNT]={
 "LEFT_UP PA11","LEFT_DOWN PA12","LEFT_LEFT PA15","LEFT_RIGHT PB3",
 "KEY_A PB6","KEY_A_PLUS PB7","KEY_B PB8","KEY_B_PLUS PB9",
 "LEFT_ROCKER PA8","RIGHT_ROCKER PC15","KEY_1 PC14","WAUP PA0"};
// Printing is diagnostic only: a stalled UART must never block the input bridge or stop the board.
// After TEST_TX_FAULT_LIMIT consecutive timeouts the printer gives up instead of retrying forever.
#define TEST_TX_FAULT_LIMIT 4U
static uint8_t Send(const char *Text)
{
 uint32_t Started=HAL_GetTick();
 if(TxFaults>=TEST_TX_FAULT_LIMIT)return 0;
 while(*Text)
 {
  while(!(USART1->SR&USART_SR_TXE))if((uint32_t)(HAL_GetTick()-Started)>=50U){TxFaults++;return 0;}
  USART1->DR=(uint8_t)*Text++;
 }
 while(!(USART1->SR&USART_SR_TC))if((uint32_t)(HAL_GetTick()-Started)>=50U){TxFaults++;return 0;}
 TxFaults=0;
 return 1;
}
void Test_Init(void)
{
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_USART1_CLK_ENABLE();
 GPIO_InitTypeDef GPIO={0};
 GPIO.Pin=GPIO_PIN_9;
 GPIO.Mode=GPIO_MODE_AF_PP;
 GPIO.Speed=GPIO_SPEED_FREQ_HIGH;
 HAL_GPIO_Init(GPIOA,&GPIO);
 GPIO.Pin=GPIO_PIN_10;
 GPIO.Mode=GPIO_MODE_INPUT;
 GPIO.Pull=GPIO_PULLUP;
 HAL_GPIO_Init(GPIOA,&GPIO);
 USART1->CR1=0;
 USART1->CR2=0;
 USART1->CR3=0;
 USART1->BRR=(HAL_RCC_GetPCLK2Freq()+57600U)/115200U;
 USART1->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
 BSP_InputInit();
 RawKeys=StableKeys=0;
 KeyTick=HAL_GetTick();
 Validity=2;
 (void)Send("\r\nCHIP1 INPUT BRIDGE | USART1 115200 8N1\r\nI2C MASTER PA5/PA7 ADDR 0x32 | OLED REMOVED | NO RADIO\r\n");
}
void Test_Step(void)
{
 uint32_t Now=HAL_GetTick();
 BSP_InputStep(Now);
 Remote_Input Input=BSP_InputRead(Now);
 char Text[180];
 if(Input.Keys!=RawKeys){RawKeys=Input.Keys;KeyTick=Now;}
 if((uint32_t)(Now-KeyTick)>=REMOTE_DEBOUNCE_MS && RawKeys!=StableKeys)
 {
  uint16_t Changed=RawKeys^StableKeys;
  StableKeys=RawKeys;
  for(uint8_t t=0;t<KEY_COUNT;t++)if(Changed&KEY_MASK(t))
  {
   (void)snprintf(Text,sizeof(Text),"KEY %s %s\r\n",KeyNames[t],StableKeys&KEY_MASK(t)?"DOWN":"UP");
   if(!Send(Text))return;
  }
 }
 if(Input.Valid!=Validity)
 {
  Validity=Input.Valid;
  (void)Send(Validity?"ADC VALID\r\n":"ADC INVALID\r\n");
 }
}
