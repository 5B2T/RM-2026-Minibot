#include "test.h"
#include "main.h"
#include "input/bsp_input.h"
#include "remote_config.h"
#include <stdio.h>
static uint16_t RawKeys,StableKeys;
static uint32_t KeyTick;
static uint8_t Validity=2;
static char Tx[512];
static uint16_t TxRead,TxWrite;
// Names must follow Remote_Key order, not the old working labels: the right stick
// buttons are joint selectors (R_UP/R_DOWN/R_LEFT/R_RIGHT), PC14 is the gripper and
// PA0 is deploy. L_PRESS/R_PRESS are not wired on this board and never report.
static const char *KeyNames[KEY_COUNT]={
 "L_UP PA11","L_DOWN PA12","L_LEFT PA15","L_RIGHT PB3",
 "R_UP PB6","R_DOWN PB7","R_LEFT PB8","R_RIGHT PB9",
 "L_PRESS PA8","R_PRESS PC15","GRIPPER PC14","DEPLOY PA0"};
// Printing is diagnostic only: text is queued and flushed a few bytes per step, so a stalled
// UART can never block the input bridge. Full queue simply drops the line.
static void Send(const char *Text)
{
 while(*Text){uint16_t Next=(TxWrite+1U)%sizeof(Tx);if(Next==TxRead)return;Tx[TxWrite]=*Text++;TxWrite=Next;}
}
static void Flush(void)
{
 for(unsigned i=0;i<16 && TxRead!=TxWrite && (USART1->SR&USART_SR_TXE);i++)
 {USART1->DR=(uint8_t)Tx[TxRead];TxRead=(TxRead+1U)%sizeof(Tx);}
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
 TxRead=TxWrite=0;
 Send("\r\nCHIP1 INPUT BRIDGE | USART1 115200 8N1\r\nI2C MASTER PA5/PA7 ADDR 0x32 | OLED REMOVED | NO RADIO\r\n");
}
void Test_Step(void)
{
 Flush(); // Diagnostic output must never stall input sampling / STOP forwarding.
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
   Send(Text);
  }
 }
 if(Input.Valid!=Validity)
 {
  Validity=Input.Valid;
  Send(Validity?"ADC VALID\r\n":"ADC INVALID\r\n");
 }
}
