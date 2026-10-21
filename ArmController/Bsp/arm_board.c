#include "arm_board.h"
#include "spi.h"
#include "command/input_link.h"
#include <string.h>
SPI_HandleTypeDef hspi2;
static I2C_HandleTypeDef Link;
static ADC_HandleTypeDef ADC;
static uint8_t Rx[INPUT_LINK_SIZE],Mailbox[INPUT_LINK_SIZE];
static volatile uint8_t MailPending,Rearm;
static volatile uint32_t MailTick;
static volatile uint8_t Receiving;
static uint8_t HaveSequence,LastSequence;
static volatile uint32_t ReceiveTick;
static uint32_t RetryTick;
static Remote_Input Snapshot;
volatile uint32_t ArmBoard_RxCount,ArmBoard_Errors;
static uint32_t Lock(void)
{uint32_t Mask=__get_PRIMASK();__disable_irq();return Mask;}
static void Unlock(uint32_t Mask){__set_PRIMASK(Mask);}
void ArmBoard_Print(const char *Text)
{
 uint32_t Started=HAL_GetTick();
 while(*Text)
 {
  while(!(USART1->SR&USART_SR_TXE))if((uint32_t)(HAL_GetTick()-Started)>=30U)return;
  USART1->DR=(uint8_t)*Text++;
 }
}
static uint8_t Listen(void)
{
 return HAL_I2C_EnableListen_IT(&Link)==HAL_OK;
}
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *I2C,uint8_t Direction,uint16_t Address)
{
 (void)Address;
 if(I2C!=&Link)return;
 if(Direction!=I2C_DIRECTION_TRANSMIT)
 {CLEAR_BIT(I2C->Instance->CR1,I2C_CR1_ACK);Rearm=1;return;}
 Receiving=1;ReceiveTick=HAL_GetTick();
 if(HAL_I2C_Slave_Seq_Receive_IT(I2C,Rx,sizeof(Rx),I2C_FIRST_AND_LAST_FRAME)!=HAL_OK)Rearm=1;
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2C)
{
 if(I2C!=&Link)return;
 memcpy(Mailbox,Rx,sizeof(Rx));MailTick=HAL_GetTick();MailPending=1;
}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *I2C)
{if(I2C==&Link){Receiving=0;if(!Listen())Rearm=1;}}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2C)
{if(I2C==&Link){ArmBoard_Errors++;Receiving=0;Rearm=1;}}
void I2C1_EV_IRQHandler(void){HAL_I2C_EV_IRQHandler(&Link);}
void I2C1_ER_IRQHandler(void){HAL_I2C_ER_IRQHandler(&Link);}
static uint8_t ConfigureLink(void)
{
 Link.Instance=I2C1;
 Link.Init.ClockSpeed=100000;
 Link.Init.DutyCycle=I2C_DUTYCYCLE_2;
 Link.Init.OwnAddress1=INPUT_LINK_ADDRESS<<1;
 Link.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
 Link.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
 Link.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
 Link.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
 if(HAL_I2C_Init(&Link)!=HAL_OK)return 0;
 return Listen();
}
void ArmBoard_Init(void)
{
 __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
 __HAL_RCC_AFIO_CLK_ENABLE();__HAL_RCC_I2C1_CLK_ENABLE();
 __HAL_RCC_SPI2_CLK_ENABLE();__HAL_RCC_ADC1_CLK_ENABLE();__HAL_RCC_USART1_CLK_ENABLE();
 GPIO_InitTypeDef GPIO={0};
 GPIO.Pin=GPIO_PIN_6|GPIO_PIN_7;GPIO.Mode=GPIO_MODE_AF_OD;
 GPIO.Pull=GPIO_NOPULL;GPIO.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB,&GPIO);
 GPIO.Pin=GPIO_PIN_10|GPIO_PIN_12;GPIO.Mode=GPIO_MODE_OUTPUT_PP;HAL_GPIO_Init(GPIOB,&GPIO);
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
 GPIO.Pin=GPIO_PIN_13|GPIO_PIN_15;GPIO.Mode=GPIO_MODE_AF_PP;HAL_GPIO_Init(GPIOB,&GPIO);
 GPIO.Pin=GPIO_PIN_11|GPIO_PIN_14;GPIO.Mode=GPIO_MODE_INPUT;GPIO.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOB,&GPIO);
 GPIO.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
 GPIO.Mode=GPIO_MODE_ANALOG;GPIO.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOA,&GPIO);
 GPIO.Pin=GPIO_PIN_9;GPIO.Mode=GPIO_MODE_AF_PP;HAL_GPIO_Init(GPIOA,&GPIO);
 GPIO.Pin=GPIO_PIN_10;GPIO.Mode=GPIO_MODE_INPUT;GPIO.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOA,&GPIO);
 USART1->CR1=0;USART1->CR2=0;USART1->CR3=0;
 USART1->BRR=(HAL_RCC_GetPCLK2Freq()+57600U)/115200U;
 USART1->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
 hspi2.Instance=SPI2;hspi2.Init.Mode=SPI_MODE_MASTER;
 hspi2.Init.Direction=SPI_DIRECTION_2LINES;hspi2.Init.DataSize=SPI_DATASIZE_8BIT;
 hspi2.Init.CLKPolarity=SPI_POLARITY_LOW;hspi2.Init.CLKPhase=SPI_PHASE_1EDGE;
 hspi2.Init.NSS=SPI_NSS_SOFT;hspi2.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_8;
 hspi2.Init.FirstBit=SPI_FIRSTBIT_MSB;hspi2.Init.TIMode=SPI_TIMODE_DISABLE;
 hspi2.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;hspi2.Init.CRCPolynomial=7;
 if(HAL_SPI_Init(&hspi2)!=HAL_OK)ArmBoard_Errors++;
 __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);
 ADC.Instance=ADC1;ADC.Init.ScanConvMode=ADC_SCAN_DISABLE;
 ADC.Init.ContinuousConvMode=DISABLE;ADC.Init.DiscontinuousConvMode=DISABLE;
 ADC.Init.ExternalTrigConv=ADC_SOFTWARE_START;ADC.Init.DataAlign=ADC_DATAALIGN_RIGHT;ADC.Init.NbrOfConversion=1;
 if(HAL_ADC_Init(&ADC)!=HAL_OK || HAL_ADCEx_Calibration_Start(&ADC)!=HAL_OK)ArmBoard_Errors++;
 HAL_NVIC_SetPriority(I2C1_EV_IRQn,1,0);HAL_NVIC_SetPriority(I2C1_ER_IRQn,1,0);
 HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
 // A failed peripheral setup is retried by the service loop; stopping here would take the whole remote offline.
 if(!ConfigureLink()){ArmBoard_Errors++;Rearm=1;}
}
void ArmBoard_Service(void)
{
 uint32_t Now=HAL_GetTick();
 if(Receiving && (uint32_t)(Now-ReceiveTick)>=15U)Rearm=1;
 if(Rearm && (uint32_t)(Now-RetryTick)>=20U)
 {
  RetryTick=Now;HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  (void)HAL_I2C_DeInit(&Link);__HAL_RCC_I2C1_FORCE_RESET();__HAL_RCC_I2C1_RELEASE_RESET();
  __HAL_RCC_I2C1_CLK_ENABLE();Receiving=0;
  if(ConfigureLink())Rearm=0;else {ArmBoard_Errors++;Rearm=1;}
  HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
 }
 uint8_t Data[INPUT_LINK_SIZE];uint32_t Tick;
 uint32_t Mask=Lock();
 if(!MailPending){Unlock(Mask);return;}
 memcpy(Data,Mailbox,sizeof(Data));Tick=MailTick;MailPending=0;Unlock(Mask);
 if(Data[0]!=0xa6 || Data[1]!=1 || Data[3]>1 ||
    InputLink_CRC(Data,18)!=InputLink_Get(Data+18) || (InputLink_Get(Data+4)&0xf000U))
 {ArmBoard_Errors++;return;}
 uint8_t Difference=(uint8_t)(Data[2]-LastSequence);
 if(HaveSequence && (uint32_t)(Tick-Snapshot.TickMs)<100U && (Difference==0 || Difference>127))return;
 Remote_Input Next={.TickMs=Tick,.Keys=InputLink_Get(Data+4),.Valid=Data[3]};
 for(uint8_t t=0;t<5;t++)
 {Next.ADC[t]=InputLink_Get(Data+6+t*2);if(Next.ADC[t]>4095){ArmBoard_Errors++;return;}}
 Snapshot=Next;HaveSequence=1;LastSequence=Data[2];ArmBoard_RxCount++;
}
Remote_Input ArmBoard_Input(uint32_t Now)
{
 Remote_Input Input=Snapshot;
 Input.Valid=Input.Valid && HaveSequence && (uint32_t)(Now-Input.TickMs)<ARM_BOARD_INPUT_STALE_MS;
 if(!Input.Valid)Input.Keys=0;
 return Input;
}
uint8_t ArmBoard_Pots(uint16_t Values[5])
{
 uint16_t Next[5];
 for(uint8_t t=0;t<5;t++)
 {
  ADC_ChannelConfTypeDef Channel={0};
  Channel.Channel=(uint32_t)t;Channel.Rank=ADC_REGULAR_RANK_1;Channel.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;
  if(HAL_ADC_ConfigChannel(&ADC,&Channel)!=HAL_OK || HAL_ADC_Start(&ADC)!=HAL_OK)return 0;
  if(HAL_ADC_PollForConversion(&ADC,1)!=HAL_OK){(void)HAL_ADC_Stop(&ADC);return 0;}
  Next[t]=(uint16_t)HAL_ADC_GetValue(&ADC);
  if(HAL_ADC_Stop(&ADC)!=HAL_OK)return 0;
 }
 memcpy(Values,Next,sizeof(Next));return 1;
}
