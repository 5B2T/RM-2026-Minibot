#include "i2c/bsp_soft_i2c.h"
#include "main.h"
#include <stddef.h>
#define SCL GPIO_PIN_5
#define SDA GPIO_PIN_7
static uint32_t Started;
static void Set(uint16_t Pin,uint8_t High)
{
 uint32_t Shift=Pin==SCL?20U:28U;
 if(High)GPIOA->BSRR=Pin;
 else GPIOA->BRR=Pin;
 MODIFY_REG(GPIOA->CRL,0xfU<<Shift,(High?0x8U:0x7U)<<Shift);
}
static uint8_t Read(uint16_t Pin){return HAL_GPIO_ReadPin(GPIOA,Pin)==GPIO_PIN_SET;}
static void Delay(void){uint32_t n=DWT->CYCCNT;while((uint32_t)(DWT->CYCCNT-n)<SystemCoreClock/40000U){} }
static uint8_t ClockHigh(void)
{
 Set(SCL,1);uint32_t n=DWT->CYCCNT;
 while(!Read(SCL))if((uint32_t)(DWT->CYCCNT-n)>=SystemCoreClock/1000U)return 0;
 Delay();return (uint32_t)(HAL_GetTick()-Started)<14;
}
static uint8_t Stop(void)
{
 Set(SCL,0);Set(SDA,0);Delay();uint8_t OK=ClockHigh();
 Set(SDA,1);Delay();Set(SCL,1);return OK&&Read(SCL)&&Read(SDA);
}
static uint8_t Byte(uint8_t Value)
{
 for(unsigned i=0;i<8;i++){Set(SCL,0);Set(SDA,(Value&0x80)!=0);Delay();if(!ClockHigh())return 0;Value<<=1;}
 Set(SCL,0);Set(SDA,1);Delay();if(!ClockHigh())return 0;
 uint8_t Ack=!Read(SDA);Set(SCL,0);return Ack;
}
uint8_t BSP_SoftI2C_Write(uint8_t Address,const uint8_t *Data,uint16_t Length)
{
 if(Address>0x7f||Data==NULL||Length==0||Length>33)return 0;
 CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
 Started=HAL_GetTick();Set(SDA,1);if(!ClockHigh())goto Failure;
 if(!Read(SDA)){ // Recover a device left in the middle of a byte after reset.
  for(unsigned i=0;i<9;i++){Set(SCL,0);Delay();if(!ClockHigh())goto Failure;}
  if(!Stop())goto Failure;
 }
 Set(SDA,0);Delay();Set(SCL,0);
 if(!Byte((uint8_t)(Address<<1)))goto Failure;
 for(uint16_t i=0;i<Length;i++)if(!Byte(Data[i]))goto Failure;
 return Stop();
Failure:(void)Stop();return 0;
}
