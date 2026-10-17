#include "remote_app.h"
#include "input/bsp_input.h"
#include "i2c/bsp_soft_i2c.h"
#include "command/input_link.h"
#include "main.h"
#include "test.h"
static uint32_t SendTick;
static uint8_t Sequence;
void RemoteApp_Init(void)
{
 Test_Init();
 SendTick=HAL_GetTick()-20U;Sequence=0;
}
void RemoteApp_Step(void)
{
 Test_Step();
 uint32_t Now=HAL_GetTick();
 if((uint32_t)(Now-SendTick)<20U)return;
 SendTick=Now;
 Remote_Input Input=BSP_InputRead(Now);
 uint8_t Data[INPUT_LINK_SIZE]={0xa6,1,Sequence++,Input.Valid};
 InputLink_Put(Data+4,Input.Keys);
 for(uint8_t t=0;t<5;t++)InputLink_Put(Data+6+t*2,Input.ADC[t]);
 InputLink_Put(Data+18,InputLink_CRC(Data,18));
 uint8_t OK=BSP_SoftI2C_Write(INPUT_LINK_ADDRESS,Data,sizeof(Data));
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,OK?GPIO_PIN_RESET:GPIO_PIN_SET);
}
