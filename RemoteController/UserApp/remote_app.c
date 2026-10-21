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
 SendTick=HAL_GetTick()-INPUT_LINK_PERIOD_MS;Sequence=0;
}
void RemoteApp_Step(void)
{
 Test_Step();
 uint32_t Now=HAL_GetTick();
 if((uint32_t)(Now-SendTick)<INPUT_LINK_PERIOD_MS)return;
 SendTick=Now;
 Remote_Input Input=BSP_InputRead(Now);
 InputLink_Frame Frame={.Sequence=Sequence++,.Keys=Input.Keys,.Valid=Input.Valid};
 memcpy(Frame.ADC,Input.ADC,sizeof(Frame.ADC));
 uint8_t Data[INPUT_LINK_SIZE];
 uint8_t OK=InputLink_Encode(&Frame,Data) && BSP_SoftI2C_Write(INPUT_LINK_ADDRESS,Data,sizeof(Data));
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,OK?GPIO_PIN_RESET:GPIO_PIN_SET);
}
