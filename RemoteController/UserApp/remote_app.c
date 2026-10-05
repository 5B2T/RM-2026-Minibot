#include "remote_app.h"
#include "remote_config.h"
#include "input/bsp_input.h"
#include "control/remote_control.h"
#include "components/radio/remote_radio.h"
#include "components/display/remote_display.h"
#include "time/bsp_time.h"
#include "main.h"
static uint32_t DisplayTick;
void RemoteApp_Init(void)
{Controller_Init();BSP_InputInit();RemoteRadio_Init(BSP_TimeMs());DisplayTick=BSP_TimeMs();}
void RemoteApp_Step(void)
{
 BSP_InputStep(BSP_TimeMs());Remote_Input Input=BSP_InputRead(BSP_TimeMs());
 Controller_Update(&Input,BSP_TimeMs());RemoteRadio_Step(BSP_TimeMs());
 Remote_Status s=Controller_GetStatus();
 // Active-low LEDs show local TX authorization and radio ACK, never car execution.
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,s.LinkAck?GPIO_PIN_RESET:GPIO_PIN_SET);
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,s.Mode==REMOTE_ACTIVE?GPIO_PIN_RESET:GPIO_PIN_SET);
 if((uint32_t)(BSP_TimeMs()-DisplayTick)>=5){RemoteDisplay_Step(BSP_TimeMs());DisplayTick=BSP_TimeMs();}
}
