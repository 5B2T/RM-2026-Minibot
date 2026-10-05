#include "time/bsp_time.h"
#include "main.h"
uint32_t BSP_TimeMs(void){return HAL_GetTick();}
void BSP_WaitMs(uint32_t Ms){HAL_Delay(Ms);}
