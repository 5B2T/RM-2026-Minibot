#include "time/bsp_time.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
uint32_t BSP_TimeMs(void)
{
    return HAL_GetTick();
}
void BSP_WaitMs(uint32_t Ms)
{
    // This project runs at 1 kHz. Extra tick prevents waiting less than requested.
    if(xTaskGetSchedulerState()==taskSCHEDULER_RUNNING)osDelay(Ms+1U);
    else HAL_Delay(Ms);
}
