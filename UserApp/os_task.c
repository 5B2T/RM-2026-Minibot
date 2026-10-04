#include "os_task.h"
#include "robot/robot.h"
#include "robot/robot_config.h"
#include "time/bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

void StartMotorTast(void const *argument)
{
    (void)argument;
    const TickType_t Period=pdMS_TO_TICKS(ROBOT_CONTROL_PERIOD_MS);
    TickType_t LastWakeTime=xTaskGetTickCount();
    Robot_ControlReset();
    uint32_t LastSampleTick=BSP_TimeMs();
    for(;;)
    {
        vTaskDelayUntil(&LastWakeTime, Period);
        TickType_t Now=xTaskGetTickCount();
        // Skip missed scheduling slots instead of running a burst of catch-up cycles.
        if((TickType_t)(Now-LastWakeTime)>=Period)LastWakeTime=Now;

        uint32_t SampleTick=BSP_TimeMs();
        uint32_t ElapsedMs=SampleTick-LastSampleTick;
        if(ElapsedMs==0)continue;
        float SampleTime=(float)ElapsedMs/1000.0f;
        LastSampleTick=SampleTick;
        Robot_ControlStep(SampleTime);
    }
}

void StartCommTask(void const *Argument)
{
    (void)Argument;
    for(;;)osDelay(Robot_CommunicationStep());
}

void StartArmOledTask(void const *Argument)
{
    (void)Argument;
    for(;;)osDelay(Robot_DisplayStep());
}
