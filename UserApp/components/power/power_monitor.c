#include "components/power/power_monitor.h"
#include "robot/robot_config.h"
#include "power/ina219.h"
#include "i2c/bsp_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static Robot_PowerStatus Snapshot;
static uint8_t Initialized,Waiting;
static uint32_t DelayTick,DelayMs;
void PowerMonitor_Init(void)
{
    memset(&Snapshot,0,sizeof(Snapshot));
    Initialized=Waiting=0;
}
Robot_PowerStatus PowerMonitor_GetStatus(uint32_t NowMs)
{
    taskENTER_CRITICAL();
    Robot_PowerStatus Copy=Snapshot;
    taskEXIT_CRITICAL();
    if(Copy.Valid && (uint32_t)(NowMs-Copy.TickMs)>=ROBOT_POWER_STALE_MS)
    {Copy.Valid=0;Copy.State=ROBOT_POWER_STALE;}
    return Copy;
}
void PowerMonitor_Step(uint32_t NowMs)
{
    if(Waiting && (uint32_t)(NowMs-DelayTick)<DelayMs)return;
    Waiting=0;
    if(!Initialized)
    {
        Initialized=INA219_Init(BSP_I2C_Write,BSP_I2C_ReadRegister,ROBOT_INA219_ADDRESS,
            ROBOT_INA219_SHUNT_OHM,ROBOT_INA219_CURRENT_LSB_A,ROBOT_POWER_CONVERSION_TIMEOUT_MS);
        taskENTER_CRITICAL();
        Snapshot.Valid=0;
        Snapshot.State=Initialized?ROBOT_POWER_WAITING:ROBOT_POWER_CONFIG_ERROR;
        taskEXIT_CRITICAL();
        if(!Initialized)goto Retry;
    }
    INA219_Sample Sample;
    INA219_Result Result=INA219_Step(NowMs,&Sample);
    if(Result==INA219_PENDING)return;
    taskENTER_CRITICAL();
    if(Result==INA219_SAMPLE)
        Snapshot=(Robot_PowerStatus){.VoltageV=Sample.VoltageV,.CurrentA=Sample.CurrentA,
            .PowerW=Sample.PowerW,.TickMs=NowMs,.Valid=1,.State=ROBOT_POWER_VALID};
    else
    {
        Snapshot.Valid=0;
        Snapshot.State=Result==INA219_IO_ERROR?ROBOT_POWER_OFFLINE:
            Result==INA219_OVERFLOW?ROBOT_POWER_OVERFLOW:
            Result==INA219_TIMEOUT?ROBOT_POWER_TIMEOUT:
            Result==INA219_INVALID_SAMPLE?ROBOT_POWER_INVALID:ROBOT_POWER_CONFIG_ERROR;
    }
    taskEXIT_CRITICAL();
    DelayTick=NowMs;Waiting=1;
    if(Result==INA219_SAMPLE) {DelayMs=ROBOT_POWER_SAMPLE_MS;return;}
    Initialized=0;
Retry:
    DelayTick=NowMs;DelayMs=ROBOT_POWER_RETRY_MS;Waiting=1;
}
