//
// Created by Admin on 2026/9/30.
//

#include "Start.h"
#include "FreeRTOS.h"
#include "task.h"

void ALL_Start()
{
    Motor_Init();
    Encoder_Init();
    Speed_Init();
    MotorControl_Init();
    NRF24L01_SPI_Init();
    NRF24L01_RX_Mode();
    RemoteControl_Init();
}

void StartDefaultTask(void const *argument)
{
    (void)argument;
    for(;;)
    {
        osDelay(1);
    }
}

void StartMotorTast(void const *argument)
{
    (void)argument;
    const TickType_t Period=pdMS_TO_TICKS(SPEED_SAMPLE_PERIOD_MS);
    TickType_t LastWakeTime=xTaskGetTickCount();
    Speed_Reset();
    uint32_t LastSampleTick=HAL_GetTick();
    for(;;)
    {
        vTaskDelayUntil(&LastWakeTime, Period);
        TickType_t Now=xTaskGetTickCount();
        // Skip missed scheduling slots instead of running a burst of catch-up cycles.
        if((TickType_t)(Now-LastWakeTime)>=Period)LastWakeTime=Now;

        uint32_t SampleTick=HAL_GetTick();
        uint32_t ElapsedMs=SampleTick-LastSampleTick;
        if(ElapsedMs==0)continue;
        float SampleTime=(float)ElapsedMs/1000.0f;
        if(Speed_Update(SampleTime))
        {
            LastSampleTick=SampleTick;
            RemoteControl_Update(SampleTime);
        }
    }
}

void StartCommTask(void const *argument)
{
    uint8_t RxData[RX_PLOAD_WIDTH];

    (void)argument;
    for(;;)
    {
        if(NRF24L01_RxPacket(RxData)==0)
        {
            RemoteControl_Process(RxData, RX_PLOAD_WIDTH);
        }
        osDelay(5);
    }
}

void StartArmOledTask(void const *argument)
{
    (void)argument;
    for(;;)
    {
        osDelay(10);
    }
}
