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
    RemoteControl_Init();
    NRF24L01_SPI_Init();
    // Failure leaves the radio and motors disabled; the communication task retries.
    (void)NRF24L01_RX_Mode();
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
        NRF24L01_Result Result=NRF_EMPTY;
        // Hardware FIFO holds three packets. Bound each batch even under continuous RX.
        for(uint8_t i=0;i<NRF_RX_FIFO_DEPTH;i++)
        {
            Result=NRF24L01_RxPacket(RxData);
            if(Result!=NRF_OK)break;
            RemoteControl_Process(RxData, RX_PLOAD_WIDTH);
        }
        if(Result!=NRF_OK && Result!=NRF_EMPTY)
        {
            RemoteControl_Disconnect();
            osDelay(100);
            (void)NRF24L01_RX_Mode();
        }
        else osDelay(5);
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
