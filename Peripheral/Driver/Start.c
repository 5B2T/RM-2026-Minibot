//
// Created by Admin on 2026/9/30.
//

#include "Start.h"

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
    for(;;)
    {
        RemoteControl_Update();
        osDelay(10);
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
