#include "robot/robot.h"
#include "robot/robot_config.h"
#include "components/chassis/chassis.h"
#include "components/command/remote_control.h"
#include "motor/bsp_motor.h"
#include "encoder/bsp_encoder.h"
#include "radio/nrf24l01.h"
#include "time/bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
static Robot_Status StatusSnapshot;
static uint8_t RadioRecoveryPending;
static uint8_t Ready;
_Static_assert(RX_PLOAD_WIDTH==REMOTE_CONTROL_FRAME_SIZE, "Radio payload must match protocol");

uint8_t Robot_Init(void)
{
    Ready=0;
    memset(&StatusSnapshot, 0, sizeof(StatusSnapshot));
    RadioRecoveryPending=0;
    RemoteControl_Init();
    if(!BSP_Motor_Init() || !BSP_Encoder_Init() || !Chassis_Init())
    {
        BSP_Motor_Disable();
        return 0;
    }
    NRF24L01_SPI_Init();
    (void)NRF24L01_RX_Mode();
    Ready=1;
    return 1;
}
void Robot_ControlReset(void)
{
    if(Ready)Chassis_ResetSampling();
}
void Robot_ControlStep(float DtS)
{
    if(!Ready) {BSP_Motor_Disable();return;}
    Robot_WheelCommand Command;
    if(!Chassis_Measure(DtS))Chassis_Stop();
    else
    {
        RemoteControl_GetCommand(&Command);
        Chassis_Control(&Command, DtS);
    }
    Robot_Status Status={0};
    Chassis_GetFeedback(&Status);
    Status.TickMs=BSP_TimeMs();
    RemoteControl_LinkStatus Link=RemoteControl_GetLinkStatus();
    Status.Connected=Link.Connected;
    Status.Armed=Link.Armed;
    taskENTER_CRITICAL();
    StatusSnapshot=Status;
    taskEXIT_CRITICAL();
}
uint32_t Robot_CommunicationStep(void)
{
    if(!Ready)return ROBOT_RADIO_RETRY_MS;
    if(RadioRecoveryPending)
    {
        (void)NRF24L01_RX_Mode();
        RadioRecoveryPending=0;
    }
    uint8_t Data[RX_PLOAD_WIDTH];
    NRF24L01_Result Result=NRF_EMPTY;
    for(uint8_t i=0;i<NRF_RX_FIFO_DEPTH;i++)
    {
        Result=NRF24L01_RxPacket(Data);
        if(Result!=NRF_OK)break;
        RemoteControl_Process(Data, RX_PLOAD_WIDTH);
    }
    if(Result!=NRF_OK && Result!=NRF_EMPTY)
    {
        RemoteControl_Disconnect();
        RadioRecoveryPending=1;
        return ROBOT_RADIO_RETRY_MS;
    }
    return ROBOT_COMM_POLL_MS;
}
void Robot_GetStatus(Robot_Status *Status)
{
    if(Status==NULL)return;
    taskENTER_CRITICAL();
    *Status=StatusSnapshot;
    taskEXIT_CRITICAL();
}
