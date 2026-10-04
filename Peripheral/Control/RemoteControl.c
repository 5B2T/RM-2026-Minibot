#include "RemoteControl.h"
#include "MotorControl.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define REMOTE_CONTROL_HEADER 0xA5U
#define REMOTE_CONTROL_VERSION 0x01U

static float RemoteControl_Target[4];
static uint32_t RemoteControl_LastRxTick;
static uint8_t RemoteControl_Connected;

static uint16_t RemoteControl_CRC16(const uint8_t* Data, uint8_t Length)
{
    uint16_t Crc = 0xFFFFU;
    for(uint8_t i=0;i<Length;i++)
    {
        Crc ^= Data[i];
        for(uint8_t bit=0;bit<8;bit++)
        {
            if(Crc & 1U)Crc=(Crc>>1)^0xA001U;
            else Crc>>=1;
        }
    }
    return Crc;
}

void RemoteControl_Init(void)
{
    memset(RemoteControl_Target, 0, sizeof(RemoteControl_Target));
    RemoteControl_LastRxTick=HAL_GetTick();
    RemoteControl_Connected=0;
}

uint8_t RemoteControl_Process(const uint8_t* Data, uint8_t Length)
{
    RemoteControl_FrameTypeDef Frame;
    uint16_t Crc;

    if(Data==0 || Length!=REMOTE_CONTROL_FRAME_SIZE)return 0;
    memcpy(&Frame, Data, sizeof(Frame));
    if(Frame.Header!=REMOTE_CONTROL_HEADER || Frame.Version!=REMOTE_CONTROL_VERSION)return 0;
    Crc=RemoteControl_CRC16(Data, REMOTE_CONTROL_FRAME_SIZE-2U);
    if(Crc!=Frame.Frame_CRC)return 0;

    if(Frame.Flags & REMOTE_CONTROL_FLAG_STOP)
    {
        RemoteControl_Target[0]=0;
        RemoteControl_Target[1]=0;
        RemoteControl_Target[2]=0;
        RemoteControl_Target[3]=0;
    }
    else
    {
        RemoteControl_Target[0]=(float)Frame.M1/REMOTE_CONTROL_SPEED_SCALE;
        RemoteControl_Target[1]=(float)Frame.M2/REMOTE_CONTROL_SPEED_SCALE;
        RemoteControl_Target[2]=(float)Frame.M3/REMOTE_CONTROL_SPEED_SCALE;
        RemoteControl_Target[3]=(float)Frame.M4/REMOTE_CONTROL_SPEED_SCALE;
    }
    RemoteControl_LastRxTick=HAL_GetTick();
    RemoteControl_Connected=1;
    return 1;
}

void RemoteControl_Update(float SampleTime)
{
    if(!RemoteControl_Connected || HAL_GetTick()-RemoteControl_LastRxTick>REMOTE_CONTROL_TIMEOUT_MS)
    {
        RemoteControl_Connected=0;
        MotorControl_Stop();
        return;
    }

    MotorControl_SetTarget(RemoteControl_Target[0], RemoteControl_Target[1],
                           RemoteControl_Target[2], RemoteControl_Target[3]);
    MotorControl_Update(SampleTime);
}

uint8_t RemoteControl_IsConnected(void)
{
    return RemoteControl_Connected;
}

uint32_t RemoteControl_LastTick(void)
{
    return RemoteControl_LastRxTick;
}
