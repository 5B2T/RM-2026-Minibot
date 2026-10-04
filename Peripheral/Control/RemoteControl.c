#include "RemoteControl.h"
#include "MotorControl.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define REMOTE_CONTROL_HEADER 0xA5U
#define REMOTE_CONTROL_VERSION 0x01U

typedef struct
{
    float Target[4];
    uint32_t LastRxTick;
    uint8_t HasFrame;
    uint8_t Armed;
    uint8_t MotionValid;
    uint8_t StopPending;
    uint8_t ResumePending;
    uint8_t SequenceKnown;
    uint8_t Sequence;
}RemoteControl_StateTypeDef;

static RemoteControl_StateTypeDef RemoteControl_State;

static uint16_t RemoteControl_CRC16(const uint8_t* Data, uint8_t Length)
{
    uint16_t Crc=0xFFFFU;
    for(uint8_t i=0;i<Length;i++)
    {
        Crc^=Data[i];
        for(uint8_t bit=0;bit<8;bit++)
        {
            if(Crc & 1U)Crc=(Crc>>1)^0xA001U;
            else Crc>>=1;
        }
    }
    return Crc;
}

// Caller holds the task critical section. Expiry is checked before accepting a frame.
static void RemoteControl_Expire(uint32_t Now)
{
    if(RemoteControl_State.HasFrame &&
       Now-RemoteControl_State.LastRxTick>=REMOTE_CONTROL_TIMEOUT_MS)
    {
        RemoteControl_State.HasFrame=0;
        RemoteControl_State.Armed=0;
        RemoteControl_State.MotionValid=0;
        RemoteControl_State.StopPending=1;
        RemoteControl_State.ResumePending=0;
        RemoteControl_State.SequenceKnown=0;
        memset(RemoteControl_State.Target, 0, sizeof(RemoteControl_State.Target));
    }
}

void RemoteControl_Init(void)
{
    // Called once before the scheduler starts; no task can access the state yet.
    memset(&RemoteControl_State, 0, sizeof(RemoteControl_State));
    RemoteControl_State.LastRxTick=HAL_GetTick();
    RemoteControl_State.StopPending=1;
}

uint8_t RemoteControl_Process(const uint8_t* Data, uint8_t Length)
{
    RemoteControl_FrameTypeDef Frame;
    if(Data==0 || Length!=REMOTE_CONTROL_FRAME_SIZE)return 0;
    memcpy(&Frame, Data, sizeof(Frame));
    if(Frame.Header!=REMOTE_CONTROL_HEADER || Frame.Version!=REMOTE_CONTROL_VERSION)return 0;
    if(Frame.Flags & ~(REMOTE_CONTROL_FLAG_STOP | REMOTE_CONTROL_FLAG_RESUME))return 0;
    if(RemoteControl_CRC16(Data, REMOTE_CONTROL_FRAME_SIZE-2U)!=Frame.Frame_CRC)return 0;

    uint8_t Stop=(Frame.Flags & REMOTE_CONTROL_FLAG_STOP)!=0;
    uint8_t Resume=(Frame.Flags & REMOTE_CONTROL_FLAG_RESUME)!=0;
    if(!Stop && Resume && (Frame.M1 || Frame.M2 || Frame.M3 || Frame.M4))return 0;
    float Target[4]={(float)Frame.M1/REMOTE_CONTROL_SPEED_SCALE,
                     (float)Frame.M2/REMOTE_CONTROL_SPEED_SCALE,
                     (float)Frame.M3/REMOTE_CONTROL_SPEED_SCALE,
                     (float)Frame.M4/REMOTE_CONTROL_SPEED_SCALE};

    taskENTER_CRITICAL();
    uint32_t Now=HAL_GetTick();
    RemoteControl_Expire(Now);
    uint8_t Delta=(uint8_t)(Frame.Sequence-RemoteControl_State.Sequence);
    uint8_t Fresh=!RemoteControl_State.SequenceKnown || (Delta>0 && Delta<128U);
    // A valid STOP is always honored, even if its sequence is stale.
    if(!Stop && !Fresh)
    {
        taskEXIT_CRITICAL();
        return 0;
    }
    if(Fresh)
    {
        RemoteControl_State.Sequence=Frame.Sequence;
        RemoteControl_State.SequenceKnown=1;
    }
    RemoteControl_State.LastRxTick=Now;
    RemoteControl_State.HasFrame=1;

    if(Stop)
    {
        RemoteControl_State.Armed=0;
        RemoteControl_State.MotionValid=0;
        RemoteControl_State.StopPending=1;
        RemoteControl_State.ResumePending=0;
        memset(RemoteControl_State.Target, 0, sizeof(RemoteControl_State.Target));
    }
    else if(Resume)
    {
        RemoteControl_State.MotionValid=0;
        memset(RemoteControl_State.Target, 0, sizeof(RemoteControl_State.Target));
        if(RemoteControl_State.StopPending)RemoteControl_State.ResumePending=1;
        else RemoteControl_State.Armed=1;
    }
    else if(RemoteControl_State.Armed)
    {
        memcpy(RemoteControl_State.Target, Target, sizeof(Target));
        RemoteControl_State.MotionValid=1;
    }
    taskEXIT_CRITICAL();
    return 1;
}

void RemoteControl_Update(float SampleTime)
{
    float Target[4];
    uint8_t Run;
    taskENTER_CRITICAL();
    RemoteControl_Expire(HAL_GetTick());
    if(RemoteControl_State.StopPending)
    {
        // Consume STOP before allowing queued RESUME. No old target survives.
        RemoteControl_State.StopPending=0;
        RemoteControl_State.Armed=RemoteControl_State.ResumePending;
        RemoteControl_State.ResumePending=0;
        RemoteControl_State.MotionValid=0;
        memset(RemoteControl_State.Target, 0, sizeof(RemoteControl_State.Target));
        Run=0;
    }
    else
    {
        Run=RemoteControl_State.HasFrame && RemoteControl_State.Armed &&
            RemoteControl_State.MotionValid;
    }
    memcpy(Target, RemoteControl_State.Target, sizeof(Target));
    taskEXIT_CRITICAL();

    // Only the control task touches PI state and motor outputs.
    if(!Run)
    {
        MotorControl_Stop();
        return;
    }
    MotorControl_SetTarget(Target[0], Target[1], Target[2], Target[3]);
    MotorControl_Update(SampleTime);
}

uint8_t RemoteControl_IsConnected(void)
{
    taskENTER_CRITICAL();
    uint8_t Connected=RemoteControl_State.HasFrame &&
        HAL_GetTick()-RemoteControl_State.LastRxTick<REMOTE_CONTROL_TIMEOUT_MS;
    taskEXIT_CRITICAL();
    return Connected;
}

uint8_t RemoteControl_IsArmed(void)
{
    taskENTER_CRITICAL();
    uint8_t Armed=RemoteControl_State.HasFrame && RemoteControl_State.Armed &&
        !RemoteControl_State.StopPending &&
        HAL_GetTick()-RemoteControl_State.LastRxTick<REMOTE_CONTROL_TIMEOUT_MS;
    taskEXIT_CRITICAL();
    return Armed;
}

uint32_t RemoteControl_LastTick(void)
{
    taskENTER_CRITICAL();
    uint32_t Tick=RemoteControl_State.LastRxTick;
    taskEXIT_CRITICAL();
    return Tick;
}
