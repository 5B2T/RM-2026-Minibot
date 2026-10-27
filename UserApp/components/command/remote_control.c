#include "components/command/remote_control.h"
#include "robot/robot_config.h"
#include "time/bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef struct
{
    float Target[4];
    Robot_CommandMode Mode;
    uint32_t LastRxTick;
    uint8_t HasFrame;
    uint8_t Armed;
    uint8_t MotionValid;
    uint8_t StopPending;
    uint8_t ResumePending;
    uint8_t SequenceKnown;
    uint8_t Sequence;
    uint32_t StopGeneration;
    RemoteControl_ArmInput Arm;
    RemoteControl_ArmTargets Targets;
    uint8_t DeployCounter,GripperCounter;
}RemoteControl_StateTypeDef;

static RemoteControl_StateTypeDef RemoteControl_State;
static RemoteControl_TelemetryStatus RemoteControl_Telemetry;

// Caller holds the task critical section; motor outputs remain owned by control task.
static void RemoteControl_ClearConnection(void)
{
    RemoteControl_State.HasFrame=0;
    RemoteControl_State.Armed=0;
    RemoteControl_State.MotionValid=0;
    RemoteControl_State.StopPending=1;
    RemoteControl_State.ResumePending=0;
    RemoteControl_State.SequenceKnown=0;
    RemoteControl_State.Mode=ROBOT_COMMAND_WHEELS;
    memset(RemoteControl_State.Target, 0, sizeof(RemoteControl_State.Target));
    memset(&RemoteControl_State.Arm,0,sizeof(RemoteControl_State.Arm));
    memset(&RemoteControl_State.Targets,0,sizeof(RemoteControl_State.Targets));
}

void RemoteControl_Disconnect(void)
{
    taskENTER_CRITICAL();
    RemoteControl_ClearConnection();
    taskEXIT_CRITICAL();
}

// Caller holds the task critical section. Expiry is checked before accepting a frame.
static void RemoteControl_Expire(uint32_t Now)
{
    if(RemoteControl_State.HasFrame &&
       Now-RemoteControl_State.LastRxTick>=ROBOT_REMOTE_TIMEOUT_MS)
    {
        RemoteControl_ClearConnection();
    }
}

void RemoteControl_Init(void)
{
    // Called once before the scheduler starts; no task can access the state yet.
    memset(&RemoteControl_State, 0, sizeof(RemoteControl_State));
    memset(&RemoteControl_Telemetry, 0, sizeof(RemoteControl_Telemetry));
    RemoteControl_State.LastRxTick=BSP_TimeMs();
    RemoteControl_State.StopPending=1;
}

uint8_t RemoteControl_Process(const uint8_t* Data, uint8_t Length)
{
    RemoteControl_FrameTypeDef Frame;
    RemoteTelemetry_FrameTypeDef Telemetry;
    // Telemetry is decoded first: same header, size and CRC, different version byte.
    if(RemoteTelemetry_Decode(Data, Length, &Telemetry))
    {
        taskENTER_CRITICAL();
        memcpy(RemoteControl_Telemetry.Raw,Telemetry.Raw,sizeof(RemoteControl_Telemetry.Raw));
        memcpy(RemoteControl_Telemetry.AngleCdeg,Telemetry.AngleCdeg,
               sizeof(RemoteControl_Telemetry.AngleCdeg));
        RemoteControl_Telemetry.Valid=(Telemetry.Flags&REMOTE_TELEMETRY_FLAG_POTS)!=0;
        RemoteControl_Telemetry.Sequence=Telemetry.Sequence;
        RemoteControl_Telemetry.TickMs=BSP_TimeMs();
        RemoteControl_Telemetry.HasFrame=1;
        taskEXIT_CRITICAL();
        return 1; // Diagnostic only: the command lease above is left untouched.
    }
    if(!CommandProtocol_Decode(Data, Length, &Frame))return 0;

    uint8_t Stop=(Frame.Flags & REMOTE_CONTROL_FLAG_STOP)!=0;
    uint8_t Resume=(Frame.Flags & REMOTE_CONTROL_FLAG_RESUME)!=0;
    float Target[4]={(float)Frame.M1/REMOTE_CONTROL_SPEED_SCALE,
                     (float)Frame.M2/REMOTE_CONTROL_SPEED_SCALE,
                     (float)Frame.M3/REMOTE_CONTROL_SPEED_SCALE,
                     (float)Frame.M4/REMOTE_CONTROL_SPEED_SCALE};

    if(Frame.Version!=REMOTE_CONTROL_VERSION_WHEELS)Target[2]=(float)Frame.M3/REMOTE_CONTROL_ANGULAR_SCALE;

    taskENTER_CRITICAL();
    uint32_t Now=BSP_TimeMs();
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
        RemoteControl_State.Targets.Valid=0;
        RemoteControl_State.StopGeneration++;
        RemoteControl_State.Armed=0;
        RemoteControl_State.MotionValid=0;
        RemoteControl_State.StopPending=1;
        RemoteControl_State.ResumePending=0;
        memset(RemoteControl_State.Target, 0, sizeof(RemoteControl_State.Target));
        memset(&RemoteControl_State.Arm,0,sizeof(RemoteControl_State.Arm));
    }
    else if(Resume)
    {
        RemoteControl_State.Targets.Valid=0;
        memset(&RemoteControl_State.Arm,0,sizeof(RemoteControl_State.Arm));
        RemoteControl_State.DeployCounter=Frame.Reserved[3];
        RemoteControl_State.GripperCounter=Frame.Reserved[4];
        RemoteControl_State.MotionValid=0;
        memset(RemoteControl_State.Target, 0, sizeof(RemoteControl_State.Target));
        if(RemoteControl_State.StopPending)RemoteControl_State.ResumePending=1;
        else RemoteControl_State.Armed=1;
    }
    else if(RemoteControl_State.Armed)
    {
        RemoteControl_State.Targets.Valid=0;
        RemoteControl_State.Mode=Frame.Version!=REMOTE_CONTROL_VERSION_WHEELS?
                                  ROBOT_COMMAND_VELOCITY:ROBOT_COMMAND_WHEELS;
        memcpy(RemoteControl_State.Target, Target, sizeof(Target));
        RemoteControl_State.MotionValid=1;
        if(Frame.Version==REMOTE_CONTROL_VERSION_ROBOT)
        {
            if(Frame.Flags&REMOTE_CONTROL_FLAG_ARM_TARGETS)
            {
                RemoteControl_ArmTargets *Targets=&RemoteControl_State.Targets;
                for(unsigned i=0;i<5;i++)
                {
                    int32_t Value=(uint16_t)Frame.Reserved[5+2*i]|
                                  ((uint16_t)Frame.Reserved[6+2*i]<<8);
                    if(Value>=32768)Value-=65536;
                    Targets->AngleDeg[i]=(float)Value/10.0f;
                }
                Targets->TickMs=Now;Targets->Valid=1;
            }
            RemoteControl_ArmInput *Arm=&RemoteControl_State.Arm;
            Arm->Enabled=1;Arm->Joint=Frame.Reserved[0];Arm->Base=Target[3];
            int32_t Rate=(uint16_t)Frame.Reserved[1]|((uint16_t)Frame.Reserved[2]<<8);
            if(Rate>=32768)Rate-=65536;
            Arm->Rate=(float)Rate/1000.0f;
            if(Frame.Reserved[3]!=RemoteControl_State.DeployCounter)Arm->Initialize=1;
            if(Frame.Reserved[4]!=RemoteControl_State.GripperCounter)Arm->Gripper=1;
            RemoteControl_State.DeployCounter=Frame.Reserved[3];
            RemoteControl_State.GripperCounter=Frame.Reserved[4];
        }
        else memset(&RemoteControl_State.Arm,0,sizeof(RemoteControl_State.Arm));
    }
    taskEXIT_CRITICAL();
    return 1;
}

void RemoteControl_GetCommand(Robot_ChassisCommand *Command)
{
    if(Command==NULL)return;
    uint8_t Run;
    taskENTER_CRITICAL();
    RemoteControl_Expire(BSP_TimeMs());
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
    Command->Mode=RemoteControl_State.Mode;
    if(Command->Mode==ROBOT_COMMAND_VELOCITY)
        Command->Target.Velocity=(Robot_ChassisVelocity){RemoteControl_State.Target[0],
                                                        RemoteControl_State.Target[1],
                                                        RemoteControl_State.Target[2]};
    else memcpy(Command->Target.Wheels, RemoteControl_State.Target, sizeof(Command->Target.Wheels));
    Command->Enabled=Run;
    taskEXIT_CRITICAL();
}

RemoteControl_LinkStatus RemoteControl_GetLinkStatus(void)
{
    taskENTER_CRITICAL();
    uint8_t Connected=RemoteControl_State.HasFrame &&
        BSP_TimeMs()-RemoteControl_State.LastRxTick<ROBOT_REMOTE_TIMEOUT_MS;
    RemoteControl_LinkStatus Status={
        .LastRxTick=RemoteControl_State.LastRxTick,
        .StopGeneration=RemoteControl_State.StopGeneration,
        .Connected=Connected,
        .Armed=Connected && RemoteControl_State.Armed && !RemoteControl_State.StopPending};
    taskEXIT_CRITICAL();
    return Status;
}
RemoteControl_ArmInput RemoteControl_TakeArmInput(void)
{
    return RemoteControl_TakeArmSnapshot().Input;
}
RemoteControl_ArmSnapshot RemoteControl_TakeArmSnapshot(void)
{
    taskENTER_CRITICAL();
    uint32_t Now=BSP_TimeMs();RemoteControl_Expire(Now);
    RemoteControl_ArmSnapshot Snapshot={0};
    Snapshot.Input=RemoteControl_State.Arm;
    Snapshot.Input.Enabled=Snapshot.Input.Enabled && RemoteControl_State.HasFrame && RemoteControl_State.Armed &&
                  RemoteControl_State.MotionValid && !RemoteControl_State.StopPending;
    Snapshot.Targets=RemoteControl_State.Targets;
    Snapshot.Targets.Valid=Snapshot.Targets.Valid && Snapshot.Input.Enabled;
    Snapshot.Link.LastRxTick=RemoteControl_State.LastRxTick;
    Snapshot.Link.StopGeneration=RemoteControl_State.StopGeneration;
    Snapshot.Link.Connected=RemoteControl_State.HasFrame &&
                           (uint32_t)(Now-RemoteControl_State.LastRxTick)<ROBOT_REMOTE_TIMEOUT_MS;
    Snapshot.Link.Armed=Snapshot.Link.Connected && RemoteControl_State.Armed && !RemoteControl_State.StopPending;
    RemoteControl_State.Arm.Initialize=RemoteControl_State.Arm.Gripper=0;
    taskEXIT_CRITICAL();
    return Snapshot;
}
RemoteControl_TelemetryStatus RemoteControl_GetTelemetry(void)
{
    taskENTER_CRITICAL();
    RemoteControl_TelemetryStatus Status=RemoteControl_Telemetry;
    taskEXIT_CRITICAL();
    return Status;
}
