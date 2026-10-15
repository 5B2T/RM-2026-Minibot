#ifndef APP_REMOTE_CONTROL_H
#define APP_REMOTE_CONTROL_H
#include "command/command_protocol.h"
#include "robot/robot_def.h"
// Init before scheduling. Other APIs are task-context only, not ISR APIs.
// Startup/STOP/timeout/fault: zero-speed RESUME then fresh motion frames.
// Sequence increments per frame (heartbeat included), wraps 255 -> 0.
void RemoteControl_Init(void);
uint8_t RemoteControl_Process(const uint8_t *Data, uint8_t Length); // Communication owner.
void RemoteControl_Disconnect(void); // Latch STOP; hardware is written by control task.
void RemoteControl_GetCommand(Robot_ChassisCommand *Command); // Control owner, atomic snapshot.
typedef struct
{
    uint32_t LastRxTick;
    uint32_t StopGeneration; // Advances for every valid radio STOP, including stale sequences.
    uint8_t Connected;
    uint8_t Armed;
} RemoteControl_LinkStatus;
RemoteControl_LinkStatus RemoteControl_GetLinkStatus(void); // One atomic link snapshot.
#endif
