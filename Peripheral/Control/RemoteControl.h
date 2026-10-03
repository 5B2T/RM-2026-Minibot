#ifndef RM_2026_MINIBOT_REMOTE_CONTROL_H
#define RM_2026_MINIBOT_REMOTE_CONTROL_H

#include <stdint.h>

#define REMOTE_CONTROL_FRAME_SIZE 32U
#define REMOTE_CONTROL_TIMEOUT_MS 100U
#define REMOTE_CONTROL_SPEED_SCALE 1000.0f
#define REMOTE_CONTROL_FLAG_STOP 0x01U

typedef struct
{
    uint8_t Header;
    uint8_t Version;
    uint8_t Sequence;
    uint8_t Flags;
    int16_t M1;
    int16_t M2;
    int16_t M3;
    int16_t M4;
    uint8_t Reserved[18];
    uint16_t Frame_CRC;
} RemoteControl_FrameTypeDef;

_Static_assert(sizeof(RemoteControl_FrameTypeDef)==REMOTE_CONTROL_FRAME_SIZE, "RemoteControl_FrameTypeDef size error");

void RemoteControl_Init(void);
uint8_t RemoteControl_Process(const uint8_t* Data, uint8_t Length);
void RemoteControl_Update(void);
uint8_t RemoteControl_IsConnected(void);
uint32_t RemoteControl_LastTick(void);

#endif
