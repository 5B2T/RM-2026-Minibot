#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H
#include <stdint.h>
#define REMOTE_CONTROL_FRAME_SIZE 32U
#define REMOTE_CONTROL_SPEED_SCALE 1000.0f
#define REMOTE_CONTROL_FLAG_STOP 0x01U
#define REMOTE_CONTROL_FLAG_RESUME 0x02U
// Wire layout: A5, 01, sequence, flags, four int16 little-endian mm/s,
// 18 reserved bytes, CRC16 (0xffff init, 0xa001 polynomial), little-endian.
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

// Pure decoder: no HAL, scheduler, command state or motor writes.
uint8_t CommandProtocol_Decode(const uint8_t *Data, uint8_t Length,
                               RemoteControl_FrameTypeDef *Frame);
#endif
