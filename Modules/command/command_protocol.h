#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H
#include <stdint.h>
#define REMOTE_CONTROL_FRAME_SIZE 32U
#define REMOTE_CONTROL_VERSION_WHEELS 0x01U
#define REMOTE_CONTROL_VERSION_VELOCITY 0x02U
#define REMOTE_CONTROL_ANGULAR_SCALE 1000.0f
#define REMOTE_CONTROL_SPEED_SCALE 1000.0f
#define REMOTE_CONTROL_FLAG_STOP 0x01U
#define REMOTE_CONTROL_FLAG_RESUME 0x02U
// Wire layout: A5, version, sequence, flags, four little-endian int16 values,
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

// V1: M1..M4 wheel-rim mm/s in physical channel order.
// V2: Vx mm/s, Vy mm/s, Wz millirad/s, zero reserved M4.
// STOP ignores speed fields; RESUME requires all four fields zero in either version.
// Pure decoder: no HAL, scheduler, command state or motor writes.
uint8_t CommandProtocol_Decode(const uint8_t *Data, uint8_t Length,
                               RemoteControl_FrameTypeDef *Frame);
// Explicit wire encoding; rejects invalid commands and leaves Output unchanged on failure.
uint8_t CommandProtocol_Encode(const RemoteControl_FrameTypeDef *Frame,
                               uint8_t Output[REMOTE_CONTROL_FRAME_SIZE]);
#endif
