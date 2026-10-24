#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H
#include <stdint.h>
#define REMOTE_TELEMETRY_FRAME_SIZE 32U
#define REMOTE_TELEMETRY_VERSION 0x04U
#define REMOTE_TELEMETRY_CHANNELS 5U
#define REMOTE_TELEMETRY_FLAG_POTS 0x01U
// Set while the controller is following the potentiometers (pot mode). The angles
// then come from the calibration table instead of the provisional linear scale.
#define REMOTE_TELEMETRY_FLAG_POT_MODE 0x02U
#define REMOTE_TELEMETRY_ANGLE_MAX 18000U
#define REMOTE_TELEMETRY_ADC_MAX 4095U
// Wire layout: A5, version 0x04, sequence, flags, five little-endian uint16 angles
// (0.01 deg), five little-endian uint16 raw ADC samples, six reserved zero bytes,
// CRC16 (0xffff init, 0xa001 polynomial), little-endian. Same size and framing as
// the command frames, so both share one radio pipe and differ only by version.
typedef struct
{
    uint8_t Header;
    uint8_t Version;
    uint8_t Sequence;
    uint8_t Flags;
    uint16_t AngleCdeg[REMOTE_TELEMETRY_CHANNELS];
    uint16_t Raw[REMOTE_TELEMETRY_CHANNELS];
    uint8_t Reserved[6];
    uint16_t Frame_CRC;
} RemoteTelemetry_FrameTypeDef;

_Static_assert(sizeof(RemoteTelemetry_FrameTypeDef)==REMOTE_TELEMETRY_FRAME_SIZE, "RemoteTelemetry_FrameTypeDef size error");

// Chip 2 samples five potentiometers and publishes them here; the arm mapping is the
// provisional linear ADC scale, not a calibrated joint angle, and telemetry never
// drives the arm. Both functions are pure: no HAL, scheduler or radio access.
uint8_t RemoteTelemetry_Decode(const uint8_t *Data, uint8_t Length,
                               RemoteTelemetry_FrameTypeDef *Frame);
// Explicit wire encoding; rejects invalid frames and leaves Output unchanged on failure.
uint8_t RemoteTelemetry_Encode(const RemoteTelemetry_FrameTypeDef *Frame,
                               uint8_t Output[REMOTE_TELEMETRY_FRAME_SIZE]);
#endif
