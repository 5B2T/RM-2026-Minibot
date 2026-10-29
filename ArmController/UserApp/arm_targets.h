#ifndef ARM_TARGETS_H
#define ARM_TARGETS_H
#include <stdint.h>
#include "command/command_protocol.h"

// Five potentiometers describe the arm: PA0 base, PA1 shoulder, PA2 elbow,
// PA3 wrist pitch, PA4 wrist roll. The gripper has no sensor and stays on its
// remote key. This module owns the calibration table, the pot-to-angle mapping,
// the pot-mode arbitration and the wire extension of the V3 command frame.
//
// Wire extension (only present while pot mode is active and the remote is
// ACTIVE): Flags bit 0x04, Reserved[5..14] five int16 little-endian target
// angles in 0.1 deg, Reserved[15] calibrated channel mask, Reserved[16] pot
// mode, Reserved[17] zero. Frames without bit 0x04 keep the previous meaning.
// 0.1 deg is used on the wire because the base joint can exceed 327.67 deg and
// an int16 holding 0.01 deg units would overflow there; the 10 deg/s steps the
// arm already applies are far coarser than this resolution.
#define ARM_TARGET_CHANNELS 5U
#define ARM_TARGET_RESERVED_OFFSET 5U
#define ARM_TARGET_MASK_INDEX 15U
#define ARM_TARGET_MODE_INDEX 16U

typedef struct
{
    uint16_t AdcMin, AdcMax;      // Potentiometer counts that map to DegMin/DegMax.
    int32_t DegMinCdeg, DegMaxCdeg; // Joint angle at AdcMin/AdcMax, 0.01 deg.
    int8_t Dir;                   // +1 or -1, applied after the linear map.
    int32_t SoftMinCdeg, SoftMaxCdeg; // Command limits in the same unit.
} ArmTarget_Cal;

typedef enum
{
    ARM_TARGET_ENTRY_READY = 0,
    ARM_TARGET_ENTRY_LOCKED,
    ARM_TARGET_ENTRY_INPUT,
    ARM_TARGET_ENTRY_ADC,
    ARM_TARGET_ENTRY_SETTLING,
    ARM_TARGET_ENTRY_EXPIRED
} ArmTarget_EntryBlock;

typedef struct
{
    uint16_t Raw[ARM_TARGET_CHANNELS];
    int32_t AngleCdeg[ARM_TARGET_CHANNELS];
    uint8_t ValidMask, Active, Stable;
    uint32_t UpdateMs;
    uint8_t EntryPending;
    ArmTarget_EntryBlock EntryBlock;
} ArmTargets_Status;

void ArmTargets_Init(void);
// Debounced right-stick press toggles pot mode. Entry may wait up to 3s for
// valid pots stable for100ms; lock/input loss cancels pending entry. An already
// selected mode survives communication/acquisition loss with output suspended;
// fresh valid input resumes it. Explicit STOP or a mode toggle still exits.
void ArmTargets_Update(const uint16_t Pots[ARM_TARGET_CHANNELS], uint8_t Valid,
                       uint16_t Keys, uint32_t NowMs);
// Injects the extension into a V3 frame. Silent unless pot mode is active.
void ArmTargets_Apply(RemoteControl_FrameTypeDef *Frame);
ArmTargets_Status ArmTargets_GetStatus(void);
// V4 base is a normalized unsigned 0..333 deg display value, NOT the signed V3 target.
uint16_t ArmTargets_TelemetryAngle(uint8_t Channel);
// Read-only view for commissioning tools; never enumerated for modification.
const ArmTarget_Cal *ArmTargets_Calibration(uint8_t Channel);
#endif
