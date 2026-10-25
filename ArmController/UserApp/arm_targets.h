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

typedef struct
{
    uint16_t Raw[ARM_TARGET_CHANNELS];
    int32_t AngleCdeg[ARM_TARGET_CHANNELS];
    uint8_t ValidMask, Active, Stable;
    uint32_t UpdateMs;
} ArmTargets_Status;

void ArmTargets_Init(void);
// Keys are the debounced remote key mask; a right-stick up+down chord toggles
// pot mode, a left+right chord leaves it, and any lock leaves it too.
void ArmTargets_Update(const uint16_t Pots[ARM_TARGET_CHANNELS], uint8_t Valid,
                       uint16_t Keys, uint32_t NowMs);
// Injects the extension into a V3 frame. Silent unless pot mode is active.
void ArmTargets_Apply(RemoteControl_FrameTypeDef *Frame);
ArmTargets_Status ArmTargets_GetStatus(void);
// Read-only view for commissioning tools; never enumerated for modification.
const ArmTarget_Cal *ArmTargets_Calibration(uint8_t Channel);
#endif
