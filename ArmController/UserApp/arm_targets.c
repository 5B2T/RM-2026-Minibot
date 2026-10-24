#include "arm_targets.h"
#include "control/remote_control.h"
#include "remote_def.h"
#include <string.h>

// Pot mode entry needs the sticks to be quiet, so a floating or freshly rewired
// channel cannot drive the arm the moment the operator arms the remote.
#define ARM_TARGET_STABLE_MS 300U
#define ARM_TARGET_STABLE_COUNTS 12U
// Target slew per received sample (20 ms): 5 deg per frame, roughly 250 deg/s.
// The main controller still applies its own 90 deg/s position ramp.
#define ARM_TARGET_MAX_STEP_CDEG 500

static const ArmTarget_Cal DefaultCal[ARM_TARGET_CHANNELS] =
{
    // Base: ALPS RDC503013A, 10 k, 333 deg single turn.
    {0, 4095, 0, 33300, 1, 500, 32800},
    // Shoulder: servo travel 295 deg.
    {0, 4095, 0, 33300, 1, 1000, 28500},
    // Elbow: continuous servo, absolute angle is a reference only.
    {0, 4095, 0, 33300, 1, 0, 33300},
    // Wrist pitch: continuous servo, absolute angle is a reference only.
    {0, 4095, 0, 33300, 1, 0, 33300},
    // Wrist roll: servo travel 180 deg.
    {0, 4095, 0, 33300, 1, 500, 17500}
};

static ArmTarget_Cal Cal[ARM_TARGET_CHANNELS];
static uint16_t Prev[ARM_TARGET_CHANNELS];
static int32_t Target[ARM_TARGET_CHANNELS];
static uint8_t Mask, Active, Stable, LastChord;
static uint32_t StableTick;
static ArmTargets_Status Snapshot;

static const uint16_t ChordToggle = KEY_MASK(KEY_R_UP) | KEY_MASK(KEY_R_DOWN);
static const uint16_t ChordLeave = KEY_MASK(KEY_R_LEFT) | KEY_MASK(KEY_R_RIGHT);
static const uint8_t MaskAll = (uint8_t)((1U << ARM_TARGET_CHANNELS) - 1U);

static int32_t Clamp32(int32_t Value, int32_t Low, int32_t High)
{
    if (Low > High) { int32_t Swap = Low; Low = High; High = Swap; }
    if (Value < Low) Value = Low;
    if (Value > High) Value = High;
    return Value;
}

static uint8_t MapAngle(uint8_t Channel, uint16_t Adc, int32_t *AngleCdeg)
{
    const ArmTarget_Cal *C = &Cal[Channel];
    if (Adc < C->AdcMin || Adc > C->AdcMax) return 0;
    int32_t Den = (int32_t)C->AdcMax - (int32_t)C->AdcMin;
    if (Den <= 0) return 0;
    int32_t Num = (int32_t)Adc - (int32_t)C->AdcMin;
    // Dir = -1 reverses the mapping so ADC growth lowers the joint angle.
    if (C->Dir < 0) Num = Den - Num;
    int32_t Span = (int32_t)C->DegMaxCdeg - (int32_t)C->DegMinCdeg;
    int32_t Angle = (int32_t)C->DegMinCdeg + Num * Span / Den;
    *AngleCdeg = Clamp32(Angle, C->SoftMinCdeg, C->SoftMaxCdeg);
    return 1;
}

void ArmTargets_Init(void)
{
    memcpy(Cal, DefaultCal, sizeof(Cal));
    memset(Prev, 0, sizeof(Prev));
    memset(Target, 0, sizeof(Target));
    memset(&Snapshot, 0, sizeof(Snapshot));
    Mask = Active = Stable = LastChord = 0;
    StableTick = 0;
}

void ArmTargets_Update(const uint16_t Pots[ARM_TARGET_CHANNELS], uint8_t Valid,
                       uint16_t Keys, uint32_t NowMs)
{
    uint8_t Chord = ((Keys & ChordToggle) == ChordToggle);
    uint8_t Leave = ((Keys & ChordLeave) == ChordLeave);
    uint8_t Toggle = (uint8_t)(Chord && !LastChord);
    LastChord = Chord;
    Snapshot.UpdateMs = NowMs;
    if (Pots == NULL || !Valid)
    {
        Mask = Active = Stable = 0;
        Snapshot.ValidMask = Snapshot.Active = Snapshot.Stable = 0;
        return;
    }
    uint8_t Quiet = 1;
    for (uint8_t i = 0; i < ARM_TARGET_CHANNELS; i++)
    {
        uint16_t Delta = (Pots[i] > Prev[i]) ? (uint16_t)(Pots[i] - Prev[i]) : (uint16_t)(Prev[i] - Pots[i]);
        if (Delta > ARM_TARGET_STABLE_COUNTS) Quiet = 0;
        Prev[i] = Pots[i];
    }
    if (!Quiet) StableTick = NowMs;
    Stable = (uint8_t)((uint32_t)(NowMs - StableTick) >= ARM_TARGET_STABLE_MS);
    Mask = 0;
    for (uint8_t i = 0; i < ARM_TARGET_CHANNELS; i++)
    {
        int32_t Angle = 0;
        Snapshot.Raw[i] = Pots[i];
        if (MapAngle(i, Pots[i], &Angle))
        {
            Snapshot.AngleCdeg[i] = Angle;
            Mask |= (uint8_t)(1U << i);
        }
        else Snapshot.AngleCdeg[i] = 0;
    }
    Snapshot.ValidMask = Mask;
    Snapshot.Stable = Stable;
    if (Controller_GetStatus().Mode != REMOTE_ACTIVE) Active = 0;
    if (Leave) Active = 0;
    if (Toggle)
    {
        if (Active) Active = 0;
        // Pot mode only takes over an already armed remote: entering while locked would
        // let the arm jump the moment the operator arms.
        else if (Stable && Mask == MaskAll && Controller_GetStatus().Mode == REMOTE_ACTIVE)
        {
            Active = 1;
            // Enter on the current pose: the main controller ramps from there.
            for (uint8_t i = 0; i < ARM_TARGET_CHANNELS; i++) Target[i] = Snapshot.AngleCdeg[i];
        }
    }
    if (Active && Mask != MaskAll) Active = 0;
    for (uint8_t i = 0; Active && i < ARM_TARGET_CHANNELS; i++)
    {
        int32_t Step = (int32_t)Snapshot.AngleCdeg[i] - Target[i];
        if (Step > ARM_TARGET_MAX_STEP_CDEG) Step = ARM_TARGET_MAX_STEP_CDEG;
        if (Step < -ARM_TARGET_MAX_STEP_CDEG) Step = -ARM_TARGET_MAX_STEP_CDEG;
        Target[i] += Step;
    }
    Snapshot.Active = Active;
}

void ArmTargets_Apply(RemoteControl_FrameTypeDef *Frame)
{
    if (Frame == NULL || !Active || Mask != MaskAll) return;
    if (Frame->Version != REMOTE_CONTROL_VERSION_ROBOT) return;
    if (Frame->Flags & (REMOTE_CONTROL_FLAG_STOP | REMOTE_CONTROL_FLAG_RESUME)) return;
    if (Controller_GetStatus().Mode != REMOTE_ACTIVE) return;
    Frame->Flags |= REMOTE_CONTROL_FLAG_ARM_TARGETS;
    for (uint8_t i = 0; i < ARM_TARGET_CHANNELS; i++)
    {
        // 0.01 deg internally, 0.1 deg on the wire so the full 333 deg base fits an int16.
        int32_t Wire = Clamp32(Target[i] / 10, -32768, 32767);
        uint16_t Bits = (uint16_t)(int16_t)Wire;
        Frame->Reserved[ARM_TARGET_RESERVED_OFFSET + 2U * i] = (uint8_t)(Bits & 0xffU);
        Frame->Reserved[ARM_TARGET_RESERVED_OFFSET + 2U * i + 1U] = (uint8_t)(Bits >> 8);
    }
    Frame->Reserved[ARM_TARGET_MASK_INDEX] = Mask;
    Frame->Reserved[ARM_TARGET_MODE_INDEX] = 1;
    Frame->Reserved[17] = 0;
}

ArmTargets_Status ArmTargets_GetStatus(void) { return Snapshot; }

const ArmTarget_Cal *ArmTargets_Calibration(uint8_t Channel)
{
    return Channel < ARM_TARGET_CHANNELS ? &Cal[Channel] : NULL;
}
