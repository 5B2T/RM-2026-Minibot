#include "arm_targets.h"
#include "control/remote_control.h"
#include "remote_def.h"
#include <string.h>

// Pot mode entry requires valid, settled potentiometers and an explicit press
// after arming; never queue an entry across a lock or input loss.
#define ARM_TARGET_STABLE_MS 100U
#define ARM_TARGET_STABLE_COUNTS 12U
#define ARM_TARGET_ENTRY_WAIT_MS 3000U
#define ARM_TARGET_LOSS_HOLD_MS 200U
// Target slew per received sample (20 ms): 5 deg per frame, roughly 250 deg/s.
// The main controller still applies its own 90 deg/s position ramp.
#define ARM_TARGET_MAX_STEP_CDEG 500

static const ArmTarget_Cal DefaultCal[ARM_TARGET_CHANNELS] =
{
    // User-paired command-coordinate calibration, refreshed 2026-10-28.
    // J1: new paired95/5deg anchors. J2/J4 retain the earlier three-pose fit.
    // J3: repeatable190deg at ADC2023 and270deg at3056; inconsistent later240deg pose excluded.
    // Keep rail values 0/4095 invalid: once active, hold only the affected channel.
    // Base zero ADC1964 retained. User reports90deg input gives about80deg at rest:
    // provisional9/8 correction about zero; do not alter encoder CPR or motor polarity.
    // These are command coordinates, not external joint-position feedback.
    {1, 4094, -18277, 16844, -1, -18000, 18000}, // base:9/8 gain, original zero and direction
    {1, 4094, -13525, 20870, -1, 500, 15000}, // shoulder: ADC1354=95deg, ADC2425=5deg
    {1, 4094, 2341, 37323, 1, 2300, 18000}, // elbow
    {1, 4094, 3341, 35039, 1, 2200, 27000}, // pitch
    {1, 4094, -18543, 15408, 1, 0, 18000}, // roll
};

static ArmTarget_Cal Cal[ARM_TARGET_CHANNELS];
static uint16_t Prev[ARM_TARGET_CHANNELS];
static int32_t FilteredAdc[ARM_TARGET_CHANNELS]; // Q8, 1/4 update at 20 ms.
static uint8_t FilterMask;
static int32_t Target[ARM_TARGET_CHANNELS];
static uint8_t Mask, Active, Stable, LastPress, EntryPending;
static uint32_t StableTick, EntryTick, LossTick;
static uint8_t LossPending;
static ArmTargets_Status Snapshot;

static const uint16_t ToggleKey = KEY_MASK(KEY_R_PRESS);
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
    memset(FilteredAdc,0,sizeof(FilteredAdc));FilterMask=0;
    memset(Target, 0, sizeof(Target));
    memset(&Snapshot, 0, sizeof(Snapshot));
    Mask = Active = Stable = LastPress = EntryPending = 0;
    StableTick = EntryTick = LossTick = 0;LossPending=0;
}

void ArmTargets_Update(const uint16_t Pots[ARM_TARGET_CHANNELS], uint8_t Valid,
                       uint16_t Keys, uint32_t NowMs)
{
    uint8_t Press = (Keys & ToggleKey) != 0;
    uint8_t Leave = ((Keys & ChordLeave) == ChordLeave);
    uint8_t Toggle = (uint8_t)(Press && !LastPress);
    LastPress = Press;
    Remote_Status Status = Controller_GetStatus();
    Snapshot.UpdateMs = NowMs;
    if(Status.Mode != REMOTE_ACTIVE)Active=EntryPending=0;
    if (Pots == NULL || !Valid)
    {
        if(!LossPending){LossPending=1;LossTick=NowMs;}
        // An isolated acquisition/bridge glitch freezes targets, never substitutes
        // zero or invents a new target. Persistent loss exits; no automatic re-entry.
        if(Active && Status.Mode==REMOTE_ACTIVE && !Leave && !Toggle &&
           (uint32_t)(NowMs-LossTick)<ARM_TARGET_LOSS_HOLD_MS)
        {
            Mask=Stable=0;Snapshot.ValidMask=Snapshot.Stable=0;
            Snapshot.Active=1;Snapshot.EntryPending=0;
            Snapshot.EntryBlock=Status.InputValid?ARM_TARGET_ENTRY_ADC:ARM_TARGET_ENTRY_INPUT;
            return;
        }
        Mask = Active = Stable = FilterMask = EntryPending = 0;
        StableTick = NowMs;
        Snapshot.ValidMask = Snapshot.Active = Snapshot.Stable = 0;
        Snapshot.EntryPending = 0;
        Snapshot.EntryBlock = Status.InputValid?ARM_TARGET_ENTRY_ADC:ARM_TARGET_ENTRY_INPUT;
        return;
    }
    if(LossPending && Status.InputValid){LossPending=0;FilterMask=0;StableTick=NowMs;}
    uint8_t Quiet = 1;
    Mask = 0;
    for (uint8_t i = 0; i < ARM_TARGET_CHANNELS; i++)
    {
        int32_t Angle = 0;
        Snapshot.Raw[i] = Pots[i];
        if (MapAngle(i, Pots[i], &Angle))
        {
            // Reject raw invalid endpoints before filtering. While active, hold
            // this channel's last target; input/link failures still leave the mode.
            if(!(FilterMask&(1U<<i)))FilteredAdc[i]=(int32_t)Pots[i]*256;
            else FilteredAdc[i]+=((int32_t)Pots[i]*256-FilteredAdc[i])/4;
            FilterMask|=(uint8_t)(1U<<i);
            uint16_t Smooth=(uint16_t)((FilteredAdc[i]+128)/256);
            uint16_t Delta=Smooth>Prev[i]?Smooth-Prev[i]:Prev[i]-Smooth;
            if(Delta>ARM_TARGET_STABLE_COUNTS)Quiet=0;
            Prev[i]=Smooth;
            (void)MapAngle(i,Smooth,&Angle);
            Snapshot.AngleCdeg[i] = Angle;
            Mask |= (uint8_t)(1U << i);
        }
        else {FilterMask&=(uint8_t)~(1U<<i);Quiet=0;} // Keep last mapped value; never substitute zero.
    }
    if(!Quiet)StableTick=NowMs;
    Stable=(uint8_t)((uint32_t)(NowMs-StableTick)>=ARM_TARGET_STABLE_MS);
    Snapshot.ValidMask = Mask;
    Snapshot.Stable = Stable;
    // UART/V4 diagnostics describe the local pots even when the upstream input
    // board is absent. Only control targets freeze; never gate ADC/angle reporting.
    if(!Status.InputValid)
    {
        if(!LossPending){LossPending=1;LossTick=NowMs;}
        EntryPending=0;
        if(Status.Mode!=REMOTE_ACTIVE || Leave || Toggle ||
           (uint32_t)(NowMs-LossTick)>=ARM_TARGET_LOSS_HOLD_MS)Active=0;
        Snapshot.Active=Active;Snapshot.EntryPending=0;
        Snapshot.EntryBlock=ARM_TARGET_ENTRY_INPUT;
        return;
    }
    if (Status.Mode != REMOTE_ACTIVE) Active = EntryPending = 0;
    if (Leave) Active = EntryPending = 0;
    if (Toggle)
    {
        if (Active || EntryPending) Active = EntryPending = 0;
        else if (!Leave && Status.Mode == REMOTE_ACTIVE && Status.InputValid)
        {EntryPending = 1; EntryTick = NowMs;}
    }
    uint8_t Expired = EntryPending && (uint32_t)(NowMs - EntryTick) >= ARM_TARGET_ENTRY_WAIT_MS;
    if (Expired) EntryPending = 0;
    if (EntryPending && Stable && Mask == MaskAll)
    {
        Active = 1; EntryPending = 0;
        for (uint8_t i = 0; i < ARM_TARGET_CHANNELS; i++) Target[i] = Snapshot.AngleCdeg[i];
    }
    Snapshot.EntryPending = EntryPending;
    if (!Status.InputValid) Snapshot.EntryBlock = ARM_TARGET_ENTRY_INPUT;
    else if (Status.Mode != REMOTE_ACTIVE) Snapshot.EntryBlock = ARM_TARGET_ENTRY_LOCKED;
    else if (Expired || (!Toggle && Snapshot.EntryBlock == ARM_TARGET_ENTRY_EXPIRED && !Active && !EntryPending))
        Snapshot.EntryBlock = ARM_TARGET_ENTRY_EXPIRED;
    else if (Mask != MaskAll) Snapshot.EntryBlock = ARM_TARGET_ENTRY_ADC;
    else if (!Stable) Snapshot.EntryBlock = ARM_TARGET_ENTRY_SETTLING;
    else Snapshot.EntryBlock = ARM_TARGET_ENTRY_READY;
    for (uint8_t i = 0; Active && i < ARM_TARGET_CHANNELS; i++)
    {
        if(!(Mask&(1U<<i)))continue; // Out-of-range raw ADC holds this channel's last sent target.
        int32_t Step = (int32_t)Snapshot.AngleCdeg[i] - Target[i];
        if (Step > ARM_TARGET_MAX_STEP_CDEG) Step = ARM_TARGET_MAX_STEP_CDEG;
        if (Step < -ARM_TARGET_MAX_STEP_CDEG) Step = -ARM_TARGET_MAX_STEP_CDEG;
        Target[i] += Step;
    }
    Snapshot.Active = Active;
}

void ArmTargets_Apply(RemoteControl_FrameTypeDef *Frame)
{
    if (Frame == NULL || !Active) return;
    if (Frame->Version != REMOTE_CONTROL_VERSION_ROBOT) return;
    if (Frame->Flags & (REMOTE_CONTROL_FLAG_STOP | REMOTE_CONTROL_FLAG_RESUME)) return;
    Remote_Status Status = Controller_GetStatus();
    if (Status.Mode != REMOTE_ACTIVE) return; // Brief invalid input holds the last targets until Update expires it.
    Frame->Flags |= REMOTE_CONTROL_FLAG_ARM_TARGETS;
    for (uint8_t i = 0; i < ARM_TARGET_CHANNELS; i++)
    {
        // 0.01 deg internally, 0.1 deg on the wire so the full 333 deg base fits an int16.
        int32_t Wire = Clamp32(Target[i] / 10, -32768, 32767);
        uint16_t Bits = (uint16_t)(int16_t)Wire;
        Frame->Reserved[ARM_TARGET_RESERVED_OFFSET + 2U * i] = (uint8_t)(Bits & 0xffU);
        Frame->Reserved[ARM_TARGET_RESERVED_OFFSET + 2U * i + 1U] = (uint8_t)(Bits >> 8);
    }
    // Mode entry required all five valid once. This mask describes the complete
    // command (fresh or held), while Snapshot.ValidMask/POTRAW MASK show raw validity.
    Frame->Reserved[ARM_TARGET_MASK_INDEX] = MaskAll;
    Frame->Reserved[ARM_TARGET_MODE_INDEX] = 1;
    Frame->Reserved[17] = 0;
}

ArmTargets_Status ArmTargets_GetStatus(void) { return Snapshot; }

const ArmTarget_Cal *ArmTargets_Calibration(uint8_t Channel)
{
    return Channel < ARM_TARGET_CHANNELS ? &Cal[Channel] : NULL;
}
uint16_t ArmTargets_TelemetryAngle(uint8_t Channel)
{
    if(Channel>=ARM_TARGET_CHANNELS || !(Snapshot.ValidMask&(1U<<Channel)))return 0;
    int32_t Angle=Snapshot.AngleCdeg[Channel];
    // V4 stays unsigned 0..333 deg. Only the base uses this normalized display
    // coordinate; V3 commands and POTRAW/ANG retain signed relative joint angles.
    if(Channel==0)
    {
        const ArmTarget_Cal *C=&Cal[Channel];
        int32_t Low=Clamp32(C->DegMinCdeg,C->SoftMinCdeg,C->SoftMaxCdeg);
        int32_t High=Clamp32(C->DegMaxCdeg,C->SoftMinCdeg,C->SoftMaxCdeg);
        int32_t Span=High-Low;
        if(Span<=0)return 0;
        Angle=(int32_t)((int64_t)(Angle-Low)*33300/Span);
    }
    return (uint16_t)Clamp32(Angle,0,33300);
}
