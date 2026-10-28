#ifndef APP_ARM_DEF_H
#define APP_ARM_DEF_H
#include <stdint.h>
#define ARM_SERVO_COUNT 5U
typedef enum {ARM_SHOULDER=0,ARM_ELBOW,ARM_WRIST_PITCH,ARM_WRIST_ROLL,ARM_GRIPPER,ARM_BASE,ARM_JOINT_COUNT} Arm_Joint;
typedef enum {ARM_STOP=0,ARM_PRESET,ARM_POSITION,ARM_RATE,ARM_NEUTRAL,ARM_ZERO,ARM_INITIALIZE,ARM_REMOTE,ARM_FOLLOW,ARM_CLEAR} Arm_Action;
typedef enum {ARM_SEQUENCE_ZERO=0,ARM_SEQUENCE_DEPLOYED,ARM_SEQUENCE_UNKNOWN} Arm_SequencePose;
// Stable semantic IDs for separate buttons/host controls; user slots are not configured yet.
typedef enum {ARM_POSE_FOLDED=0,ARM_POSE_EXTENDED,ARM_POSE_USER1,ARM_POSE_USER2,ARM_POSE_COUNT} Arm_PoseId;
typedef struct
{
    Arm_Action Action;
    Arm_Joint Joint;
    float Value; // POS: J1 5..150, J2 23..180, J3 22..270, J4 0..180, J5 6..45 deg.
                 // BASE POS: signed -180..180 deg relative to manually aligned boot zero.
                 // RATE -1..1: positive raises J1/J2/J3 (J3 angle decreases).
    uint32_t DurationMs; // Requests expire, including preset and queued zero confirmation.
} Arm_Command;
typedef enum {ARM_OK=0,ARM_EXPIRED,ARM_INVALID_COMMAND,ARM_BAD_SAMPLE,ARM_HARDWARE_ERROR,ARM_NEEDS_ZERO,ARM_NEEDS_CALIBRATION,ARM_BASE_FEEDBACK_ERROR} Arm_Fault;
typedef struct
{
    float CommandedAngleDeg[ARM_SERVO_COUNT]; // Position joints only; valid after PositionEnabledMask is set, not measured.
    float Rate[ARM_SERVO_COUNT];
    uint16_t PulseUs[ARM_SERVO_COUNT];
    int64_t BaseCounts; // Relative to last software zero, no absolute homing.
    float BaseRelativeAngleDeg,BaseRpm;
    float BaseRawRpm,BaseTargetRpm; // Filtered BaseRpm remains in the existing CSV slot.
    float BasePositionTargetDeg,BasePositionReferenceDeg,BasePositionErrorDeg;
    uint8_t BasePositionEnabled,BasePositionPickedUp,BasePositionAtTarget;
    int16_t BasePWM;
    uint32_t TickMs;
    Arm_Joint SelectedJoint;
    Arm_Action Action;
    Arm_Fault Fault;
    uint8_t Ready,Active; // Ready = software initialized, not detection of connected actuators.
    uint8_t PositionEnabledMask; // Signal-free until explicit POS/PRESET or sequence activation.
    uint32_t ZeroGeneration; // 1 after automatic startup zero; increments on manual confirmation.
    Arm_SequencePose SequencePose; // Open-loop reference, not measured pose.
    uint8_t SequenceRunning,SequenceReturning,SequenceDoneMask; // Done bits: J1..J5.
    uint8_t SequenceStage; // Deploy 0 -> 255(J2/J3/J4) -> 4(grip); fold 2(+grip/wrist0/base0),1,3,0.
} Arm_Status;
#endif
