#ifndef APP_ARM_DEF_H
#define APP_ARM_DEF_H
#include <stdint.h>
#define ARM_SERVO_COUNT 5U
typedef enum {ARM_SHOULDER=0,ARM_ELBOW,ARM_WRIST_PITCH,ARM_WRIST_ROLL,ARM_GRIPPER,ARM_BASE,ARM_JOINT_COUNT} Arm_Joint;
typedef enum {ARM_STOP=0,ARM_PRESET,ARM_POSITION,ARM_RATE,ARM_NEUTRAL,ARM_ZERO,ARM_INITIALIZE,ARM_REMOTE} Arm_Action;
typedef enum {ARM_SEQUENCE_ZERO=0,ARM_SEQUENCE_DEPLOYED,ARM_SEQUENCE_UNKNOWN} Arm_SequencePose;
typedef struct
{
    Arm_Action Action;
    Arm_Joint Joint;
    float Value; // POSITION: J1 0..150, J4 0..180, J5 closed=0/open=45 only; RATE -1..1 except J5; NEUTRAL us.
    uint32_t DurationMs; // Requests expire, including preset and queued zero confirmation.
} Arm_Command;
typedef enum {ARM_OK=0,ARM_EXPIRED,ARM_INVALID_COMMAND,ARM_BAD_SAMPLE,ARM_HARDWARE_ERROR,ARM_NEEDS_ZERO} Arm_Fault;
typedef struct
{
    float CommandedAngleDeg[ARM_SERVO_COUNT]; // Position joints only; valid after PositionEnabledMask is set, not measured.
    float Rate[ARM_SERVO_COUNT];
    uint16_t PulseUs[ARM_SERVO_COUNT];
    int64_t BaseCounts; // Relative to last software zero, no absolute homing.
    float BaseRelativeAngleDeg,BaseRpm;
    int16_t BasePWM;
    uint32_t TickMs;
    Arm_Joint SelectedJoint;
    Arm_Action Action;
    Arm_Fault Fault;
    uint8_t Ready,Active; // Ready = software initialized, not detection of connected actuators.
    uint8_t PositionEnabledMask; // Signal-free until explicit POS/PRESET or sequence activation.
    uint32_t ZeroGeneration; // 1 after automatic startup zero; increments on manual confirmation.
    Arm_SequencePose SequencePose; // Open-loop reference, not measured pose.
    uint8_t SequenceRunning,SequenceReturning,SequenceDoneMask; // Done bits: J1,J2,J3,J5.
    uint8_t SequenceStage; // 0/1/2/3 = J1/J2/J3/J5; deploy 0..3, return 3..0.
    float EstimatedContinuousDeg[2]; // J2/J3 open-loop integration from startup pose, NOT measured.
} Arm_Status;
#endif
