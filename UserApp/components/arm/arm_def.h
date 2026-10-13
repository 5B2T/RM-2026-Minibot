#ifndef APP_ARM_DEF_H
#define APP_ARM_DEF_H
#include <stdint.h>
#define ARM_SERVO_COUNT 5U
typedef enum {ARM_SHOULDER=0,ARM_ELBOW,ARM_WRIST_PITCH,ARM_WRIST_ROLL,ARM_GRIPPER,ARM_BASE,ARM_JOINT_COUNT} Arm_Joint;
typedef enum {ARM_STOP=0,ARM_PRESET,ARM_POSITION,ARM_RATE} Arm_Action;
typedef struct
{
    Arm_Action Action;
    Arm_Joint Joint;
    float Value; // POSITION degrees; RATE -1..1. STOP/PRESET ignore Value/Joint.
    uint32_t DurationMs; // All motion requests expire, including preset movement.
} Arm_Command;
typedef enum {ARM_OK=0,ARM_EXPIRED,ARM_INVALID_COMMAND,ARM_BAD_SAMPLE,ARM_HARDWARE_ERROR} Arm_Fault;
typedef struct
{
    float CommandedAngleDeg[ARM_SERVO_COUNT]; // Position joints only; continuous entries unused. Not measured angles.
    float Rate[ARM_SERVO_COUNT];
    uint16_t PulseUs[ARM_SERVO_COUNT];
    int64_t BaseCounts; // Relative to initialization, no absolute homing.
    float BaseRelativeAngleDeg,BaseRpm;
    int16_t BasePWM;
    uint32_t TickMs;
    Arm_Joint SelectedJoint;
    Arm_Action Action;
    Arm_Fault Fault;
    uint8_t Ready,Active; // Ready = software initialized, not detection of connected actuators.
} Arm_Status;
#endif
