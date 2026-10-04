#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H
#include <stdint.h>
#define ROBOT_WHEEL_COUNT 4U
// Canonical wheel position order; actual physical channels are configured separately.
typedef enum {ROBOT_WHEEL_FL=0,ROBOT_WHEEL_FR,ROBOT_WHEEL_RL,ROBOT_WHEEL_RR} Robot_WheelPosition;
typedef struct
{
    float Vx; // m/s, forward positive.
    float Vy; // m/s, left positive.
    float Wz; // rad/s, counterclockwise positive viewed from above.
} Robot_ChassisVelocity;
// M1..M4 retain physical channel order. Rim m/s, forward positive after calibration.
typedef struct
{
    float WheelSpeed[ROBOT_WHEEL_COUNT];
    uint8_t Enabled;
} Robot_WheelCommand;

typedef enum {ROBOT_COMMAND_WHEELS=0,ROBOT_COMMAND_VELOCITY=1} Robot_CommandMode;
typedef struct
{
    Robot_CommandMode Mode;
    uint8_t Enabled;
    union
    {
        float Wheels[ROBOT_WHEEL_COUNT]; // Physical M1..M4 order.
        Robot_ChassisVelocity Velocity; // Body axes, m/s and rad/s.
    } Target;
} Robot_ChassisCommand;

typedef enum {ROBOT_POWER_OFFLINE=0,ROBOT_POWER_WAITING,ROBOT_POWER_VALID,
    ROBOT_POWER_OVERFLOW,ROBOT_POWER_TIMEOUT,ROBOT_POWER_CONFIG_ERROR,
    ROBOT_POWER_INVALID,ROBOT_POWER_STALE} Robot_PowerState;
typedef struct
{
    float VoltageV,CurrentA,PowerW; // Battery source side; discharge current/power positive.
    uint32_t TickMs; // Last successful sample, including across tick wrap.
    uint8_t Valid; // Numbers retained on error must not be displayed as valid.
    Robot_PowerState State;
} Robot_PowerStatus;
typedef struct
{
    float MeasuredSpeed[ROBOT_WHEEL_COUNT];
    float TargetSpeed[ROBOT_WHEEL_COUNT];
    float PWM[ROBOT_WHEEL_COUNT]; // Forward-positive control output before motor polarity, +/-999.
    Robot_CommandMode CommandMode; // Last active command type.
    uint32_t TickMs;
    uint8_t Connected;
    uint8_t Armed; // Command link authorizes motion; independent of actual driver enable.
    uint8_t MotorsEnabled;
    Robot_PowerStatus Power; // Independently sampled with its own timestamp and validity.
} Robot_Status;
#endif
