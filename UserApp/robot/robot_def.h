#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H
#include <stdint.h>
#include "components/arm/arm_def.h"
#include "components/chassis/chassis_test_def.h"
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
    float SocPercent,RemainingMah; // Voltage estimate, nominal capacity; not accuracy certified.
    uint8_t SocValid; // Requires Valid too; unavailable on sensor fault or stale sample.
    uint32_t TickMs; // Last successful sample, including across tick wrap.
    uint8_t Valid; // Numbers retained on error must not be displayed as valid.
    Robot_PowerState State;
} Robot_PowerStatus;
#define ROBOT_TELEMETRY_CHANNELS 5U
// Diagnostic pot feedback from chip 2. AngleCdeg is the provisional linear ADC scale,
// not a calibrated joint angle, and it never participates in arm control.
typedef struct
{
    uint16_t Raw[ROBOT_TELEMETRY_CHANNELS];
    uint16_t AngleCdeg[ROBOT_TELEMETRY_CHANNELS]; // 0.01 deg.
    uint32_t TickMs; // Telemetry arrival tick, independent of the command lease.
    uint8_t Valid; // Chip 2 reported a good conversion for the frame received.
    uint8_t Online; // A frame arrived within ROBOT_TELEMETRY_TIMEOUT_MS.
} Robot_Telemetry;
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
    Arm_Status Arm; // Commanded pose and relative base feedback, no absolute arm sensing.
    Robot_Telemetry Telemetry; // Read-only pot feedback from the remote, display only.
    ChassisTest_Status ChassisTest;
} Robot_Status;
#endif
