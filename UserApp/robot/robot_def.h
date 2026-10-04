#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H
#include <stdint.h>
#define ROBOT_WHEEL_COUNT 4U
// M1..M4 retain the original physical channel order. Wheel speed is rim m/s.
typedef struct
{
    float WheelSpeed[ROBOT_WHEEL_COUNT];
    uint8_t Enabled;
} Robot_WheelCommand;

typedef struct
{
    float MeasuredSpeed[ROBOT_WHEEL_COUNT];
    float TargetSpeed[ROBOT_WHEEL_COUNT];
    float PWM[ROBOT_WHEEL_COUNT]; // Signed control output, +/-999.
    uint32_t TickMs;
    uint8_t Connected;
    uint8_t Armed; // Command link authorizes motion; independent of actual driver enable.
    uint8_t MotorsEnabled;
} Robot_Status;
#endif
