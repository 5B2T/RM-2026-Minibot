#ifndef MECANUM_H
#define MECANUM_H
#include <stdint.h>
#define MECANUM_WHEEL_COUNT 4U
// 45-degree rollers in X arrangement, rectangular chassis, rotation about center.
// Order FL/FR/RL/RR; wheel speed m/s, forward positive. Vx forward, Vy left,
// Wz counterclockwise rad/s; wheelbase/track are center distances in meters.
// Uniform limiting preserves wheel-speed ratios. Invalid input clears output, returns 0.
uint8_t Mecanum_Inverse(float Vx, float Vy, float Wz, float WheelbaseM, float TrackM,
                        float WheelLimitMps, float Wheels[MECANUM_WHEEL_COUNT]);
#endif
