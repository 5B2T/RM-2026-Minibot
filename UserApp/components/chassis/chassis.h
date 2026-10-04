#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H
#include <stdint.h>
#include "robot/robot_def.h"
// Owns wheel sampling/closed-loop/output. All runtime APIs belong to control task.
uint8_t Chassis_Init(void);
void Chassis_ResetSampling(void);
uint8_t Chassis_Measure(float DtS);
void Chassis_Control(const Robot_ChassisCommand *Command, float DtS);
// Direct interfaces for the control-task owner; other tasks submit commands through mailbox.
void Chassis_ControlWheels(const Robot_WheelCommand *Command, float DtS);
void Chassis_ControlVelocity(const Robot_ChassisVelocity *Velocity, uint8_t Enabled, float DtS);
void Chassis_Stop(void);
void Chassis_GetFeedback(Robot_Status *Status); // Fill wheel fields for robot publication.
#endif
