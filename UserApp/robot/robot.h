#ifndef MINIBOT_ROBOT_H
#define MINIBOT_ROBOT_H
#include <stdint.h>
#include "robot/robot_def.h"
// Whole-robot entry point. Future arm/sensor components initialize here.
// Motor/encoder failure returns 0. Radio failure is recoverable and keeps chassis disarmed.
uint8_t Robot_Init(void);
void Robot_ControlReset(void); // Called once by control task before its sampling loop.
void Robot_ControlStep(float DtS); // Control owner, actual sampling interval in seconds.
uint32_t Robot_CommunicationStep(void); // Communication owner, returns next delay in ms.
// Atomic copy for future OLED/telemetry tasks; feedback reflects the last control cycle.
void Robot_GetStatus(Robot_Status *Status);
#endif
