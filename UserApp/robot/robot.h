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
uint32_t Robot_DisplayStep(void); // Power/display service owner; next polling delay in ms.
// Each component is copied atomically; chassis and power have independent timestamps.
void Robot_GetStatus(Robot_Status *Status);
// Local commissioning / future command ingress. Task context, never ISR.
// Motion requires an explicit bounded request; arm commands are not in radio frames yet.
uint8_t Robot_ArmSubmit(const Arm_Command *Command);
// Diagnostic-session stop never cancels a radio-owned action. Explicit ARM STOP remains global.
uint8_t Robot_ArmDebugAvailable(void);
void Robot_ArmDebugStop(void);
void Robot_ArmStop(void); // Dominates queued motion, applied by control task next cycle.
#endif
