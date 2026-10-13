#ifndef APP_ARM_H
#define APP_ARM_H
#include "components/arm/arm_def.h"
// Hardware/state owner is the robot control task; Init is before scheduling.
uint8_t Arm_Init(void);
uint8_t Arm_CommandValid(const Arm_Command *Command);
uint8_t Arm_Request(const Arm_Command *Command,uint32_t CreatedMs,uint32_t NowMs);
void Arm_Stop(void); // Continuous joints stop; position joints hold last commanded pose.
void Arm_Disable(void); // Initialization/fatal fault: stop all PWM and clear readiness.
void Arm_Step(float DtS,uint32_t NowMs);
Arm_Status Arm_GetStatus(void);
void Arm_ResetSampling(void);
#endif
