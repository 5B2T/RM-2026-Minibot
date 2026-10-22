#ifndef APP_ARM_H
#define APP_ARM_H
#include "components/arm/arm_def.h"
// Hardware/state owner is the robot control task; Init is before scheduling.
uint8_t Arm_Init(void);
uint8_t Arm_CommandValid(const Arm_Command *Command);
// ARM_ZERO resets relative base feedback only; it cannot read or redefine servo angles.
// Manual zero remains available to serial commissioning only; no remote key binding.
// INIT: deploy J1, parallel J2/J3/J4, open grip; fold J3+close grip, J2,J4,J1.
// Boot assumes the user manually placed J1/J2/J3/J4 at 10/23/13/180 before power-on.
// Legacy PRESET is unsupported.
uint8_t Arm_Request(const Arm_Command *Command,uint32_t CreatedMs,uint32_t NowMs);
void Arm_Stop(void); // Stop base/jogging; all enabled servos hold last commanded position.
void Arm_Disable(void); // Initialization/fatal fault: stop all PWM and clear readiness.
void Arm_Step(float DtS,uint32_t NowMs);
Arm_Status Arm_GetStatus(void);
void Arm_ResetSampling(void);
// Control-task owner; independent base, selected joint and gripper. Neutral inputs
// preserve an active automatic sequence. Fresh radio lease required each control tick.
void Arm_RemoteInput(float Base,Arm_Joint Joint,float Rate,uint8_t Initialize,uint8_t Gripper,uint32_t NowMs);
#endif
