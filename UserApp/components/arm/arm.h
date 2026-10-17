#ifndef APP_ARM_H
#define APP_ARM_H
#include "components/arm/arm_def.h"
// Hardware/state owner is the robot control task; Init is before scheduling.
uint8_t Arm_Init(void);
uint8_t Arm_CommandValid(const Arm_Command *Command);
// ARM_ZERO stops motion and resets relative feedback without moving to a preset.
// Manual zero remains available to serial commissioning only; no remote key binding.
// ARM_INITIALIZE toggles deploy/return once complete; duplicate requests during a sequence are ignored.
uint8_t Arm_Request(const Arm_Command *Command,uint32_t CreatedMs,uint32_t NowMs);
void Arm_Stop(void); // Continuous joints stop; position joints hold last commanded pose.
void Arm_Disable(void); // Initialization/fatal fault: stop all PWM and clear readiness.
void Arm_Step(float DtS,uint32_t NowMs);
Arm_Status Arm_GetStatus(void);
void Arm_ResetSampling(void);
// Control-task owner; independent base, selected joint and gripper. Neutral inputs
// preserve an active automatic sequence. Fresh radio lease required each control tick.
void Arm_RemoteInput(float Base,Arm_Joint Joint,float Rate,uint8_t Initialize,uint8_t Gripper,uint32_t NowMs);
#endif
