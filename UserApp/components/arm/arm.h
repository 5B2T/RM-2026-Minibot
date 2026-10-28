#ifndef APP_ARM_H
#define APP_ARM_H
#include "components/arm/arm_def.h"
// Hardware/state owner is the robot control task; Init is before scheduling.
uint8_t Arm_Init(void);
uint8_t Arm_CommandValid(const Arm_Command *Command);
// ARM_ZERO resets relative base feedback only; it cannot read or redefine servo angles.
// Manual zero is accepted only after STOP and measured standstill; no remote key binding.
// INIT: deploy J1, parallel J2/J3/J4, open grip; fold J3+wrist0+close grip+base0, J2,wrist90,J1.
// Boot assumes J1/J2/J3/J4 at 5/23/22/90 and gripper closed at6 before power-on.
// Legacy PRESET is unsupported.
uint8_t Arm_Request(const Arm_Command *Command,uint32_t CreatedMs,uint32_t NowMs);
// Explicit semantic action, independent of the toggle. USER1/USER2 currently return0.
// A nonstandard but software-tracked pose can be folded; this does not redefine physical zero.
uint8_t Arm_GoToPose(Arm_PoseId Pose,uint32_t NowMs);
// Explicit stopped acknowledgement; preserves boot reference and all joint coordinates.
uint8_t Arm_ClearFault(void);
void Arm_Stop(void); // Stop base/jogging; all enabled servos hold last commanded position.
void Arm_Disable(void); // Initialization/fatal fault: stop all PWM and clear readiness.
void Arm_Step(float DtS,uint32_t NowMs);
Arm_Status Arm_GetStatus(void);
void Arm_ResetSampling(void);
// Control-task owner; independent base, selected joint and gripper. Neutral inputs
// preserve an active automatic sequence. Fresh radio lease required each control tick.
void Arm_RemoteInput(float Base,Arm_Joint Joint,float Rate,uint8_t Initialize,uint8_t Gripper,uint32_t NowMs);
// Wire order base,J1,J2,J3,J4: explicit mode entry starts a limited ramp from the tracked base position.
// Clamps to structural limits, ramps from current commanded/assumed boot pose.
// CreatedMs is the actual radio receive time; cached snapshots cannot extend the lease.
uint8_t Arm_FollowTargets(const float AngleDeg[5],uint8_t Gripper,uint32_t CreatedMs,uint32_t NowMs);
#endif
