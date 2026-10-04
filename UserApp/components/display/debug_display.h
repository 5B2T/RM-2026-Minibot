#ifndef DEBUG_DISPLAY_H
#define DEBUG_DISPLAY_H
#include "robot/robot_def.h"
typedef enum {DEBUG_PAGE_STATUS=0,DEBUG_PAGE_SPEED,DEBUG_PAGE_PWM,DEBUG_PAGE_COUNT} DebugDisplay_Page;
// Display task owns initialization, rendering and transfers. Returns next delay in ms.
uint32_t DebugDisplay_Step(const Robot_Status *Status, uint32_t NowMs);
// Task-context input hooks. SetPage/NextPage pause automatic paging.
void DebugDisplay_SetPage(DebugDisplay_Page Page);
void DebugDisplay_NextPage(void);
void DebugDisplay_SetAuto(uint8_t Enabled);
// Atomic menu snapshot; future telemetry or debugger can inspect this.
typedef struct {DebugDisplay_Page Page; uint8_t Auto; uint8_t Online;} DebugDisplay_State;
DebugDisplay_State DebugDisplay_GetState(void);
#endif
