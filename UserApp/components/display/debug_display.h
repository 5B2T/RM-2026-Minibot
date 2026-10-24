#ifndef DEBUG_DISPLAY_H
#define DEBUG_DISPLAY_H
#include "robot/robot_def.h"
typedef enum {DEBUG_PAGE_CHASSIS_TEST=0,DEBUG_PAGE_STATUS,DEBUG_PAGE_SPEED,DEBUG_PAGE_PWM,DEBUG_PAGE_POWER,DEBUG_PAGE_POT,DEBUG_PAGE_COUNT} DebugDisplay_Page;
// Display task owns initialization, rendering and transfers. Returns next delay in ms.
uint32_t DebugDisplay_Step(const Robot_Status *Status, uint32_t NowMs);
void DebugDisplay_Init(void); // Software reset before scheduler start.
// Service task polls raw keys, including when the OLED is offline.
void DebugDisplay_HandleKeys(uint8_t NextPressed,uint8_t BackPressed,uint32_t NowMs);
// Task-context/debugger hooks for selecting diagnostic views.
void DebugDisplay_SetPage(DebugDisplay_Page Page);
void DebugDisplay_NextPage(void);
void DebugDisplay_ShowMenu(void);
// Atomic menu snapshot; future telemetry or debugger can inspect this.
typedef struct {DebugDisplay_Page Page; uint8_t MainMenu,Cursor,Online;} DebugDisplay_State;
DebugDisplay_State DebugDisplay_GetState(void);
#endif
