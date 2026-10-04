#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H
#include "robot/robot_def.h"
void PowerMonitor_Init(void); // Software reset before tasks; no I2C transactions.
void PowerMonitor_Step(uint32_t NowMs); // Service-task owner, at most one transaction.
Robot_PowerStatus PowerMonitor_GetStatus(uint32_t NowMs); // Atomic, freshness checked.
#endif
