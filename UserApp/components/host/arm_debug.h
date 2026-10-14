#ifndef APP_ARM_DEBUG_H
#define APP_ARM_DEBUG_H
#include <stdint.h>
// Dedicated low-priority service task; no actuator access from interrupt context.
uint32_t ArmDebug_Step(void);
#endif
