#ifndef BSP_EMERGENCY_H
#define BSP_EMERGENCY_H
#include <stdint.h>
// Fault context only. Never returns; coordinates are not silently re-zeroed by reset.
void BSP_EmergencyHalt(uint32_t Reason);
#endif
