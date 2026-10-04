#ifndef BSP_TIME_H
#define BSP_TIME_H
#include <stdint.h>
uint32_t BSP_TimeMs(void); // uint32_t wrapping tick; compare by unsigned subtraction.
void BSP_WaitMs(uint32_t Ms); // Task context; yields at runtime, HAL delay before scheduler.
#endif
