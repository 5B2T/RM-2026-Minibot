#ifndef BSP_BUTTONS_H
#define BSP_BUTTONS_H
#include <stdint.h>
#define BSP_BUTTON_NEXT 1U
#define BSP_BUTTON_BACK 2U
uint8_t BSP_Buttons_Read(void); // Raw pressed bits; display component debounces.
#endif
