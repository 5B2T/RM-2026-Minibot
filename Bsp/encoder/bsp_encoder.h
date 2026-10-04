#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H
#include <stdint.h>
#define BSP_ENCODER_COUNT 4U
// Single control-task owner after initialization; sample displacement must be <32768.
uint8_t BSP_Encoder_Init(void);
uint16_t BSP_Encoder_GetCount(uint8_t Channel);
int16_t BSP_Encoder_GetDelta(uint8_t Channel);
void BSP_Encoder_Sync(uint8_t Channel); // Capture baseline without resetting hardware.
#endif
