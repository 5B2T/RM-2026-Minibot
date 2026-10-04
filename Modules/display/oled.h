#ifndef OLED_H
#define OLED_H
#include <stdint.h>
#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U
typedef uint8_t (*OLED_WriteFn)(uint8_t Address, const uint8_t *Data, uint16_t Length);
// Single display-task owner. Uses SSD1306 128x64 page addressing and internal pump.
// Module power must be stable before Init; four-pin module resets on its own PCB.
uint8_t OLED_Init(OLED_WriteFn Write, uint8_t Address);
uint8_t OLED_IsReady(void);
void OLED_Clear(void); // RAM only; never triggers an implicit bus transfer.
void OLED_DrawPoint(int16_t X, int16_t Y, uint8_t On);
void OLED_DrawText(int16_t X, int16_t Y, const char *Text); // 6x8, clipped ASCII.
// Freeze drawing during a flush. One Step sends at most 32 pixels' bytes.
uint8_t OLED_FlushBegin(void);
uint8_t OLED_FlushStep(void); // 0: failed/offline, 1: progress or finished.
uint8_t OLED_IsFlushing(void);
#endif
