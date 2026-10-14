#ifndef BSP_DEBUG_UART_H
#define BSP_DEBUG_UART_H
#include <stdint.h>
// Single service-task consumer/producer; ISR only moves bytes and flags.
#define BSP_DEBUG_UART_TX_SIZE 512U
uint8_t BSP_DebugUart_Init(void);
void BSP_DebugUart_Service(void);
uint8_t BSP_DebugUart_Read(uint8_t *Byte);
uint8_t BSP_DebugUart_Write(const char *Data,uint16_t Length);
uint32_t BSP_DebugUart_Faults(void);
void BSP_DebugUart_FlushRx(void);
#endif
