#ifndef BSP_NRF24L01_H
#define BSP_NRF24L01_H
#include <stdint.h>
void BSP_NRF24L01_Init(void);
void BSP_NRF24L01_SetCE(uint8_t Enabled);
// One CSN transaction, one bounded SPI timeout. Return 1 on success, 0 on failure.
uint8_t BSP_NRF24L01_Transfer(const uint8_t *Tx, uint8_t *Rx, uint8_t Length);
#endif
