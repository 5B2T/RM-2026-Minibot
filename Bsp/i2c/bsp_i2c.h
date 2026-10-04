#ifndef BSP_I2C_H
#define BSP_I2C_H
#include <stdint.h>
// Initialize once from Robot_Init before tasks start. No GPIO or clock changes.
uint8_t BSP_I2C_Init(void);
// Task context only. Address is seven-bit; return 1 only for an acknowledged write.
// Short transactions and a priority-inheriting mutex allow future bus sharing.
uint8_t BSP_I2C_Write(uint8_t Address, const uint8_t *Data, uint16_t Length);
#endif
