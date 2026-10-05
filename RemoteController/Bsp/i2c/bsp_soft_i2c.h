#ifndef BSP_SOFT_I2C_H
#define BSP_SOFT_I2C_H
#include <stdint.h>
uint8_t BSP_SoftI2C_Write(uint8_t Address,const uint8_t *Data,uint16_t Length);
#endif
