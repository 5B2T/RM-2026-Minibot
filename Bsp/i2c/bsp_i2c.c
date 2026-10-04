#include "i2c/bsp_i2c.h"
#include "board_config.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stddef.h>

static StaticSemaphore_t BusMutexStorage;
static SemaphoreHandle_t BusMutex;

uint8_t BSP_I2C_Init(void)
{
    if(BusMutex==NULL)BusMutex=xSemaphoreCreateMutexStatic(&BusMutexStorage);
    return BusMutex!=NULL;
}

uint8_t BSP_I2C_Write(uint8_t Address, const uint8_t *Data, uint16_t Length)
{
    if(Address>0x7fU || Data==NULL || Length==0 ||
       Length>BOARD_I2C_MAX_WRITE_SIZE || BusMutex==NULL)return 0;
    if(xSemaphoreTake(BusMutex,pdMS_TO_TICKS(BOARD_I2C_LOCK_TIMEOUT_MS))!=pdTRUE)return 0;
    HAL_StatusTypeDef Result=HAL_I2C_Master_Transmit(&BOARD_DEVICE_I2C,
        (uint16_t)Address<<1,(uint8_t *)Data,Length,BOARD_I2C_WRITE_TIMEOUT_MS);
    (void)xSemaphoreGive(BusMutex);
    return Result==HAL_OK;
}
uint8_t BSP_I2C_ReadRegister(uint8_t Address, uint8_t Register, uint8_t *Data, uint16_t Length)
{
    if(Address>0x7fU || Data==NULL || Length==0 ||
       Length>BOARD_I2C_MAX_WRITE_SIZE || BusMutex==NULL)return 0;
    if(xSemaphoreTake(BusMutex,pdMS_TO_TICKS(BOARD_I2C_LOCK_TIMEOUT_MS))!=pdTRUE)return 0;
    HAL_StatusTypeDef Result=HAL_I2C_Mem_Read(&BOARD_DEVICE_I2C,(uint16_t)Address<<1,
        Register,I2C_MEMADD_SIZE_8BIT,Data,Length,BOARD_I2C_WRITE_TIMEOUT_MS);
    (void)xSemaphoreGive(BusMutex);
    return Result==HAL_OK;
}
