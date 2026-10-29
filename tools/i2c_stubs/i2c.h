#ifndef I2C_TEST_HAL_H
#define I2C_TEST_HAL_H
#include <stdint.h>
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET,GPIO_PIN_SET} GPIO_PinState;
typedef struct {uint32_t ErrorCode;} I2C_HandleTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {uint32_t Pins;} GPIO_TypeDef;
extern I2C_HandleTypeDef hi2c1;
extern GPIO_TypeDef TestPort;
extern uint32_t SystemCoreClock;
#define HAL_I2C_ERROR_BERR 1U
#define HAL_I2C_ERROR_ARLO 2U
#define HAL_I2C_ERROR_AF 4U
#define HAL_I2C_ERROR_OVR 8U
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_MODE_AF_OD 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define I2C_MEMADD_SIZE_8BIT 1U
#define __NOP() ((void)0)
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
void HAL_GPIO_Init(GPIO_TypeDef *,GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *,uint16_t);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *,uint16_t,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
#endif
