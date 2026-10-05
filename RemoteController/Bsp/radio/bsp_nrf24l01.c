#include "radio/bsp_nrf24l01.h"
#include "main.h"
#include "spi.h"
#include <stddef.h>
void BSP_NRF24L01_SetCE(uint8_t Enabled){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,Enabled?GPIO_PIN_SET:GPIO_PIN_RESET);}
void BSP_NRF24L01_Init(void){BSP_NRF24L01_SetCE(0);HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);}
uint8_t BSP_NRF24L01_Transfer(const uint8_t *Tx,uint8_t *Rx,uint8_t Length)
{
 if(Tx==NULL||Rx==NULL||Length==0||Length>33)return 0;
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
 HAL_StatusTypeDef Result=HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)Tx,Rx,Length,2);
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);return Result==HAL_OK;
}
