#include "radio/bsp_nrf24l01.h"
#include "board_config.h"
#include "spi.h"
#include <stddef.h>
void BSP_NRF24L01_SetCE(uint8_t Enabled)
{
    HAL_GPIO_WritePin(BOARD_RADIO_CE_PORT, BOARD_RADIO_CE_PIN,
                      Enabled?GPIO_PIN_SET:GPIO_PIN_RESET);
}
void BSP_NRF24L01_Init(void)
{
    BSP_NRF24L01_SetCE(0);
    HAL_GPIO_WritePin(BOARD_RADIO_CSN_PORT, BOARD_RADIO_CSN_PIN, GPIO_PIN_SET);
}
uint8_t BSP_NRF24L01_Transfer(const uint8_t *Tx, uint8_t *Rx, uint8_t Length)
{
    if(Tx==NULL || Rx==NULL || Length==0 || Length>33)return 0;
    HAL_GPIO_WritePin(BOARD_RADIO_CSN_PORT, BOARD_RADIO_CSN_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef Result=HAL_SPI_TransmitReceive(&BOARD_RADIO_SPI, (uint8_t *)Tx,
                                                   Rx, Length, BOARD_RADIO_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(BOARD_RADIO_CSN_PORT, BOARD_RADIO_CSN_PIN, GPIO_PIN_SET);
    return Result==HAL_OK;
}
