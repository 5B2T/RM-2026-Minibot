#ifndef NRF24L01_H
#define NRF24L01_H
#include <stdint.h>
#define TX_ADR_WIDTH 5U
#define RX_ADR_WIDTH 5U
#define TX_PLOAD_WIDTH 32U
#define RX_PLOAD_WIDTH 32U

// Results are separate from chip STATUS bits; NRF_OK is zero for both RX and TX.
typedef enum
{
    NRF_OK=0,
    NRF_EMPTY,
    NRF_SPI_ERROR,
    NRF_CONFIG_ERROR,
    NRF_NOT_READY,
    NRF_TX_MAX_RETRY,
    NRF_TX_TIMEOUT,
    NRF_INVALID_ARGUMENT
} NRF24L01_Result;

#define NRF_RX_FIFO_DEPTH 3U
// Single owner: initialize before scheduling, then use only from the communication task.
// Mode changes discard old FIFOs. Check is read-only; errors require mode reinitialization.
void NRF24L01_SPI_Init(void);
NRF24L01_Result NRF24L01_RX_Mode(void);
NRF24L01_Result NRF24L01_TX_Mode(void);
NRF24L01_Result NRF24L01_Check(void);
NRF24L01_Result NRF24L01_TxPacket(const uint8_t *txbuf);
NRF24L01_Result NRF24L01_RxPacket(uint8_t *rxbuf);

#endif
