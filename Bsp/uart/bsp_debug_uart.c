#include "uart/bsp_debug_uart.h"
#include "usart.h"
#include <string.h>
#define RX_SIZE 512U
static uint8_t RxByte,RxBuffer[RX_SIZE],TxBuffer[BSP_DEBUG_UART_TX_SIZE];
static volatile uint16_t Head,Tail;
static volatile uint8_t TxBusy,Recover;
static volatile uint32_t Faults;
static uint8_t Initialized;

uint8_t BSP_DebugUart_Init(void)
{
    if(Initialized)return 1;
    Head=Tail=0;TxBusy=Recover=0;Faults=0;
    if(HAL_UART_Receive_IT(&huart1,&RxByte,1)!=HAL_OK)return 0;
    Initialized=1;
    return 1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *Uart)
{
    if(Uart!=&huart1)return;
    uint16_t Next=(uint16_t)((Head+1U)%RX_SIZE);
    if(Next==Tail)Faults++;
    else {RxBuffer[Head]=RxByte;__DMB();Head=Next;}
    if(HAL_UART_Receive_IT(Uart,&RxByte,1)!=HAL_OK) {Recover=1;Faults++;}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *Uart)
{
    if(Uart==&huart1)TxBusy=0;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *Uart)
{
    if(Uart==&huart1) {Recover=1;Faults++;}
}
void BSP_DebugUart_Service(void)
{
    if(!Initialized)return;
    if(Recover)
    {
        // Restart outside the ISR, discard the damaged receive stream.
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        (void)HAL_UART_AbortReceive(&huart1);
        Tail=Head;Recover=0;
        if(HAL_UART_Receive_IT(&huart1,&RxByte,1)!=HAL_OK)Recover=1;
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}
uint8_t BSP_DebugUart_Read(uint8_t *Byte)
{
    if(Byte==NULL || Tail==Head)return 0;
    *Byte=RxBuffer[Tail];__DMB();Tail=(uint16_t)((Tail+1U)%RX_SIZE);
    return 1;
}
void BSP_DebugUart_FlushRx(void)
{
    HAL_NVIC_DisableIRQ(USART1_IRQn);Tail=Head;HAL_NVIC_EnableIRQ(USART1_IRQn);
}
uint8_t BSP_DebugUart_Write(const char *Data,uint16_t Length)
{
    if(!Initialized || Data==NULL || Length==0 || Length>sizeof(TxBuffer) || TxBusy)return 0;
    memcpy(TxBuffer,Data,Length);TxBusy=1;
    if(HAL_UART_Transmit_IT(&huart1,TxBuffer,Length)!=HAL_OK) {TxBusy=0;return 0;}
    return 1;
}
uint32_t BSP_DebugUart_Faults(void) {return Faults;}
