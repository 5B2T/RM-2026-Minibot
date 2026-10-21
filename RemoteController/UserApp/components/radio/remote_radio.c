#include "components/radio/remote_radio.h"
#include "control/remote_control.h"
#include "remote_config.h"
#include "radio/nrf24l01.h"
#include "time/bsp_time.h"
static uint8_t Ready,Pending,Flags,Sequence;
static Remote_Mode LastMode;
static uint32_t RetryTick,SendTick;
void RemoteRadio_Init(uint32_t NowMs)
{Ready=Pending=Sequence=0;LastMode=REMOTE_LOCKED;RetryTick=NowMs-REMOTE_RADIO_RETRY_MS;SendTick=NowMs-REMOTE_TX_PERIOD_MS;}
static void Failure(uint32_t NowMs)
{Controller_OnTx(Flags,0,NowMs);Ready=Pending=0;RetryTick=NowMs;}
void RemoteRadio_Step(uint32_t NowMs)
{
 if(!Ready){
  if((uint32_t)(NowMs-RetryTick)<REMOTE_RADIO_RETRY_MS)return;
  RetryTick=NowMs;NRF24L01_SPI_Init();
  if(NRF24L01_TX_Mode()!=NRF_OK){Failure(BSP_TimeMs());return;}
  Ready=1;SendTick=BSP_TimeMs()-REMOTE_TX_PERIOD_MS;
  return; // Refresh inputs after initialization before assembling any command.
 }
 Remote_Status Status=Controller_GetStatus();
 if(Status.Mode==REMOTE_LOCKED&&LastMode!=REMOTE_LOCKED)SendTick=NowMs-REMOTE_TX_PERIOD_MS;
 LastMode=Status.Mode;
 if(Pending&&Status.Mode==REMOTE_LOCKED&&Flags!=REMOTE_CONTROL_FLAG_STOP){
  NRF24L01_Result Result=NRF24L01_TxAbort();Pending=0;SendTick=NowMs-REMOTE_TX_PERIOD_MS;
  if(Result!=NRF_OK){Failure(BSP_TimeMs());return;}
 }
 if(Pending){
  NRF24L01_Result Result=NRF24L01_TxPoll();
  if(Result==NRF_PENDING)return;
  Pending=0;
  if(Result!=NRF_OK){Failure(BSP_TimeMs());return;}
  Controller_OnTx(Flags,1,BSP_TimeMs());
 }
 if((uint32_t)(NowMs-SendTick)<REMOTE_TX_PERIOD_MS)return;
 RemoteControl_FrameTypeDef Frame;Controller_Command(&Frame,Sequence++);
 uint8_t Data[REMOTE_CONTROL_FRAME_SIZE];
 if(!CommandProtocol_Encode(&Frame,Data)){Controller_Lock();return;}
 Flags=Frame.Flags;
 if(NRF24L01_TxBegin(Data)!=NRF_PENDING){Failure(BSP_TimeMs());return;}
 Pending=1;SendTick=BSP_TimeMs();
}
