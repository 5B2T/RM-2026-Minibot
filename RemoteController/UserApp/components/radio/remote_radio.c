#include "components/radio/remote_radio.h"
#include "control/remote_control.h"
#include "remote_config.h"
#include "radio/nrf24l01.h"
#include "time/bsp_time.h"
// Chip 2 only: the pot-mode targets are appended to the V3 frame before encoding.
// Chip 1 does not build this file, so the dependency stays one directional.
#include "arm_targets.h"
static uint8_t Ready,Pending,Flags,Sequence,PendingTelemetry,TelemetryValid;
static Remote_Mode LastMode;
static uint32_t RetryTick,SendTick,TelemetryTick;
static uint8_t Telemetry[REMOTE_TELEMETRY_FRAME_SIZE];
void RemoteRadio_Init(uint32_t NowMs)
{
 Ready=Pending=PendingTelemetry=TelemetryValid=Sequence=0;LastMode=REMOTE_LOCKED;
 RetryTick=NowMs-REMOTE_RADIO_RETRY_MS;SendTick=NowMs-REMOTE_TX_PERIOD_MS;
 TelemetryTick=NowMs-REMOTE_TELEMETRY_PERIOD_MS;
}
void RemoteRadio_SetTelemetry(const RemoteTelemetry_FrameTypeDef *Frame)
{
 TelemetryValid=RemoteTelemetry_Encode(Frame,Telemetry);
}
static void Failure(uint32_t NowMs)
{
 Controller_OnTx(Flags,0,NowMs);Ready=Pending=PendingTelemetry=0;RetryTick=NowMs;
}
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
  NRF24L01_Result Result=NRF24L01_TxAbort();Pending=PendingTelemetry=0;
  SendTick=NowMs-REMOTE_TX_PERIOD_MS;
  if(Result!=NRF_OK){Failure(BSP_TimeMs());return;}
 }
 if(Pending){
  NRF24L01_Result Result=NRF24L01_TxPoll();
  if(Result==NRF_PENDING)return;
  Pending=0;
  if(Result!=NRF_OK){
   // A lost telemetry frame is diagnostic only: it must not lock the remote.
   if(PendingTelemetry){PendingTelemetry=0;return;}
   Failure(BSP_TimeMs());return;
  }
  if(!PendingTelemetry)Controller_OnTx(Flags,1,BSP_TimeMs());
  PendingTelemetry=0;
 }
 uint32_t Due=(uint32_t)(NowMs-SendTick)>=REMOTE_TX_PERIOD_MS;
 if(TelemetryValid&&!Due&&(uint32_t)(NowMs-TelemetryTick)>=REMOTE_TELEMETRY_PERIOD_MS){
  // Never ahead of a due command: STOP and motion frames keep their 20 ms cadence.
  TelemetryTick=NowMs;
  if(NRF24L01_TxBegin(Telemetry)==NRF_PENDING){Pending=1;PendingTelemetry=1;}
  return;
 }
 if(!Due)return;
 RemoteControl_FrameTypeDef Frame;Controller_Command(&Frame,Sequence++);
 ArmTargets_Apply(&Frame);
 uint8_t Data[REMOTE_CONTROL_FRAME_SIZE];
 if(!CommandProtocol_Encode(&Frame,Data)){Controller_Lock();return;}
 Flags=Frame.Flags;
 if(NRF24L01_TxBegin(Data)!=NRF_PENDING){Failure(BSP_TimeMs());return;}
 Pending=1;SendTick=BSP_TimeMs();
}
