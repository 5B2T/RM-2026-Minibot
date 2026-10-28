// Exercise the real cooperative radio sender after exhausted on-air retries.
#include "components/radio/remote_radio.h"
#include "control/remote_control.h"
#include "radio/nrf24l01.h"
#include "remote_config.h"
#include <assert.h>
#include <stdio.h>
static uint32_t Now;
static unsigned Inits,Starts,Bad,Good,Aborts;
static int16_t Wanted;
static uint8_t StopFlags=REMOTE_CONTROL_FLAG_STOP;
static Remote_Status Status;
static NRF24L01_Result PollResult=NRF_OK;
static RemoteControl_FrameTypeDef Last;
uint32_t BSP_TimeMs(void){return Now;}
void NRF24L01_SPI_Init(void){Inits++;}
NRF24L01_Result NRF24L01_TX_Mode(void){return NRF_OK;}
NRF24L01_Result NRF24L01_TxAbort(void){Aborts++;return NRF_OK;}
NRF24L01_Result NRF24L01_TxPoll(void){return PollResult;}
NRF24L01_Result NRF24L01_TxBegin(const uint8_t *P)
{assert(CommandProtocol_Decode(P,32,&Last));Starts++;return NRF_PENDING;}
Remote_Status Controller_GetStatus(void){return Status;}
void Controller_Lock(void){Status.Mode=REMOTE_LOCKED;}
void Controller_OnTx(uint8_t F,uint8_t Ack,uint32_t T)
{(void)F;(void)T;if(Ack)Good++;else Bad++;}
void Controller_Command(RemoteControl_FrameTypeDef *F,uint8_t S)
{*F=(RemoteControl_FrameTypeDef){.Header=0xa5,.Version=3,.Sequence=S,.M4=Wanted};
 if(Status.Mode==REMOTE_LOCKED){F->Flags=StopFlags;F->M4=0;}}
void ArmTargets_Apply(RemoteControl_FrameTypeDef *F){(void)F;}
static void Tick(uint32_t T){Now=T;RemoteRadio_Step(Now);}
int main(void)
{
 Status.Mode=REMOTE_ACTIVE;RemoteRadio_Init(0);Tick(0);assert(Inits==1);
 Tick(1);assert(Starts==1);uint8_t Seq=Last.Sequence;
 PollResult=NRF_TX_MAX_RETRY;Tick(2);assert(Bad==1 && Inits==1);
 Wanted=500;PollResult=NRF_OK;Tick(21);assert(Starts==2 && Last.M4==500 && Last.Sequence!=Seq);
 Tick(22);assert(Good==1 && Inits==1); // No500ms reinit after a normal MAX_RT.
 Tick(41);Status.Mode=REMOTE_LOCKED;StopFlags=REMOTE_CONTROL_FLAG_STOP|REMOTE_CONTROL_FLAG_PASSIVE_STOP;Tick(42);
 assert(Aborts==1 && Last.Flags==StopFlags); // STOP replaces a pending motion.
 Tick(43);assert(Good==2 && Aborts==1); // Passive STOP must not abort itself every poll.
 Tick(62);assert(Last.Flags==9);StopFlags=REMOTE_CONTROL_FLAG_STOP;Tick(63);
 assert(Aborts==2 && Last.Flags==REMOTE_CONTROL_FLAG_STOP); // Operator STOP preempts a pending passive STOP.
 Tick(64);assert(Good==3);
 Status.Mode=REMOTE_ACTIVE;Tick(83);PollResult=NRF_SPI_ERROR;Tick(84);
 assert(Bad==2);Tick(85);assert(Inits==1);
 Tick(84+REMOTE_RADIO_RETRY_MS);assert(Inits==2); // Real hardware failures still reinitialize.
 puts("PASS: MAX_RT retries latest frame without reinit, pending motion aborts for STOP, SPI faults reinitialize");
 return 0;
}
