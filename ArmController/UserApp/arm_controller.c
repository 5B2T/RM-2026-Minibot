#include "arm_controller.h"
#include "arm_board.h"
#include "control/remote_control.h"
#include "components/radio/remote_radio.h"
#include "radio/nrf24l01.h"
#include "command/telemetry_protocol.h"
#include "arm_targets.h"
#include <stdio.h>
static uint32_t SampleTick,PrintTick;
static uint16_t Pots[5];
static uint8_t PotsValid,TelemetrySequence;
static void PublishTelemetry(void)
{
 RemoteTelemetry_FrameTypeDef Frame={0};
 ArmTargets_Status Targets=ArmTargets_GetStatus();
 Frame.Header=0xa5U;Frame.Version=REMOTE_TELEMETRY_VERSION;
 Frame.Sequence=TelemetrySequence++;
 Frame.Flags=PotsValid?REMOTE_TELEMETRY_FLAG_POTS:0;
 if(Targets.Active)Frame.Flags|=REMOTE_TELEMETRY_FLAG_POT_MODE;
 for(uint8_t i=0;i<REMOTE_TELEMETRY_CHANNELS;i++)
 {
  // Out-of-range samples are never published as valid: the car must not read a stale
  // or broken conversion as a joint angle.
  if(!PotsValid||Pots[i]>REMOTE_TELEMETRY_ADC_MAX)continue;
  Frame.Raw[i]=Pots[i];
  Frame.AngleCdeg[i]=(uint16_t)(Targets.AngleCdeg[i]>0?Targets.AngleCdeg[i]:0);
 }
 RemoteRadio_SetTelemetry(&Frame);
}
void ArmController_Init(void)
{
 ArmBoard_Init();Controller_Init();RemoteRadio_Init(HAL_GetTick());ArmTargets_Init();
 SampleTick=PrintTick=HAL_GetTick();TelemetrySequence=0;
 ArmBoard_Print("\r\nCHIP2 I2C SLAVE PB6/PB7 0x32 | NRF ROBOT V3 + POT V4\r\nPOTS PA0..4 BASE,SHOULDER,ELBOW,WRISTP,WRISTR | R_UP+R_DOWN TOGGLES POT MODE\r\n");
}
void ArmController_Step(void)
{
 ArmBoard_Service();
 uint32_t Now=HAL_GetTick();
 Remote_Input Input=ArmBoard_Input(Now);
 Controller_Update(&Input,Now);
 RemoteRadio_Step(Now);
 if((uint32_t)(Now-SampleTick)>=20U)
 {
  SampleTick=Now;PotsValid=ArmBoard_Pots(Pots);
  ArmTargets_Update(Pots,PotsValid,Controller_GetStatus().Keys,Now);
  PublishTelemetry();
 }
 if((uint32_t)(Now-PrintTick)>=1000U)
 {
  PrintTick=Now;Remote_Status S=Controller_GetStatus();static char Text[240];
  (void)snprintf(Text,sizeof(Text),"I2C RX=%lu ERR=%lu VALID=%u KEYS=%04X ACK=%u MODE=%u J=%u LOCK=%u N=%lu POTS=%u,%u,%u,%u,%u ADC=%u POT=%u\r\n",
   (unsigned long)ArmBoard_RxCount,(unsigned long)ArmBoard_Errors,Input.Valid,Input.Keys,
   S.LinkAck,S.Mode,S.SelectedJoint+1U,(unsigned)S.LastLockReason,(unsigned long)S.LockCount,
   Pots[0],Pots[1],Pots[2],Pots[3],Pots[4],PotsValid,ArmTargets_GetStatus().Active);
  ArmBoard_Print(Text);
 }
}
