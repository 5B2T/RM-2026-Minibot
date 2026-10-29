#include "arm_controller.h"
#include "arm_board.h"
#include "control/remote_control.h"
#include "components/radio/remote_radio.h"
#include "radio/nrf24l01.h"
#include "command/telemetry_protocol.h"
#include "arm_targets.h"
#include <stdio.h>
static uint32_t SampleTick,PrintTick,PotTick;
static uint16_t Pots[5];
static uint8_t PotsValid,TelemetrySequence;
// Commissioning aid: the raw counts and the calibrated angle of every channel on
// one line, twenty times a second, so a potentiometer can be swept by hand while it
// is being watched. Angles are formed as whole.fraction to keep printf float-free.
static void FormatCdeg(char *Out,size_t Size,int32_t Cdeg)
{
 uint32_t Magnitude=Cdeg<0?0U-(uint32_t)Cdeg:(uint32_t)Cdeg;
 (void)snprintf(Out,Size,"%s%lu.%02lu",Cdeg<0?"-":"",
  (unsigned long)(Magnitude/100U),(unsigned long)(Magnitude%100U));
}
static void PublishTelemetry(void)
{
 RemoteTelemetry_FrameTypeDef Frame={0};
 ArmTargets_Status Targets=ArmTargets_GetStatus();
 Frame.Header=0xa5U;Frame.Version=REMOTE_TELEMETRY_VERSION;
 Frame.Sequence=TelemetrySequence++;
 Frame.Flags=(PotsValid && Targets.ValidMask==0x1f)?REMOTE_TELEMETRY_FLAG_POTS:0;
 if(Targets.Active && (Frame.Flags&REMOTE_TELEMETRY_FLAG_POTS))Frame.Flags|=REMOTE_TELEMETRY_FLAG_POT_MODE;
 for(uint8_t i=0;i<REMOTE_TELEMETRY_CHANNELS;i++)
 {
  // Out-of-range samples are never published as valid: the car must not read a stale
  // or broken conversion as a joint angle.
  if(!PotsValid||Pots[i]>REMOTE_TELEMETRY_ADC_MAX)continue;
  Frame.Raw[i]=Pots[i];
  Frame.AngleCdeg[i]=ArmTargets_TelemetryAngle(i);
 }
 RemoteRadio_SetTelemetry(&Frame);
}
void ArmController_Init(void)
{
 ArmBoard_Init();Controller_Init();RemoteRadio_Init(HAL_GetTick());ArmTargets_Init();
 SampleTick=PrintTick=PotTick=HAL_GetTick();TelemetrySequence=0;
 ArmBoard_Print("\r\nCHIP2 I2C SLAVE PB6/PB7 0x32 | NRF ROBOT V3 + POT V4\r\nPOTS PA0..4 BASE,SHOULDER,ELBOW,WRISTP,WRISTR | R_PRESS PC15 TOGGLES POT MODE\r\nPOTRAW/ANG 20HZ, ANG IN DEG FROM THE CALIBRATION TABLE\r\n");
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
 if((uint32_t)(Now-PotTick)>=50U)
 {
  PotTick=Now;ArmTargets_Status T=ArmTargets_GetStatus();
  static char Line[192],Ang[5][16];
  for(uint8_t i=0;i<ARM_TARGET_CHANNELS;i++)FormatCdeg(Ang[i],sizeof Ang[i],T.AngleCdeg[i]);
  (void)snprintf(Line,sizeof(Line),
   "POTRAW %u,%u,%u,%u,%u | ANG %s,%s,%s,%s,%s | VALID %u POT %u MASK %02X\r\n",
   Pots[0],Pots[1],Pots[2],Pots[3],Pots[4],
   Ang[0],Ang[1],Ang[2],Ang[3],Ang[4],PotsValid,T.Active,(unsigned)T.ValidMask);
  ArmBoard_Print(Line);
 }
 if((uint32_t)(Now-PrintTick)>=1000U)
 {
  PrintTick=Now;Remote_Status S=Controller_GetStatus();ArmTargets_Status T=ArmTargets_GetStatus();static char Text[256];
  (void)snprintf(Text,sizeof(Text),"I2C RX=%lu ERR=%lu VALID=%u KEYS=%04X ACK=%u MODE=%u J=%u LOCK=%u N=%lu POTS=%u,%u,%u,%u,%u ADC=%u POT=%u STABLE=%u WAIT=%u BLOCK=%u REC=%u\r\n",
   (unsigned long)ArmBoard_RxCount,(unsigned long)ArmBoard_Errors,Input.Valid,Input.Keys,
   S.LinkAck,S.Mode,S.SelectedJoint+1U,(unsigned)S.LastLockReason,(unsigned long)S.LockCount,
   Pots[0],Pots[1],Pots[2],Pots[3],Pots[4],PotsValid,T.Active,T.Stable,T.EntryPending,(unsigned)T.EntryBlock,S.Recovering);
  ArmBoard_Print(Text);
 }
}
