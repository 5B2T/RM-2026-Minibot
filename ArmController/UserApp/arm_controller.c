#include "arm_controller.h"
#include "arm_board.h"
#include "control/remote_control.h"
#include "components/radio/remote_radio.h"
#include "radio/nrf24l01.h"
#include <stdio.h>
static uint32_t SampleTick,PrintTick;
static uint16_t Pots[5];
static uint8_t PotsValid;
void ArmController_Init(void)
{
 ArmBoard_Init();Controller_Init();RemoteRadio_Init(HAL_GetTick());
 SampleTick=PrintTick=HAL_GetTick();
 ArmBoard_Print("\r\nCHIP2 I2C SLAVE PB6/PB7 0x32 | NRF ROBOT V3\r\nPOTS PA0..4 RAW SERIAL ONLY, NO FOLLOW CONTROL\r\n");
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
 }
 if((uint32_t)(Now-PrintTick)>=1000U)
 {
  PrintTick=Now;Remote_Status S=Controller_GetStatus();static char Text[200];
  (void)snprintf(Text,sizeof(Text),"I2C RX=%lu ERR=%lu VALID=%u KEYS=%04X ACK=%u MODE=%u J=%u LOCK=%u N=%lu POTS=%u,%u,%u,%u,%u ADC=%u\r\n",
   (unsigned long)ArmBoard_RxCount,(unsigned long)ArmBoard_Errors,Input.Valid,Input.Keys,
   S.LinkAck,S.Mode,S.SelectedJoint+1U,(unsigned)S.LastLockReason,(unsigned long)S.LockCount,
   Pots[0],Pots[1],Pots[2],Pots[3],Pots[4],PotsValid);
  ArmBoard_Print(Text);
 }
}
