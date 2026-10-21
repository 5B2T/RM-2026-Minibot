#include "arm_controller.h"
#include "arm_board.h"
#include "control/remote_control.h"
#include "components/radio/remote_radio.h"
#include "radio/nrf24l01.h"
#include "command/input_link.h"
#include <stdio.h>
static uint32_t SampleTick,PrintTick;
static uint16_t Pots[5],Angles[5];
static uint8_t PotsValid;
void ArmController_Init(void)
{
 ArmBoard_Init();Controller_Init();RemoteRadio_Init(HAL_GetTick());
 SampleTick=PrintTick=HAL_GetTick();
 ArmBoard_Print("\r\nCHIP2 I2C SLAVE PB6/PB7 0x32 | POTS PA0..4 | NRF SPI2\r\nARM ANGLES UNCALIBRATED TELEMETRY ONLY\r\n");
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
  for(uint8_t t=0;t<5;t++)Angles[t]=(uint32_t)Pots[t]*18000U/4095U;
 }
 if((uint32_t)(Now-PrintTick)>=1000U)
 {
  PrintTick=Now;Remote_Status S=Controller_GetStatus();static char Text[200];
  (void)snprintf(Text,sizeof(Text),"I2C RX=%lu ERR=%lu VALID=%u KEYS=%04X ACK=%u MODE=%u POTS=%u,%u,%u,%u,%u ADC=%u\r\n",
   (unsigned long)ArmBoard_RxCount,(unsigned long)ArmBoard_Errors,Input.Valid,Input.Keys,
   S.LinkAck,S.Mode,Pots[0],Pots[1],Pots[2],Pots[3],Pots[4],PotsValid);
  ArmBoard_Print(Text);
 }
}
uint8_t ArmController_Telemetry(uint8_t Data[32],uint8_t Sequence)
{
 for(uint8_t t=0;t<32;t++)Data[t]=0;
 Data[0]=0xa5;Data[1]=3;Data[2]=Sequence;Data[3]=PotsValid;
 for(uint8_t t=0;t<5;t++)
 {InputLink_Put(Data+4+t*2,Angles[t]);InputLink_Put(Data+14+t*2,Pots[t]);}
 InputLink_Put(Data+30,InputLink_CRC(Data,30));
 return 1;
}
