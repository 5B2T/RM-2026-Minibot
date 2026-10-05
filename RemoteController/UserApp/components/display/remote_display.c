#include "components/display/remote_display.h"
#include "control/remote_control.h"
#include "remote_config.h"
#include "display/oled.h"
#include "i2c/bsp_soft_i2c.h"
#include <stdio.h>
#include <math.h>
static uint8_t Started,Retry,FrameValid,Page;
static uint32_t StartTick,RetryTick,FrameTick;
static void Line(unsigned n,const char *t){OLED_DrawText(0,n*8,t);}
static void Render(Remote_Status s)
{
 char t[22];OLED_Clear();
 if(s.Page==0){
  Line(0,"1/3 REMOTE STATUS");
  Line(1,s.Mode==REMOTE_ACTIVE?"TX: ACTIVE":s.Mode==REMOTE_RESUMING?"TX: RESUME REQUEST":"TX: LOCKED");
  Line(2,s.LinkAck?"RADIO ACK: YES":"RADIO ACK: NO");
  Line(3,!s.InputValid?"INPUT: ERROR":s.Calibrated?"INPUT: READY":"INPUT: CENTER STICKS");
  snprintf(t,sizeof(t),"GEAR: %u LIMIT %u",s.Gear+1,(unsigned[]){200,350,500}[s.Gear]);Line(4,t);
  snprintf(t,sizeof(t),"VXY: %+ld %+ld",lroundf(s.Vx*1000),lroundf(s.Vy*1000));Line(5,t);
  snprintf(t,sizeof(t),"WZ: %+ld MRAD/S",lroundf(s.Wz*1000));Line(6,t);
  unsigned mv=(unsigned)(s.SupplyV*1000+.5f);snprintf(t,sizeof(t),"SUPPLY: %u.%03u V",mv/1000,mv%1000);Line(7,t);
 }else if(s.Page==1){
  Line(0,"2/3 INPUT CHECK");
  const char *names[]={"PA1","PA2","PA3","PA6","PA4"};
  for(unsigned i=0;i<5;i++){snprintf(t,sizeof(t),"%s: %u",names[i],s.Raw[i]);Line(i+1,t);}
  snprintf(t,sizeof(t),"KEYS: %04X",s.Keys);Line(6,t);Line(7,"ARM: NOT IMPLEMENTED");
 }else{
  Line(0,"3/3 ARM PREVIEW");Line(1,"LOCAL ONLY / NO TX");
  snprintf(t,sizeof(t),"FWD: %+ld",lroundf(s.Axis[REMOTE_RIGHT_Y_INDEX]*1000));Line(2,t);
  snprintf(t,sizeof(t),"RIGHT: %+ld",lroundf(s.Axis[REMOTE_RIGHT_X_INDEX]*1000));Line(3,t);
  snprintf(t,sizeof(t),"UP: %+ld",lroundf(s.ArmUp*1000));Line(4,t);
  snprintf(t,sizeof(t),"GRIP: %+ld",lroundf(s.Gripper*1000));Line(5,t);
  Line(6,"NO JOINT MODEL YET");Line(7,"LEFT PRESS: PAGE");
 }
}
void RemoteDisplay_Step(uint32_t NowMs)
{
 if(!Started){Started=1;StartTick=NowMs;return;}
 if((uint32_t)(NowMs-StartTick)<200)return;
 if(!OLED_IsReady()){
  if(Retry&&(uint32_t)(NowMs-RetryTick)<REMOTE_OLED_RETRY_MS)return;
  if(!OLED_Init(BSP_SoftI2C_Write,REMOTE_OLED_ADDRESS))goto Failure;
  Retry=FrameValid=0;
 }
 if(OLED_IsFlushing()){if(!OLED_FlushStep())goto Failure;return;}
 Remote_Status s=Controller_GetStatus();
 if(!FrameValid||Page!=s.Page||(uint32_t)(NowMs-FrameTick)>=REMOTE_OLED_REFRESH_MS){
  Render(s);if(!OLED_FlushBegin())goto Failure;Page=s.Page;FrameTick=NowMs;FrameValid=1;
 }
 return;
Failure:Retry=1;RetryTick=NowMs;
}
