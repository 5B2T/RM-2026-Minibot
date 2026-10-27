#include "components/display/debug_display.h"
#include "components/chassis/chassis_test.h"
#include "display/oled.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t Now;static ChassisTest_Status Test;static unsigned Toggles,Nexts;
uint8_t BSP_I2C_Init(void){return 1;}
uint8_t BSP_I2C_Write(uint8_t A,const uint8_t *D,uint16_t N){(void)A;(void)D;(void)N;return 1;}
uint8_t OLED_Init(OLED_WriteFn W,uint8_t A){(void)W;(void)A;return 1;}
uint8_t OLED_IsReady(void){return 1;}
void OLED_Clear(void){}
void OLED_DrawText(int16_t X,int16_t Y,const char *T){(void)X;(void)Y;assert(!strstr(T,"PE6"));}
uint8_t OLED_FlushBegin(void){return 1;}
uint8_t OLED_FlushStep(void){return 1;}
uint8_t OLED_IsFlushing(void){return 0;}
void ChassisTest_Next(void){Nexts++;Test.Preset=(Test.Preset+1)%CHASSIS_TEST_COUNT;Test.Running=0;}
void ChassisTest_Toggle(uint32_t T){(void)T;Toggles++;Test.Running=!Test.Running;}
void ChassisTest_Stop(void){Test.Running=0;}
void ChassisTest_Close(void){Test.Running=0;}
ChassisTest_Status ChassisTest_GetStatus(void){return Test;}
static void Poll(uint8_t Down,unsigned Ms){for(unsigned i=0;i<Ms;i+=10){Now+=10;DebugDisplay_HandleKeys(Down,0,Now);}}
static void Tap(void){Poll(1,60);Poll(0,60);}
static void Reset(uint8_t Held){Test=(ChassisTest_Status){0};Toggles=Nexts=0;DebugDisplay_Init();DebugDisplay_HandleKeys(Held,0,Now);}
int main(void){
 Reset(1);Poll(1,1200);Poll(0,50);assert(DebugDisplay_GetState().MainMenu&&DebugDisplay_GetState().Cursor==0);
 Poll(1,10);Poll(0,50);assert(DebugDisplay_GetState().Cursor==0); // Bounce.
 Tap();assert(DebugDisplay_GetState().Cursor==1);Poll(1,800);
 assert(!DebugDisplay_GetState().MainMenu&&DebugDisplay_GetState().Page==DEBUG_PAGE_STATUS);
 Poll(1,1000);assert(!DebugDisplay_GetState().MainMenu);Poll(0,60); // No repeat.
 Poll(1,800);assert(DebugDisplay_GetState().MainMenu);Poll(0,60);
 Reset(0);Poll(1,800);Poll(0,60);assert(DebugDisplay_GetState().Page==DEBUG_PAGE_CHASSIS_TEST&&!DebugDisplay_GetState().MainMenu);
 assert(!Toggles&&!Nexts);Tap();Poll(0,310);assert(Nexts==1&&!Toggles);
 Tap();Tap();assert(Toggles==1&&Test.Running&&Nexts==1);
 Poll(1,50);assert(!Test.Running);Poll(0,60);Poll(0,400);assert(Toggles==1&&Nexts==1);
 // Single click followed by a hold must return, never start.
 Tap();Poll(1,800);assert(DebugDisplay_GetState().MainMenu&&!Test.Running&&Toggles==1);Poll(0,60);
 Reset(0);Poll(1,800);Poll(0,60);Tap();Tap();assert(Test.Running);
 Poll(1,800);assert(DebugDisplay_GetState().MainMenu&&!Test.Running);Poll(0,60);assert(Toggles==1);
 Reset(0);DebugDisplay_HandleKeys(0,1,Now+100);assert(DebugDisplay_GetState().MainMenu&&DebugDisplay_GetState().Cursor==0); // Old PE6 ignored.
 Now=UINT32_MAX-100;Reset(0);Poll(1,800);assert(!DebugDisplay_GetState().MainMenu);Poll(0,60);Tap();Tap();assert(Test.Running);
 Robot_Status Status={0};
 for(unsigned i=0;i<DEBUG_PAGE_COUNT;i++){
  DebugDisplay_SetPage((DebugDisplay_Page)i);Now+=100;DebugDisplay_Step(&Status,Now);
 }
 puts("PASS: boot-held/bounce, short/hold/no repeat, select/double-start/immediate-stop/return, PE6 ignored, tick wrap, page hints");return 0;
}
