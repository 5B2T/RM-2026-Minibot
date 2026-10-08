#include "components/display/debug_display.h"
#include "robot/robot_config.h"
#include "display/oled.h"
#include "i2c/bsp_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

static DebugDisplay_State Menu={.Page=DEBUG_PAGE_STATUS,.MainMenu=1};
static uint8_t Started,RetryPending,FrameClockValid,KeysStarted;
static uint32_t RetryTick,FrameTick;
static uint32_t MenuRevision,SeenRevision;
typedef struct {uint8_t Raw,Stable;uint32_t ChangeTick;} Display_Key;
static Display_Key NextKey,BackKey;
static const char *const PageNames[]={"STATUS","SPEED","PWM","POWER"};

void DebugDisplay_Init(void)
{
    Menu=(DebugDisplay_State){.Page=DEBUG_PAGE_STATUS,.MainMenu=1};
    Started=RetryPending=FrameClockValid=KeysStarted=0;
    MenuRevision=SeenRevision=0;
    memset(&NextKey,0,sizeof(NextKey));
    memset(&BackKey,0,sizeof(BackKey));
}
// 1: debounced press, 2: debounced release. Held-at-boot keys require release first.
static uint8_t KeyUpdate(Display_Key *Key,uint8_t Raw,uint32_t NowMs)
{
    Raw=Raw!=0;
    if(Raw!=Key->Raw) {Key->Raw=Raw;Key->ChangeTick=NowMs;}
    if(Key->Stable==Raw || (uint32_t)(NowMs-Key->ChangeTick)<ROBOT_BUTTON_DEBOUNCE_MS)return 0;
    Key->Stable=Raw;
    return Raw?1:2;
}
void DebugDisplay_HandleKeys(uint8_t NextPressed,uint8_t BackPressed,uint32_t NowMs)
{
    taskENTER_CRITICAL();
    if(!KeysStarted)
    {
        KeysStarted=1;
        NextKey=(Display_Key){.Raw=NextPressed!=0,.Stable=NextPressed!=0};
        BackKey=(Display_Key){.Raw=BackPressed!=0,.Stable=BackPressed!=0};
        taskEXIT_CRITICAL();
        return;
    }
    uint8_t NextEvent=KeyUpdate(&NextKey,NextPressed,NowMs);
    uint8_t BackEvent=KeyUpdate(&BackKey,BackPressed,NowMs);
    if(BackEvent==1)
    {
        if(Menu.MainMenu)Menu.Page=(DebugDisplay_Page)Menu.Cursor;
        Menu.MainMenu=!Menu.MainMenu;
        MenuRevision++;
    }
    else if(Menu.MainMenu && NextEvent==1 && !BackPressed && !BackKey.Stable)
    {
        Menu.Cursor=(uint8_t)((Menu.Cursor+1)%DEBUG_PAGE_COUNT);
        MenuRevision++;
    }
    taskEXIT_CRITICAL();
}

void DebugDisplay_SetPage(DebugDisplay_Page Page)
{
    if((unsigned)Page>=DEBUG_PAGE_COUNT)return;
    taskENTER_CRITICAL();
    Menu.Page=Page;
    Menu.MainMenu=0;
    Menu.Cursor=(uint8_t)Page;
    MenuRevision++;
    taskEXIT_CRITICAL();
}
void DebugDisplay_NextPage(void)
{
    taskENTER_CRITICAL();
    Menu.Page=(DebugDisplay_Page)((Menu.Page+1)%DEBUG_PAGE_COUNT);
    Menu.MainMenu=0;
    Menu.Cursor=(uint8_t)Menu.Page;
    MenuRevision++;
    taskEXIT_CRITICAL();
}
void DebugDisplay_ShowMenu(void)
{
    taskENTER_CRITICAL();
    Menu.MainMenu=1;
    MenuRevision++;
    taskEXIT_CRITICAL();
}
DebugDisplay_State DebugDisplay_GetState(void)
{
    taskENTER_CRITICAL();
    DebugDisplay_State Copy=Menu;
    taskEXIT_CRITICAL();
    return Copy;
}
static void SetOnline(uint8_t Online)
{
    taskENTER_CRITICAL();
    Menu.Online=Online;
    taskEXIT_CRITICAL();
}
static long Scaled(float Value, float Scale, float Limit)
{
    if(!isfinite(Value))return 0;
    float Number=Value*Scale;
    if(Number>=Limit)return (long)Limit;
    if(Number<=-Limit)return -(long)Limit;
    return (long)(Number+(Number>=0?0.5f:-0.5f));
}
static void Line(uint8_t Row, const char *Text) {OLED_DrawText(0,(int16_t)(Row*8),Text);}
static void PowerValue(uint8_t Row,const char *Label,float Value,const char *Unit)
{
    char Text[22];
    long Milli=Scaled(Value,1000,999999);
    unsigned long Magnitude=(unsigned long)(Milli<0?-Milli:Milli);
    (void)snprintf(Text,sizeof(Text),"%s %s%lu.%03lu %s",Label,Milli<0?"-":"",
        Magnitude/1000,Magnitude%1000,Unit);
    Line(Row,Text);
}
static void Render(const Robot_Status *Status, uint32_t NowMs, DebugDisplay_State View)
{
    char Text[22];
    OLED_Clear();
    if(View.MainMenu)
    {
        Line(0,"MAIN MENU");
        for(unsigned i=0;i<DEBUG_PAGE_COUNT;i++)
        {
            (void)snprintf(Text,sizeof(Text),"%s %s",View.Cursor==i?">":" ",PageNames[i]);
            Line((uint8_t)(i+2),Text);
        }
        Line(6,"PA15: NEXT");
        Line(7,"PE6: ENTER");
        return;
    }
    (void)snprintf(Text,sizeof(Text),"%u/%u %s",(unsigned)View.Page+1,(unsigned)DEBUG_PAGE_COUNT,
        PageNames[View.Page]);
    Line(0,Text);
    if(View.Page==DEBUG_PAGE_STATUS)
    {
        Line(1,Status->Connected?"LINK: ONLINE":"LINK: OFFLINE");
        Line(2,Status->Armed?"ARMED: YES":"ARMED: NO");
        Line(3,Status->MotorsEnabled?"MOTOR: ENABLED":"MOTOR: DISABLED");
        Line(4,Status->CommandMode==ROBOT_COMMAND_VELOCITY?"MODE: BODY V2":"MODE: WHEEL V1");
        (void)snprintf(Text,sizeof(Text),"UP: %lu S",(unsigned long)(NowMs/1000));
        Line(5,Text);
        (void)snprintf(Text,sizeof(Text),"AGE: %lu MS",(unsigned long)(NowMs-Status->TickMs));
        Line(6,Text);
    }
    else if(View.Page==DEBUG_PAGE_POWER)
    {
        const Robot_PowerStatus *Power=&Status->Power;
        const char *State=Power->State==ROBOT_POWER_VALID?"INA219: OK":
            Power->State==ROBOT_POWER_WAITING?"INA219: WAIT":
            Power->State==ROBOT_POWER_OVERFLOW?"INA219: OVER RANGE":
            Power->State==ROBOT_POWER_TIMEOUT?"INA219: TIMEOUT":
            Power->State==ROBOT_POWER_STALE?"INA219: STALE":
            Power->State==ROBOT_POWER_CONFIG_ERROR?"INA219: CONFIG ERR":
            Power->State==ROBOT_POWER_INVALID?"INA219: BAD SAMPLE":"INA219: OFFLINE";
        Line(1,State);
        if(Power->Valid)
        {
            PowerValue(2,"V:",Power->VoltageV,"V");
            PowerValue(3,"I:",Power->CurrentA,"A");
            PowerValue(4,"P:",Power->PowerW,"W");
            (void)snprintf(Text,sizeof(Text),"AGE: %lu MS",(unsigned long)(NowMs-Power->TickMs));
            Line(5,Text);
        }
        else {Line(2,"V: -- V");Line(3,"I: -- A");Line(4,"P: -- W");}
        if(Power->Valid && Power->SocValid && isfinite(Power->SocPercent) &&
            Power->SocPercent>=0 && Power->SocPercent<=100)
        {
            (void)snprintf(Text,sizeof(Text),"SOC: ~%ld%%",Scaled(Power->SocPercent,1,100));
            Line(6,Text);
        }
        else Line(6,"SOC: --");
    }
    else
    {
        Line(1,View.Page==DEBUG_PAGE_SPEED?"   SET   ACT MM/S":"OUTPUT: +/-999");
        for(unsigned i=0;i<ROBOT_WHEEL_COUNT;i++)
        {
            if(View.Page==DEBUG_PAGE_SPEED)
                (void)snprintf(Text,sizeof(Text),"M%u %+5ld %+5ld",i+1,
                    Scaled(Status->TargetSpeed[i],1000,9999),Scaled(Status->MeasuredSpeed[i],1000,9999));
            else (void)snprintf(Text,sizeof(Text),"M%u %+4ld",i+1,Scaled(Status->PWM[i],1,999));
            Line((uint8_t)(i+2),Text);
        }
        if(View.Page==DEBUG_PAGE_PWM)
            (void)snprintf(Text,sizeof(Text),"LIMIT: %ld MM/S",Scaled(ROBOT_WHEEL_SPEED_LIMIT_MPS,1000,9999));
        else (void)snprintf(Text,sizeof(Text),"PHYSICAL M1..M4");
        Line(6,Text);
    }
    Line(7,"PE6: BACK TO MENU");
}
uint32_t DebugDisplay_Step(const Robot_Status *Status, uint32_t NowMs)
{
    if(Status==NULL)return ROBOT_DISPLAY_POLL_MS;
    if(!Started)
    {
        Started=1;
        return ROBOT_DISPLAY_POWER_WAIT_MS; // PCB RC reset / charge-pump power stabilization.
    }
    if(!OLED_IsReady())
    {
        if(RetryPending && (uint32_t)(NowMs-RetryTick)<ROBOT_DISPLAY_RETRY_MS)return ROBOT_DISPLAY_POLL_MS;
        SetOnline(0);
        if(!OLED_Init(BSP_I2C_Write,ROBOT_OLED_ADDRESS))goto Failure;
        RetryPending=0;
        FrameClockValid=0;
    }
    if(OLED_IsFlushing())
    {
        if(!OLED_FlushStep())goto Failure;
        if(!OLED_IsFlushing())SetOnline(1);
        return ROBOT_DISPLAY_POLL_MS;
    }
    taskENTER_CRITICAL();
    uint8_t Changed=SeenRevision!=MenuRevision;
    SeenRevision=MenuRevision;
    DebugDisplay_State View=Menu;
    taskEXIT_CRITICAL();
    if(!FrameClockValid || Changed || (uint32_t)(NowMs-FrameTick)>=ROBOT_DISPLAY_REFRESH_MS)
    {
        Render(Status,NowMs,View);
        if(!OLED_FlushBegin())goto Failure;
        FrameTick=NowMs;
        FrameClockValid=1;
    }
    return ROBOT_DISPLAY_POLL_MS;
Failure:
    SetOnline(0);
    RetryPending=1;
    RetryTick=NowMs;
    return ROBOT_DISPLAY_RETRY_MS;
}
