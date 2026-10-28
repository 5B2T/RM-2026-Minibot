#include "components/display/debug_display.h"
#include "robot/robot_config.h"
#include "display/oled.h"
#include "i2c/bsp_i2c.h"
#include "components/chassis/chassis_test.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

static DebugDisplay_State Menu={.Page=DEBUG_PAGE_CHASSIS_TEST,.MainMenu=1};
static uint8_t Started,RetryPending,FrameClockValid,KeysStarted;
static uint32_t RetryTick,FrameTick;
static uint32_t MenuRevision,SeenRevision;
typedef struct {uint8_t Raw,Stable;uint32_t ChangeTick;} Display_Key;
static Display_Key NextKey;
static const char *const PageNames[]={"CHASSIS TEST","STATUS","SPEED","PWM","POWER","POT ADC"};
static const char *const TestNames[]={"COMBO","M1+","M2+","M3+","M4+"};
static const unsigned MotorNumbers[]={1,2,3,4};
static const char *const ActionNames[]={"FWD","BACK","LEFT","RIGHT","CCW","CW"};
_Static_assert(sizeof(PageNames)/sizeof(PageNames[0])==DEBUG_PAGE_COUNT,"OLED page names mismatch");
_Static_assert(sizeof(TestNames)/sizeof(TestNames[0])==CHASSIS_TEST_COUNT,"Chassis preset names mismatch");
static uint8_t KeyDown,KeyConsumed,KeyAwaitRelease,TestClickPending,TestSecondClick;
static uint32_t KeyPressTick,TestClickTick;

void DebugDisplay_Init(void)
{
    Menu=(DebugDisplay_State){.Page=DEBUG_PAGE_CHASSIS_TEST,.MainMenu=1};
    Started=RetryPending=FrameClockValid=KeysStarted=0;
    MenuRevision=SeenRevision=0;
    KeyDown=KeyConsumed=KeyAwaitRelease=TestClickPending=TestSecondClick=0;KeyPressTick=TestClickTick=0;
    memset(&NextKey,0,sizeof(NextKey));

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
static void ResetGesture(void)
{
    KeyDown=KeyConsumed=TestClickPending=TestSecondClick=0;KeyAwaitRelease=1;
}
void DebugDisplay_HandleKeys(uint8_t NextPressed,uint8_t BackPressed,uint32_t NowMs)
{
    (void)BackPressed; // PE6 is J5 PWM, never a menu key.
    taskENTER_CRITICAL();
    if(!KeysStarted)
    {
        KeysStarted=1;
        NextKey=(Display_Key){.Raw=NextPressed!=0,.Stable=NextPressed!=0};
        taskEXIT_CRITICAL();return;
    }
    uint8_t Event=KeyUpdate(&NextKey,NextPressed,NowMs);
    if(KeyAwaitRelease)
    {
        if(!NextPressed && !NextKey.Stable)KeyAwaitRelease=0;
        taskEXIT_CRITICAL();return;
    }
    uint8_t TestPage=!Menu.MainMenu && Menu.Page==DEBUG_PAGE_CHASSIS_TEST;
    if(Event==1)
    {
        KeyDown=1;KeyConsumed=0;KeyPressTick=NowMs;TestSecondClick=0;
        if(TestPage)
        {
            if(ChassisTest_GetStatus().Running)
            {ChassisTest_Stop();KeyConsumed=1;TestClickPending=0;}
            else if(TestClickPending)
            {
                if((uint32_t)(NowMs-TestClickTick)<ROBOT_BUTTON_DOUBLE_MS)TestSecondClick=1;
                else {ChassisTest_Next();TestClickPending=0;MenuRevision++;}
            }
        }
    }
    if(KeyDown && NextKey.Stable && NextPressed &&
       (uint32_t)(NowMs-KeyPressTick)>=ROBOT_BUTTON_HOLD_MS)
    {
        // Hold only navigates. It can never start a chassis test on the way back.
        if(Menu.MainMenu){Menu.Page=(DebugDisplay_Page)Menu.Cursor;Menu.MainMenu=0;}
        else {ChassisTest_Close();Menu.MainMenu=1;}
        MenuRevision++;ResetGesture();
    }
    else if(Event==2 && KeyDown)
    {
        if(!KeyConsumed)
        {
            if(Menu.MainMenu){Menu.Cursor=(uint8_t)((Menu.Cursor+1)%DEBUG_PAGE_COUNT);MenuRevision++;}
            else if(TestPage)
            {
                if(TestSecondClick){ChassisTest_Toggle(NowMs);TestClickPending=0;MenuRevision++;}
                else {TestClickPending=1;TestClickTick=NowMs;}
            }
        }
        KeyDown=0;TestSecondClick=0;
    }
    if(TestClickPending && !NextPressed && !NextKey.Stable &&
       (uint32_t)(NowMs-TestClickTick)>=ROBOT_BUTTON_DOUBLE_MS)
    {ChassisTest_Next();TestClickPending=0;MenuRevision++;}
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
    ResetGesture();
    if(Page!=DEBUG_PAGE_CHASSIS_TEST)ChassisTest_Close();
    taskEXIT_CRITICAL();
}
void DebugDisplay_NextPage(void)
{
    taskENTER_CRITICAL();
    Menu.Page=(DebugDisplay_Page)((Menu.Page+1)%DEBUG_PAGE_COUNT);
    Menu.MainMenu=0;
    Menu.Cursor=(uint8_t)Menu.Page;
    MenuRevision++;
    ResetGesture();
    if(Menu.Page!=DEBUG_PAGE_CHASSIS_TEST)ChassisTest_Close();
    taskEXIT_CRITICAL();
}
void DebugDisplay_ShowMenu(void)
{
    taskENTER_CRITICAL();
    Menu.MainMenu=1;
    MenuRevision++;
    ChassisTest_Close();ResetGesture();
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
    if(!Online)ChassisTest_Stop();
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
            Line((uint8_t)(i+1),Text);
        }
        Line(7,"TAP:NEXT HOLD:ENTER");
        return;
    }
    (void)snprintf(Text,sizeof(Text),"%u/%u %s",(unsigned)View.Page+1,(unsigned)DEBUG_PAGE_COUNT,
        PageNames[View.Page]);
    Line(0,Text);
    if(View.Page==DEBUG_PAGE_CHASSIS_TEST)
    {
        const ChassisTest_Status *Test=&Status->ChassisTest;
        unsigned Preset=(unsigned)Test->Preset<CHASSIS_TEST_COUNT?(unsigned)Test->Preset:0;
        Line(0,"CHASSIS TEST  MM/S");
        if(Preset==CHASSIS_TEST_COMBINED && Test->Running && Test->ActiveAction<=CHASSIS_ACTION_RIGHT)
            (void)snprintf(Text,sizeof(Text),"SQ %u/%u %s RUN",(unsigned)Test->Lap,
                (unsigned)ROBOT_CHASSIS_TEST_SQUARE_LAPS,ActionNames[Test->ActiveAction]);
        else if(Preset==CHASSIS_TEST_COMBINED && Test->Running && Test->ActiveAction<=CHASSIS_ACTION_CW)
            (void)snprintf(Text,sizeof(Text),"TURN %s RUN",ActionNames[Test->ActiveAction]);
        else (void)snprintf(Text,sizeof(Text),"%s %s",TestNames[Preset],
            Test->Completed?"DONE":Test->Running?"RUN":"STOP");
        Line(1,Text);
        for(unsigned i=0;i<ROBOT_WHEEL_COUNT;i++)
        {
            (void)snprintf(Text,sizeof(Text),"M%u %+5ld / %+5ld",MotorNumbers[i],
                Scaled(Status->TargetSpeed[i],1000,9999),Scaled(Status->MeasuredSpeed[i],1000,9999));
            Line((uint8_t)(i+2),Text);
        }
        if(Test->Fault==CHASSIS_TEST_NO_ENCODER || Test->Fault==CHASSIS_TEST_REVERSED_ENCODER)
        {
            unsigned Motor=Test->FaultWheel>=1 && Test->FaultWheel<=4?MotorNumbers[Test->FaultWheel-1]:0;
            (void)snprintf(Text,sizeof(Text),"%s M%u",Test->Fault==CHASSIS_TEST_NO_ENCODER?"NO ENC":"ENC SIGN",Motor);
            Line(6,Text);
        }
        else if(Test->Fault==CHASSIS_TEST_OK && Preset==CHASSIS_TEST_COMBINED)
        {
            if(Test->Running && Test->ActiveAction>=CHASSIS_ACTION_CCW)
                (void)snprintf(Text,sizeof(Text),"TIMED 360DEG %ldMRAD",
                    Scaled(ROBOT_CHASSIS_TEST_TURN_RADPS,1000,9999));
            else (void)snprintf(Text,sizeof(Text),"%luMS %ldMM/S",
                (unsigned long)ROBOT_CHASSIS_TEST_SQUARE_SIDE_MS,
                Scaled(ROBOT_CHASSIS_TEST_SPEED_MPS,1000,9999));
            Line(6,Text);
        }
        else Line(6,Test->Fault==CHASSIS_TEST_DISPLAY_LOST?"DISPLAY LOST":
            Test->Fault==CHASSIS_TEST_REMOTE_STOP?"REMOTE STOP":
            Test->Fault==CHASSIS_TEST_BAD_SAMPLE?"SAMPLE ERROR":
            Test->Fault==CHASSIS_TEST_STALE_START?"START EXPIRED":"PA15:NEXT SET/ACT");
        Line(7,Test->Running?"TAP:STOP HOLD:BACK":"2TAP:RUN HOLD:BACK");
        return;
    }
    else if(View.Page==DEBUG_PAGE_STATUS)
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
    else if(View.Page==DEBUG_PAGE_POT)
    {
        // Read-only pot feedback from chip 2: raw ADC plus the provisional linear angle.
        const Robot_Telemetry *Telemetry=&Status->Telemetry;
        Line(0,"POT ADC DEG (UNCAL)");
        Line(1,!Telemetry->Online?"NO TELEMETRY":Telemetry->Valid?"CHIP2 OK":"POTS INVALID");
        for(unsigned i=0;i<ROBOT_TELEMETRY_CHANNELS;i++)
        {
            (void)snprintf(Text,sizeof(Text),"J%u %4u %3lu.%lud",i+1,(unsigned)Telemetry->Raw[i],
                (unsigned long)(Telemetry->AngleCdeg[i]/100U),
                (unsigned long)(Telemetry->AngleCdeg[i]%100U/10U));
            Line((uint8_t)(i+2),Text);
        }
        if(Status->Arm.Action==ARM_FOLLOW && Status->Arm.BasePositionEnabled)
            (void)snprintf(Text,sizeof(Text),"B%+04ld>%+04ld %s",
                Scaled(Status->Arm.BaseRelativeAngleDeg,1,999),Scaled(Status->Arm.BasePositionTargetDeg,1,999),
                !Status->Arm.BasePositionPickedUp?"WAIT":Status->Arm.BasePositionAtTarget?"HOLD":"RUN");
        else if(Telemetry->Online)
            (void)snprintf(Text,sizeof(Text),"AGE %luMS",(unsigned long)(NowMs-Telemetry->TickMs));
        else Text[0]='\0';
        Line(7,Text);
        return;
    }
    else
    {
        Line(1,View.Page==DEBUG_PAGE_SPEED?"   SET   ACT MM/S":"OUTPUT: +/-999");
        for(unsigned i=0;i<ROBOT_WHEEL_COUNT;i++)
        {
            if(View.Page==DEBUG_PAGE_SPEED)
                (void)snprintf(Text,sizeof(Text),"M%u %+5ld %+5ld",MotorNumbers[i],
                    Scaled(Status->TargetSpeed[i],1000,9999),Scaled(Status->MeasuredSpeed[i],1000,9999));
            else (void)snprintf(Text,sizeof(Text),"M%u %+4ld",MotorNumbers[i],Scaled(Status->PWM[i],1,999));
            Line((uint8_t)(i+2),Text);
        }
        if(View.Page==DEBUG_PAGE_PWM)
            (void)snprintf(Text,sizeof(Text),"LIMIT: %ld MM/S",Scaled(ROBOT_WHEEL_SPEED_LIMIT_MPS,1000,9999));
        else (void)snprintf(Text,sizeof(Text),"M1 M2 M3 M4");
        Line(6,Text);
    }
    Line(7,"PA15 HOLD: MENU");
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
    uint32_t RefreshMs=!View.MainMenu && View.Page==DEBUG_PAGE_CHASSIS_TEST?
        ROBOT_CHASSIS_TEST_REFRESH_MS:ROBOT_DISPLAY_REFRESH_MS;
    if(!FrameClockValid || Changed || (uint32_t)(NowMs-FrameTick)>=RefreshMs)
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
