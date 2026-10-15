#include "components/chassis/chassis_test.h"
#include "robot/robot_config.h"
#include "wheel/speed.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>
static struct {
    uint32_t Heartbeat,Started;
    ChassisTest_Preset Preset;
    uint8_t Visible,Online,Start,Stop;
} Inbox;
static ChassisTest_Status State,Published;
static uint8_t Owned,Released,WasConnected;
static uint32_t Started,LastStopGeneration,LastMovement[ROBOT_WHEEL_COUNT];

void ChassisTest_Init(void)
{
    memset(&Inbox,0,sizeof(Inbox));memset(&State,0,sizeof(State));Published=State;
    Owned=Released=WasConnected=0;Started=LastStopGeneration=0;
    memset(LastMovement,0,sizeof(LastMovement));
}
void ChassisTest_View(uint8_t Visible,uint8_t Online,uint32_t NowMs)
{
    taskENTER_CRITICAL();
    if(Visible && !Inbox.Visible) {Inbox.Stop=1;Inbox.Start=0;}
    Inbox.Visible=Visible!=0;Inbox.Online=Online!=0;Inbox.Heartbeat=NowMs;
    if(!Visible || !Online) {Inbox.Stop=1;Inbox.Start=0;}
    taskEXIT_CRITICAL();
}
void ChassisTest_Stop(void)
{
    taskENTER_CRITICAL();Inbox.Stop=1;Inbox.Start=0;taskEXIT_CRITICAL();
}
void ChassisTest_Close(void)
{
    taskENTER_CRITICAL();Inbox.Visible=0;Inbox.Stop=1;Inbox.Start=0;taskEXIT_CRITICAL();
}
void ChassisTest_Next(void)
{
    taskENTER_CRITICAL();
    Inbox.Preset=(ChassisTest_Preset)((Inbox.Preset+1U)%CHASSIS_TEST_COUNT);
    Inbox.Stop=1;Inbox.Start=0;
    taskEXIT_CRITICAL();
}
void ChassisTest_Toggle(uint32_t NowMs)
{
    taskENTER_CRITICAL();
    if(Published.Running || Inbox.Start) {Inbox.Stop=1;Inbox.Start=0;}
    else if(Inbox.Visible && Inbox.Online && !Inbox.Stop)
    {Inbox.Start=1;Inbox.Started=NowMs;}
    taskEXIT_CRITICAL();
}
ChassisTest_Status ChassisTest_GetStatus(void)
{
    taskENTER_CRITICAL();ChassisTest_Status Copy=Published;taskEXIT_CRITICAL();return Copy;
}
static void Halt(ChassisTest_Fault Fault,uint8_t Wheel)
{
    State.Running=0;State.RemainingMs=0;State.Fault=Fault;State.FaultWheel=Wheel;
}
static void Publish(void)
{
    taskENTER_CRITICAL();Published=State;taskEXIT_CRITICAL();
}
uint8_t ChassisTest_TakeRelease(void)
{
    uint8_t Copy=Released;Released=0;return Copy;
}
uint8_t ChassisTest_Control(Robot_ChassisCommand *Command,const Robot_Status *Feedback,
    uint8_t SampleValid,uint32_t Now,uint32_t RemoteStopGeneration,uint8_t RemoteConnected)
{
    uint8_t Visible,Online,Start,Stop;uint32_t Heartbeat,RequestTick;ChassisTest_Preset Preset;
    taskENTER_CRITICAL();
    Visible=Inbox.Visible;Online=Inbox.Online;Start=Inbox.Start;Stop=Inbox.Stop;
    Preset=Inbox.Preset;Heartbeat=Inbox.Heartbeat;RequestTick=Inbox.Started;
    Inbox.Start=Inbox.Stop=0;
    taskEXIT_CRITICAL();
    memset(Command,0,sizeof(*Command));Command->Mode=ROBOT_COMMAND_VELOCITY;
    uint8_t Claim=Owned || Visible;
    uint8_t Entering=Visible && !Owned;
    if(Entering)
    {
        Halt(CHASSIS_TEST_OK,0);LastStopGeneration=RemoteStopGeneration;
        WasConnected=RemoteConnected;
    }
    if(Owned && !Visible) {Released=1;Halt(CHASSIS_TEST_OK,0);}
    Owned=Visible;State.Visible=Visible;
    if(Preset!=State.Preset)Halt(CHASSIS_TEST_OK,0);
    State.Preset=Preset;
    if(!Claim) {Publish();return 0;}
    uint8_t RemoteStop=RemoteStopGeneration!=LastStopGeneration || (WasConnected && !RemoteConnected);
    LastStopGeneration=RemoteStopGeneration;WasConnected=RemoteConnected;
    if(!Visible || Entering)Halt(CHASSIS_TEST_OK,0);
    else if(!Online || (uint32_t)(Now-Heartbeat)>=ROBOT_CHASSIS_TEST_UI_TIMEOUT_MS)
        Halt(CHASSIS_TEST_DISPLAY_LOST,0);
    else if(RemoteStop)Halt(CHASSIS_TEST_REMOTE_STOP,0);
    else if(!SampleValid)Halt(CHASSIS_TEST_BAD_SAMPLE,0);
    else if(Stop)Halt(CHASSIS_TEST_OK,0);
    else
    {
        if(Start)
        {
            if((uint32_t)(Now-RequestTick)>=ROBOT_CHASSIS_TEST_UI_TIMEOUT_MS)
                Halt(CHASSIS_TEST_STALE_START,0);
            else
            {
                State.Running=1;State.Fault=CHASSIS_TEST_OK;State.FaultWheel=0;Started=RequestTick;
                for(unsigned i=0;i<ROBOT_WHEEL_COUNT;i++)LastMovement[i]=Now;
            }
        }
        if(State.Running && (uint32_t)(Now-Started)>=ROBOT_CHASSIS_TEST_DURATION_MS)
            Halt(CHASSIS_TEST_OK,0);
        if(State.Running && !Start)
        {
            for(unsigned i=0;i<ROBOT_WHEEL_COUNT;i++)
            {
                if(fabsf(Feedback->TargetSpeed[i])<0.03f || Speed_GetDelta((uint8_t)i)!=0)
                    LastMovement[i]=Now;
                if((uint32_t)(Now-Started)>=150U && Feedback->TargetSpeed[i]*Feedback->MeasuredSpeed[i]<0 &&
                   fabsf(Feedback->MeasuredSpeed[i])>0.01f)
                {Halt(CHASSIS_TEST_REVERSED_ENCODER,(uint8_t)(i+1));break;}
                if((uint32_t)(Now-LastMovement[i])>=ROBOT_CHASSIS_TEST_NO_ENCODER_MS)
                {Halt(CHASSIS_TEST_NO_ENCODER,(uint8_t)(i+1));break;}
            }
        }
    }
    if(State.Running)
    {
        State.RemainingMs=ROBOT_CHASSIS_TEST_DURATION_MS-(uint32_t)(Now-Started);
        Command->Enabled=1;
        if(State.Preset>=CHASSIS_TEST_M1)
        {
            Command->Mode=ROBOT_COMMAND_WHEELS;
            Command->Target.Wheels[State.Preset-CHASSIS_TEST_M1]=ROBOT_CHASSIS_TEST_SPEED_MPS;
        }
        else switch(State.Preset)
        {
        case CHASSIS_TEST_FORWARD:Command->Target.Velocity.Vx=ROBOT_CHASSIS_TEST_SPEED_MPS;break;
        case CHASSIS_TEST_BACKWARD:Command->Target.Velocity.Vx=-ROBOT_CHASSIS_TEST_SPEED_MPS;break;
        case CHASSIS_TEST_LEFT:Command->Target.Velocity.Vy=ROBOT_CHASSIS_TEST_SPEED_MPS;break;
        case CHASSIS_TEST_RIGHT:Command->Target.Velocity.Vy=-ROBOT_CHASSIS_TEST_SPEED_MPS;break;
        case CHASSIS_TEST_CCW:Command->Target.Velocity.Wz=ROBOT_CHASSIS_TEST_TURN_RADPS;break;
        case CHASSIS_TEST_CW:Command->Target.Velocity.Wz=-ROBOT_CHASSIS_TEST_TURN_RADPS;break;
        default:Halt(CHASSIS_TEST_BAD_SAMPLE,0);Command->Enabled=0;break;
        }
    }
    Publish();return 1;
}
