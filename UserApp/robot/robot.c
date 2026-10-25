#include "robot/robot.h"
#include "robot/robot_config.h"
#include "components/chassis/chassis.h"
#include "components/chassis/chassis_test.h"
#include "components/command/remote_control.h"
#include "motor/bsp_motor.h"
#include "encoder/bsp_encoder.h"
#include "radio/nrf24l01.h"
#include "time/bsp_time.h"
#include "i2c/bsp_i2c.h"
#include "components/display/debug_display.h"
#include "input/bsp_buttons.h"
#include "components/power/power_monitor.h"
#include "components/arm/arm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
static Robot_Status StatusSnapshot;
static uint8_t RadioRecoveryPending;
static uint8_t Ready;
static uint8_t DisplayScheduled;
static uint32_t DisplayTick,DisplayDelay;
static struct {Arm_Command Command;uint32_t CreatedMs;uint8_t Pending,Stop;} ArmMailbox;
static uint8_t RadioSeen,ArmRadioOwned;
static uint32_t RadioStopGeneration;
_Static_assert(RX_PLOAD_WIDTH==REMOTE_CONTROL_FRAME_SIZE, "Radio payload must match protocol");

uint8_t Robot_Init(void)
{
    Ready=0;
    memset(&StatusSnapshot, 0, sizeof(StatusSnapshot));
    RadioRecoveryPending=0;
    memset(&ArmMailbox,0,sizeof(ArmMailbox));
    RadioSeen=ArmRadioOwned=0;RadioStopGeneration=0;
    RemoteControl_Init();
    ChassisTest_Init();
    PowerMonitor_Init();
    DebugDisplay_Init();
    DisplayScheduled=0;
    if(!BSP_Motor_Init() || !BSP_Encoder_Init() || !Chassis_Init() || !Arm_Init())
    {
        BSP_Motor_Disable();
        Arm_Disable();
        return 0;
    }
    NRF24L01_SPI_Init();
    (void)NRF24L01_RX_Mode();
    Ready=1;
    return 1;
}
void Robot_ControlReset(void)
{
    if(Ready) {Chassis_ResetSampling();Arm_ResetSampling();}
}
void Robot_ArmStop(void)
{
    taskENTER_CRITICAL();
    ArmMailbox.Pending=0;ArmMailbox.Stop=1;
    taskEXIT_CRITICAL();
}
uint8_t Robot_ArmSubmit(const Arm_Command *Command)
{
    if(!Ready)return 0;
    if(!Arm_CommandValid(Command)) {Robot_ArmStop();return 0;}
    if(Command->Action==ARM_STOP) {Robot_ArmStop();return 1;}
    // Radio owns the arm while V3 control is active. After a radio STOP/loss,
    // serial commands cannot bypass the global interlock.
    RemoteControl_LinkStatus Link=RemoteControl_GetLinkStatus();
    if(ArmRadioOwned || (RadioSeen && !Link.Armed))return 0;
    uint32_t NowMs=BSP_TimeMs();uint8_t Accepted=0;
    taskENTER_CRITICAL();
    if(!ArmMailbox.Stop)
    {
        ArmMailbox.Command=*Command;ArmMailbox.CreatedMs=NowMs;
        ArmMailbox.Pending=1;Accepted=1;
    }
    taskEXIT_CRITICAL();
    return Accepted;
}
static void ArmControlStep(float DtS,uint32_t NowMs,RemoteControl_LinkStatus Link)
{
    Arm_Command Command;uint32_t CreatedMs;uint8_t Pending,Stop;
    taskENTER_CRITICAL();
    Command=ArmMailbox.Command;CreatedMs=ArmMailbox.CreatedMs;
    Pending=ArmMailbox.Pending;Stop=ArmMailbox.Stop;
    ArmMailbox.Pending=ArmMailbox.Stop=0;
    taskEXIT_CRITICAL();
    RemoteControl_ArmInput Input=RemoteControl_TakeArmInput();
    if(Link.Connected || Link.StopGeneration!=RadioStopGeneration)RadioSeen=1;
    if(Link.StopGeneration!=RadioStopGeneration)Stop=1;
    RadioStopGeneration=Link.StopGeneration;
    if(RadioSeen && (!Link.Connected || !Link.Armed))Stop=1;
    // Local commissioning remains available before radio use, with its own lease.
    if(Stop) {Arm_Stop();ArmRadioOwned=0;}
    else if(Input.Enabled)
    {
        if(!ArmRadioOwned)Arm_Stop();
        ArmRadioOwned=1;
        Arm_RemoteInput(Input.Base,(Arm_Joint)Input.Joint,Input.Rate,Input.Initialize,Input.Gripper,NowMs);
    }
    else if(ArmRadioOwned) {Arm_Stop();ArmRadioOwned=0;}
    else if(Pending)(void)Arm_Request(&Command,CreatedMs,NowMs);
    Arm_Step(DtS,NowMs);
}
void Robot_ControlStep(float DtS)
{
    if(!Ready) {BSP_Motor_Disable();Arm_Stop();return;}
    Robot_ChassisCommand Command,RemoteCommand;
    uint8_t SampleValid=Chassis_Measure(DtS);
    RemoteControl_GetCommand(&RemoteCommand);
    RemoteControl_LinkStatus Link=RemoteControl_GetLinkStatus();
    if(Link.Connected || Link.StopGeneration!=RadioStopGeneration)RadioSeen=1;
    uint8_t GlobalLocked=RadioSeen && (!Link.Connected || !Link.Armed);
    if(GlobalLocked)ChassisTest_Stop();
    Robot_Status Feedback={0};Chassis_GetFeedback(&Feedback);
    uint8_t TestSample=SampleValid && DtS>0 && DtS<=0.1f;
    if(!ChassisTest_Control(&Command,&Feedback,TestSample,BSP_TimeMs(),Link.StopGeneration,Link.Connected))
        Command=RemoteCommand;
    if(ChassisTest_TakeRelease())RemoteControl_Disconnect(); // Require explicit radio recovery after local testing.
    if(GlobalLocked)Command.Enabled=0;
    if(!SampleValid)Chassis_Stop();
    else Chassis_Control(&Command,DtS);
    Robot_Status Status={0};
    Chassis_GetFeedback(&Status);
    Status.TickMs=BSP_TimeMs();
    Link=RemoteControl_GetLinkStatus();
    ArmControlStep(DtS,Status.TickMs,Link);
    Status.Arm=Arm_GetStatus();
    RemoteControl_TelemetryStatus Telemetry=RemoteControl_GetTelemetry();
    memcpy(Status.Telemetry.Raw,Telemetry.Raw,sizeof(Status.Telemetry.Raw));
    memcpy(Status.Telemetry.AngleCdeg,Telemetry.AngleCdeg,sizeof(Status.Telemetry.AngleCdeg));
    Status.Telemetry.TickMs=Telemetry.TickMs;
    Status.Telemetry.Valid=Telemetry.Valid;
    Status.Telemetry.Online=Telemetry.HasFrame &&
        (uint32_t)(Status.TickMs-Telemetry.TickMs)<ROBOT_TELEMETRY_TIMEOUT_MS;
    Status.Connected=Link.Connected;
    Status.Armed=Link.Armed;
    Status.ChassisTest=ChassisTest_GetStatus();
    taskENTER_CRITICAL();
    StatusSnapshot=Status;
    taskEXIT_CRITICAL();
}
uint32_t Robot_CommunicationStep(void)
{
    if(!Ready)return ROBOT_RADIO_RETRY_MS;
    if(RadioRecoveryPending)
    {
        (void)NRF24L01_RX_Mode();
        RadioRecoveryPending=0;
    }
    uint8_t Data[RX_PLOAD_WIDTH];
    NRF24L01_Result Result=NRF_EMPTY;
    for(uint8_t i=0;i<NRF_RX_FIFO_DEPTH;i++)
    {
        Result=NRF24L01_RxPacket(Data);
        if(Result!=NRF_OK)break;
        RemoteControl_Process(Data, RX_PLOAD_WIDTH);
    }
    if(Result!=NRF_OK && Result!=NRF_EMPTY)
    {
        RemoteControl_Disconnect();
        RadioRecoveryPending=1;
        return ROBOT_RADIO_RETRY_MS;
    }
    return ROBOT_COMM_POLL_MS;
}
void Robot_GetStatus(Robot_Status *Status)
{
    if(Status==NULL)return;
    taskENTER_CRITICAL();
    *Status=StatusSnapshot;
    taskEXIT_CRITICAL();
    Status->Power=PowerMonitor_GetStatus(BSP_TimeMs());
}
uint32_t Robot_DisplayStep(void)
{
    // Create the mutex in its service task. Before scheduler start, FreeRTOS
    // critical sections leave BASEPRI set and block the HAL tick / startup delays.
    (void)BSP_I2C_Init(); // Sensor/display failure must not prevent chassis initialization.
    uint32_t NowMs=BSP_TimeMs();
    uint8_t Keys=BSP_Buttons_Read();
    DebugDisplay_HandleKeys((Keys&BSP_BUTTON_NEXT)!=0,(Keys&BSP_BUTTON_BACK)!=0,NowMs);
    DebugDisplay_State View=DebugDisplay_GetState();
    ChassisTest_View(!View.MainMenu && View.Page==DEBUG_PAGE_CHASSIS_TEST,View.Online,NowMs);
    PowerMonitor_Step(NowMs);
    if(DisplayScheduled && (uint32_t)(NowMs-DisplayTick)<DisplayDelay)return ROBOT_DISPLAY_POLL_MS;
    Robot_Status Status;
    Robot_GetStatus(&Status);
    DisplayDelay=DebugDisplay_Step(&Status,BSP_TimeMs());
    DisplayTick=BSP_TimeMs();
    DisplayScheduled=1;
    return ROBOT_DISPLAY_POLL_MS; // Sensor polling continues during OLED retry delays.
}
