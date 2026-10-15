#ifndef CHASSIS_TEST_H
#define CHASSIS_TEST_H
#include "robot/robot_def.h"
void ChassisTest_Init(void); // Before scheduler start.
// UI task submits intent only. No hardware writes here.
void ChassisTest_View(uint8_t Visible,uint8_t Online,uint32_t NowMs);
void ChassisTest_Next(void);
void ChassisTest_Toggle(uint32_t NowMs);
void ChassisTest_Stop(void);
void ChassisTest_Close(void);
ChassisTest_Status ChassisTest_GetStatus(void);
// Control task only. Return 1 while locally owned, including one stopped exit cycle.
uint8_t ChassisTest_Control(Robot_ChassisCommand *Command,const Robot_Status *Feedback,
    uint8_t SampleValid,uint32_t NowMs,uint32_t RemoteStopGeneration,uint8_t RemoteConnected);
uint8_t ChassisTest_TakeRelease(void); // Control owner clears old remote motion on exit.
#endif
