#ifndef CHASSIS_TEST_DEF_H
#define CHASSIS_TEST_DEF_H
#include <stdint.h>
typedef enum {
    CHASSIS_TEST_COMBINED=0,CHASSIS_TEST_M1,CHASSIS_TEST_M2,CHASSIS_TEST_M3,CHASSIS_TEST_M4,
    CHASSIS_TEST_COUNT
} ChassisTest_Preset;
typedef enum {
    CHASSIS_ACTION_FORWARD=0,CHASSIS_ACTION_BACKWARD,CHASSIS_ACTION_LEFT,
    CHASSIS_ACTION_RIGHT,CHASSIS_ACTION_CCW,CHASSIS_ACTION_CW
} ChassisTest_Action;
typedef enum {
    CHASSIS_TEST_OK=0,CHASSIS_TEST_NO_ENCODER,CHASSIS_TEST_REVERSED_ENCODER,
    CHASSIS_TEST_DISPLAY_LOST,CHASSIS_TEST_REMOTE_STOP,CHASSIS_TEST_BAD_SAMPLE,
    CHASSIS_TEST_STALE_START
} ChassisTest_Fault;
typedef struct {
    ChassisTest_Preset Preset;
    ChassisTest_Fault Fault;
    uint8_t Visible,Running,FaultWheel; // FaultWheel = physical M1..M4, zero if unrelated.
    ChassisTest_Action ActiveAction;
    uint8_t Step,Lap,Completed; // Combined steps 0..7 square, 8 CCW, 9 CW; square lap 1..2.
} ChassisTest_Status;
#endif
