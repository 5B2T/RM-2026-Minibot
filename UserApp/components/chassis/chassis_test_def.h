#ifndef CHASSIS_TEST_DEF_H
#define CHASSIS_TEST_DEF_H
#include <stdint.h>
typedef enum {
    CHASSIS_TEST_FORWARD=0,CHASSIS_TEST_BACKWARD,CHASSIS_TEST_LEFT,CHASSIS_TEST_RIGHT,
    CHASSIS_TEST_CCW,CHASSIS_TEST_CW,CHASSIS_TEST_M1,CHASSIS_TEST_M2,
    CHASSIS_TEST_M3,CHASSIS_TEST_M4,CHASSIS_TEST_COUNT
} ChassisTest_Preset;
typedef enum {
    CHASSIS_TEST_OK=0,CHASSIS_TEST_NO_ENCODER,CHASSIS_TEST_REVERSED_ENCODER,
    CHASSIS_TEST_DISPLAY_LOST,CHASSIS_TEST_REMOTE_STOP,CHASSIS_TEST_BAD_SAMPLE,
    CHASSIS_TEST_STALE_START
} ChassisTest_Fault;
typedef struct {
    uint32_t RemainingMs;
    ChassisTest_Preset Preset;
    ChassisTest_Fault Fault;
    uint8_t Visible,Running,FaultWheel; // FaultWheel = physical M1..M4, zero if unrelated.
} ChassisTest_Status;
#endif
