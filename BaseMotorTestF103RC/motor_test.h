#ifndef BASE_MOTOR_TEST_H
#define BASE_MOTOR_TEST_H
#include <stdint.h>
#define MOTOR_TEST_PWM 300 // ARR=999, ~30% duty; open loop, not RPM.
#define MOTOR_TEST_OFF_MS 10U
#define MOTOR_TEST_RUN_MS 1000U
#define MOTOR_TEST_GAP_MS 500U
typedef enum { MOTOR_TEST_IDLE=0, MOTOR_TEST_FORWARD, MOTOR_TEST_REVERSE, MOTOR_TEST_CYCLE } MotorTest_Mode;
typedef struct { MotorTest_Mode Mode; uint32_t StartedMs; } MotorTest;
void MotorTest_Init(MotorTest *Test);
// Single ASCII F/R/T/S; newline ignored. Motion always expires, never loops forever.
void MotorTest_Command(MotorTest *Test,uint8_t Command,uint32_t NowMs);
int16_t MotorTest_Step(MotorTest *Test,uint32_t NowMs);
// Four-edge decoding, including signed 16-bit timer wrap.
int32_t MotorTest_EncoderDelta(uint16_t Current,uint16_t Previous);
#endif
