#include "motor_test.h"
void MotorTest_Init(MotorTest *Test) { Test->Mode=MOTOR_TEST_IDLE;Test->StartedMs=0; }
void MotorTest_Command(MotorTest *Test,uint8_t Command,uint32_t NowMs)
{
    if(Command>='a' && Command<='z')Command=(uint8_t)(Command-'a'+'A');
    MotorTest_Mode Mode;
    switch(Command)
    {
    case 'F':Mode=MOTOR_TEST_FORWARD;break;
    case 'R':Mode=MOTOR_TEST_REVERSE;break;
    case 'T':Mode=MOTOR_TEST_CYCLE;break;
    case 'S':Mode=MOTOR_TEST_IDLE;break;
    default:return;
    }
    Test->Mode=Mode;Test->StartedMs=NowMs;
}
int16_t MotorTest_Step(MotorTest *Test,uint32_t NowMs)
{
    if(Test->Mode==MOTOR_TEST_IDLE)return 0;
    uint32_t Age=NowMs-Test->StartedMs;
    // Every new command starts with a zero-output gap, including polarity reversal.
    if(Age<MOTOR_TEST_OFF_MS)return 0;
    Age-=MOTOR_TEST_OFF_MS;
    if(Test->Mode==MOTOR_TEST_CYCLE)
    {
        if(Age<MOTOR_TEST_RUN_MS)return MOTOR_TEST_PWM;
        if(Age<MOTOR_TEST_RUN_MS+MOTOR_TEST_GAP_MS)return 0;
        if(Age<2U*MOTOR_TEST_RUN_MS+MOTOR_TEST_GAP_MS)return -MOTOR_TEST_PWM;
    }
    else if(Age<MOTOR_TEST_RUN_MS)
        return Test->Mode==MOTOR_TEST_FORWARD?MOTOR_TEST_PWM:-MOTOR_TEST_PWM;
    Test->Mode=MOTOR_TEST_IDLE;
    return 0;
}
int32_t MotorTest_EncoderDelta(uint16_t Current,uint16_t Previous)
{
    uint16_t Raw=(uint16_t)(Current-Previous);
    return Raw>=32768U?(int32_t)Raw-65536:(int32_t)Raw;
}
