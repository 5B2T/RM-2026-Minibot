#include "motor_test.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    MotorTest t;
    MotorTest_Init(&t);
    assert(MotorTest_Step(&t, 0) == 0);
    const unsigned times[] = {0, 9, 10, 1009, 1010, 1509, 1510, 2509, 2510};
    const int cycle[] = {0, 0, 300, 300, 0, 0, -300, -300, 0};
    for (unsigned i = 0; i < sizeof(times)/sizeof(times[0]); ++i) {
        MotorTest_Command(&t, 'T', 100);
        assert(MotorTest_Step(&t, 100 + times[i]) == cycle[i]);
    }
    for (unsigned direction = 0; direction < 2; ++direction) {
        MotorTest_Command(&t, direction ? 'r' : 'f', 100);
        assert(MotorTest_Step(&t, 109) == 0);
        assert(MotorTest_Step(&t, 110) == (direction ? -300 : 300));
        MotorTest_Command(&t, '\r', 500);
        MotorTest_Command(&t, '\n', 500);
        MotorTest_Command(&t, '?', 500);
        assert(t.StartedMs == 100);
        assert(MotorTest_Step(&t, 1109) == (direction ? -300 : 300));
        assert(MotorTest_Step(&t, 1110) == 0);
        assert(t.Mode == MOTOR_TEST_IDLE);
    }
    // Stop at each sequence stage must cancel all subsequent motion.
    for (unsigned i = 0; i < sizeof(times)/sizeof(times[0]); ++i) {
        MotorTest_Command(&t, 'T', 0);
        (void)MotorTest_Step(&t, times[i]);
        MotorTest_Command(&t, 's', times[i]);
        assert(MotorTest_Step(&t, times[i]) == 0);
        assert(MotorTest_Step(&t, times[i] + 5000) == 0);
    }
    MotorTest_Command(&t, 'F', 0);
    assert(MotorTest_Step(&t, 100) == 300);
    MotorTest_Command(&t, 'R', 100);
    assert(MotorTest_Step(&t, 100) == 0);
    assert(MotorTest_Step(&t, 109) == 0);
    assert(MotorTest_Step(&t, 110) == -300);
    MotorTest_Command(&t, 'R', 900);
    assert(MotorTest_Step(&t, 909) == 0);
    assert(MotorTest_Step(&t, 1909) == -300);
    assert(MotorTest_Step(&t, 1910) == 0);
    // Tick wrap, plus a delayed scheduler skipping expired motion.
    MotorTest_Command(&t, 'T', UINT32_MAX - 5U);
    assert(MotorTest_Step(&t, 3) == 0);
    assert(MotorTest_Step(&t, 4) == 300);
    assert(MotorTest_Step(&t, 1004) == 0);
    assert(MotorTest_Step(&t, 1504) == -300);
    assert(MotorTest_Step(&t, 2504) == 0);
    MotorTest_Command(&t, 'T', 0);
    assert(MotorTest_Step(&t, 10000) == 0);
    assert(t.Mode == MOTOR_TEST_IDLE);
    assert(MotorTest_EncoderDelta(0, 65535) == 1);
    assert(MotorTest_EncoderDelta(65535, 0) == -1);
    assert(MotorTest_EncoderDelta(123, 100) == 23);
    assert(MotorTest_EncoderDelta(100, 123) == -23);
    assert(MotorTest_EncoderDelta(100, 100) == 0);
    puts("PASS: boot/commands/stages/stop/reversal/retrigger/tick wrap/encoder wrap");
    return 0;
}
