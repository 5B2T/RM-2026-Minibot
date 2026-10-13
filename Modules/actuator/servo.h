#ifndef ACTUATOR_SERVO_H
#define ACTUATOR_SERVO_H
#include <stdint.h>
typedef enum {SERVO_POSITION=0,SERVO_CONTINUOUS=1} Servo_Mode;
typedef struct
{
    Servo_Mode Mode;
    uint16_t MinUs,NeutralUs,MaxUs;
    float MaxAngleDeg;
    int8_t Direction;
} Servo_Config;
// Position: degrees. Continuous: normalized rate -1..1, zero = neutral pulse.
uint8_t Servo_Pulse(const Servo_Config *Config,float Value,uint16_t *PulseUs);
#endif
