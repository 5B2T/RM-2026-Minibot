#include "actuator/servo.h"
#include <stddef.h>
#include <math.h>
uint8_t Servo_Pulse(const Servo_Config *Config,float Value,uint16_t *PulseUs)
{
    if(Config==NULL || PulseUs==NULL || !isfinite(Value) ||
       Config->MinUs<500 || Config->MaxUs>2500 || Config->MinUs>=Config->MaxUs ||
       Config->NeutralUs<Config->MinUs || Config->NeutralUs>Config->MaxUs ||
       (Config->Direction!=1 && Config->Direction!=-1))return 0;
    float Pulse;
    if(Config->Mode==SERVO_POSITION)
    {
        if(!isfinite(Config->MaxAngleDeg) || Config->MaxAngleDeg<=0 || Value<0 || Value>Config->MaxAngleDeg)return 0;
        float Ratio=Value/Config->MaxAngleDeg;
        if(Config->Direction<0)Ratio=1.0f-Ratio;
        Pulse=Config->MinUs+Ratio*(Config->MaxUs-Config->MinUs);
    }
    else if(Config->Mode==SERVO_CONTINUOUS)
    {
        if(Value<-1 || Value>1)return 0;
        Value*=Config->Direction;
        Pulse=Config->NeutralUs+Value*(Value>=0?Config->MaxUs-Config->NeutralUs:Config->NeutralUs-Config->MinUs);
    }
    else return 0;
    *PulseUs=(uint16_t)(Pulse+0.5f);return 1;
}
