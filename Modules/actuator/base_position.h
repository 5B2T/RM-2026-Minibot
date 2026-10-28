#ifndef BASE_POSITION_H
#define BASE_POSITION_H
#include <stdint.h>
typedef struct
{
    float MinDeg,MaxDeg,MaxRpm,TargetRateDps,KpRpmPerDeg,MinRpm;
    float ToleranceDeg,ReleaseDeg,PickupDeg,StoppedRpm;
} BasePosition_Config;
typedef struct
{
    BasePosition_Config Config;
    float TargetDeg,ReferenceDeg,ErrorDeg;
    uint8_t Enabled,PickedUp,AtTarget,Invalid;
} BasePosition_Controller;
uint8_t BasePosition_Init(BasePosition_Controller *C,const BasePosition_Config *Config);
void BasePosition_Stop(BasePosition_Controller *C);
// Relative to the manually established boot zero; never wraps at +/-180 degrees.
void BasePosition_Start(BasePosition_Controller *C,float MeasuredDeg,uint8_t RequirePickup);
uint8_t BasePosition_SetTarget(BasePosition_Controller *C,float TargetDeg);
// Returns a normalized speed for the existing speed PI. Never writes PWM itself.
float BasePosition_Step(BasePosition_Controller *C,float MeasuredDeg,float MeasuredRpm,float DtS);
#endif
