#include "algorithm/mecanum.h"
#include <math.h>
#include <stddef.h>
uint8_t Mecanum_Inverse(float Vx, float Vy, float Wz, float WheelbaseM, float TrackM,
                        float WheelLimitMps, float Wheels[MECANUM_WHEEL_COUNT])
{
    if(Wheels==NULL)return 0;
    for(uint8_t i=0;i<MECANUM_WHEEL_COUNT;i++)Wheels[i]=0;
    if(!isfinite(Vx) || !isfinite(Vy) || !isfinite(Wz) ||
       !isfinite(WheelbaseM) || WheelbaseM<=0 || !isfinite(TrackM) || TrackM<=0 ||
       !isfinite(WheelLimitMps) || WheelLimitMps<=0)return 0;
    float Rotation=(WheelbaseM*0.5f+TrackM*0.5f)*Wz;
    float Raw[MECANUM_WHEEL_COUNT]={Vx-Vy-Rotation,Vx+Vy+Rotation,
                                  Vx+Vy-Rotation,Vx-Vy+Rotation};
    float Peak=0;
    for(uint8_t i=0;i<MECANUM_WHEEL_COUNT;i++)
    {
        if(!isfinite(Raw[i]))return 0;
        if(fabsf(Raw[i])>Peak)Peak=fabsf(Raw[i]);
    }
    float Scale=Peak>WheelLimitMps?WheelLimitMps/Peak:1.0f;
    for(uint8_t i=0;i<MECANUM_WHEEL_COUNT;i++)Wheels[i]=Raw[i]*Scale;
    return 1;
}
