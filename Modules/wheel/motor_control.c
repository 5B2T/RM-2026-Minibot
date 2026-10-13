#include "wheel/motor_control.h"
#include "wheel/speed.h"
#include "algorithm/pi_controller.h"
#include "motor/bsp_motor.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static MotorControl_HandleTypeDef MotorControl[WHEEL_COUNT];
static PI_Controller Controllers[WHEEL_COUNT];
static float TargetLimit;
static uint8_t Enabled;
static uint8_t Initialized;
static int8_t MotorPolarity[WHEEL_COUNT];
static int8_t EncoderPolarity[WHEEL_COUNT];
_Static_assert(WHEEL_COUNT==BSP_MOTOR_COUNT, "Motor mapping must match wheel count");

uint8_t MotorControl_Init(float Kp, float Ki, float OutputLimit, float TargetLimitMps)
{
    MotorControl_Stop();
    Initialized=0;
    if(!isfinite(Kp) || Kp<0 || !isfinite(Ki) || Ki<0 ||
       !isfinite(OutputLimit) || OutputLimit<=0 || OutputLimit>BSP_MOTOR_PWM_MAX ||
       !isfinite(TargetLimitMps) || TargetLimitMps<=0)return 0;
    TargetLimit=TargetLimitMps;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
    {
        MotorPolarity[i]=EncoderPolarity[i]=1;
        Controllers[i]=(PI_Controller){Kp,Ki,0,OutputLimit};
        MotorControl[i]=(MotorControl_HandleTypeDef){.MotorControl_Kp=Kp,.MotorControl_Ki=Ki};
    }
    Initialized=1;
    return 1;
}
uint8_t MotorControl_SetPolarity(const int8_t MotorSign[4], const int8_t EncoderSign[4])
{
    if(!Initialized || Enabled || MotorSign==NULL || EncoderSign==NULL)return 0;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
        if((MotorSign[i]!=1 && MotorSign[i]!=-1) ||
           (EncoderSign[i]!=1 && EncoderSign[i]!=-1))return 0;
    memcpy(MotorPolarity, MotorSign, sizeof(MotorPolarity));
    memcpy(EncoderPolarity, EncoderSign, sizeof(EncoderPolarity));
    return 1;
}
uint8_t MotorControl_SetGains(uint8_t Wheel, float Kp, float Ki)
{
    if(!Initialized || Enabled || Wheel>=WHEEL_COUNT ||
       !isfinite(Kp) || Kp<0 || !isfinite(Ki) || Ki<0)return 0;
    Controllers[Wheel].Kp=Kp;
    Controllers[Wheel].Ki=Ki;
    PI_Reset(&Controllers[Wheel]);
    MotorControl[Wheel].MotorControl_Kp=Kp;
    MotorControl[Wheel].MotorControl_Ki=Ki;
    MotorControl[Wheel].MotorControl_I=0;
    return 1;
}
float MotorControl_GetMeasuredSpeed(uint8_t Wheel)
{
    return Wheel<WHEEL_COUNT?Speed_Get(Wheel)*EncoderPolarity[Wheel]:0;
}
void MotorControl_SetTarget(float M1, float M2, float M3, float M4)
{
    if(!Initialized) {MotorControl_Stop();return;}
    float Target[WHEEL_COUNT]={M1,M2,M3,M4};
    float Peak=0;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
    {
        if(!isfinite(Target[i])) {MotorControl_Stop();return;}
        if(fabsf(Target[i])>Peak)Peak=fabsf(Target[i]);
    }
    float Scale=Peak>TargetLimit?TargetLimit/Peak:1.0f;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)MotorControl[i].MotorControl_Target=Target[i]*Scale;
    Enabled=1;
}
void MotorControl_Update(float SampleTime)
{
    if(!Enabled || !isfinite(SampleTime) || SampleTime<=0)
    {MotorControl_Stop();return;}
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
        if(!isfinite(MotorControl_GetMeasuredSpeed(i))) {MotorControl_Stop();return;}
    int PWM[WHEEL_COUNT];
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
    {
        float Error=MotorControl[i].MotorControl_Target-MotorControl_GetMeasuredSpeed(i);
        float Output=PI_Update(&Controllers[i], Error, SampleTime);
        if(!isfinite(Output) || !isfinite(Controllers[i].Integral))
        {MotorControl_Stop();return;}
        MotorControl[i].MotorControl_I=Controllers[i].Integral;
        MotorControl[i].MotorControl_Out=Output;
        PWM[i]=(int)Output*MotorPolarity[i];
    }
    BSP_Motor_Load(PWM);
    BSP_Motor_Enable();
}
void MotorControl_Stop(void)
{
    Enabled=0;
    for(uint8_t i=0;i<WHEEL_COUNT;i++)
    {
        PI_Reset(&Controllers[i]);
        MotorControl[i].MotorControl_Target=0;
        MotorControl[i].MotorControl_I=0;
        MotorControl[i].MotorControl_Out=0;
    }
    BSP_Motor_Disable();
}
MotorControl_HandleTypeDef MotorControl_GetState(uint8_t Wheel)
{
    return Wheel<WHEEL_COUNT?MotorControl[Wheel]:(MotorControl_HandleTypeDef){0};
}

uint8_t MotorControl_IsEnabled(void)
{
    return Enabled;
}
