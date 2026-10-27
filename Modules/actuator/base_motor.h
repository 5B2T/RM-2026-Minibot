#ifndef BASE_MOTOR_H
#define BASE_MOTOR_H
#include <stdint.h>
typedef struct
{
    float Kp,Ki,Feedforward,StaticPwm,MaxRpm,OutputLimit;
    float AccelRpmS,FilterTauS,ReverseDelayS,FeedbackTimeoutS;
} BaseMotor_Config;
typedef enum {BASE_MOTOR_OK=0,BASE_MOTOR_NO_ENCODER,BASE_MOTOR_REVERSED,
              BASE_MOTOR_BAD_SAMPLE,BASE_MOTOR_BAD_CONFIG} BaseMotor_Fault;
typedef struct
{
    BaseMotor_Config Config;
    float TargetRpm,SetpointRpm,FilteredRpm,Integral;
    float OffSeconds,MissingSeconds,WrongSignSeconds;
    int16_t PWM;
    int8_t LastSign;
    BaseMotor_Fault Fault;
} BaseMotor_Controller;
uint8_t BaseMotor_Init(BaseMotor_Controller *C,const BaseMotor_Config *Config);
// Keeps feedback faults latched; init / explicit stopped ZERO clears them.
void BaseMotor_Stop(BaseMotor_Controller *C);
int16_t BaseMotor_Step(BaseMotor_Controller *C,float Rate,float RawRpm,float DtS);
#endif
