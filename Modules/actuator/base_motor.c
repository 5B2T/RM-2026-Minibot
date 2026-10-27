#include "actuator/base_motor.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
static uint8_t Valid(const BaseMotor_Config *P)
{
    return P && isfinite(P->Kp) && P->Kp>=0 && isfinite(P->Ki) && P->Ki>=0 &&
        isfinite(P->Feedforward) && P->Feedforward>=0 && isfinite(P->StaticPwm) && P->StaticPwm>=0 &&
        isfinite(P->MaxRpm) && P->MaxRpm>0 && P->MaxRpm<=40 &&
        isfinite(P->OutputLimit) && P->OutputLimit>0 && P->OutputLimit<=999 && P->StaticPwm<P->OutputLimit &&
        isfinite(P->AccelRpmS) && P->AccelRpmS>0 && isfinite(P->FilterTauS) && P->FilterTauS>0 &&
        isfinite(P->ReverseDelayS) && P->ReverseDelayS>=.05f &&
        isfinite(P->FeedbackTimeoutS) && P->FeedbackTimeoutS>=.2f;
}
uint8_t BaseMotor_Init(BaseMotor_Controller *C,const BaseMotor_Config *Config)
{
    if(!C || !Valid(Config))return 0;
    BaseMotor_Config Copy=*Config;memset(C,0,sizeof(*C));C->Config=Copy;
    C->OffSeconds=Copy.ReverseDelayS;return 1;
}
void BaseMotor_Stop(BaseMotor_Controller *C)
{
    if(C->PWM)C->OffSeconds=0;
    C->PWM=0;C->TargetRpm=C->SetpointRpm=C->Integral=0;
    C->MissingSeconds=C->WrongSignSeconds=0;
}
static int16_t Fail(BaseMotor_Controller *C,BaseMotor_Fault Fault)
{BaseMotor_Stop(C);C->Fault=Fault;return 0;}
int16_t BaseMotor_Step(BaseMotor_Controller *C,float Rate,float RawRpm,float DtS)
{
    if(!Valid(&C->Config))return Fail(C,BASE_MOTOR_BAD_CONFIG);
    if(!isfinite(Rate) || fabsf(Rate)>1 || !isfinite(RawRpm) || fabsf(RawRpm)>60 ||
       !isfinite(DtS) || DtS<=0 || DtS>.1f)return Fail(C,BASE_MOTOR_BAD_SAMPLE);
    BaseMotor_Config *P=&C->Config;
    C->FilteredRpm+=(RawRpm-C->FilteredRpm)*DtS/(P->FilterTauS+DtS);
    if(C->Fault || Rate==0)
    {BaseMotor_Stop(C);C->OffSeconds=fminf(C->OffSeconds+DtS,10);return 0;}
    C->TargetRpm=Rate*P->MaxRpm;
    int Sign=Rate>0?1:-1;
    if(C->LastSign && Sign!=C->LastSign)
    {
        if(C->PWM)C->OffSeconds=0;
        C->PWM=0;C->Integral=C->SetpointRpm=0;C->OffSeconds+=DtS;
        if(C->OffSeconds<P->ReverseDelayS || fabsf(C->FilteredRpm)>.4f)return 0;
    }
    C->LastSign=(int8_t)Sign;C->OffSeconds=0;
    float Step=P->AccelRpmS*DtS,Change=C->TargetRpm-C->SetpointRpm;
    C->SetpointRpm+=fmaxf(-Step,fminf(Step,Change));
    float Error=(C->SetpointRpm-C->FilteredRpm)*Sign;
    float Integral=C->Integral+P->Ki*Error*DtS;
    Integral=fmaxf(-P->OutputLimit,fminf(P->OutputLimit,Integral));
    float FF=P->StaticPwm+P->Feedforward*fabsf(C->SetpointRpm);
    float Output=FF+P->Kp*Error+Integral;
    if(!((Output>P->OutputLimit && Error>0) || (Output<0 && Error<0)))C->Integral=Integral;
    Output=FF+P->Kp*Error+C->Integral;
    C->PWM=(int16_t)(Sign*(int)(fmaxf(0,fminf(P->OutputLimit,Output))+.5f));
    if(abs(C->PWM)>=100)
    {
        C->MissingSeconds=fabsf(RawRpm)<.01f?C->MissingSeconds+DtS:0;
        C->WrongSignSeconds=Sign*C->FilteredRpm<-.5f?C->WrongSignSeconds+DtS:0;
        if(C->MissingSeconds>=P->FeedbackTimeoutS)return Fail(C,BASE_MOTOR_NO_ENCODER);
        if(C->WrongSignSeconds>=.12f)return Fail(C,BASE_MOTOR_REVERSED);
    }
    else C->MissingSeconds=C->WrongSignSeconds=0;
    return C->PWM;
}
