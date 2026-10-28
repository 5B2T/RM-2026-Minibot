#include "actuator/base_position.h"
#include <math.h>
#include <string.h>
static uint8_t Valid(const BasePosition_Config *P)
{
    return P && isfinite(P->MinDeg) && isfinite(P->MaxDeg) && P->MinDeg<0 && P->MaxDeg>0 &&
        isfinite(P->MaxRpm) && P->MaxRpm>0 && P->MaxRpm<=15 &&
        isfinite(P->TargetRateDps) && P->TargetRateDps>0 && P->TargetRateDps<=P->MaxRpm*6 &&
        isfinite(P->KpRpmPerDeg) && P->KpRpmPerDeg>0 &&
        isfinite(P->MinRpm) && P->MinRpm>0 && P->MinRpm<=P->MaxRpm &&
        isfinite(P->ToleranceDeg) && P->ToleranceDeg>0 &&
        isfinite(P->ReleaseDeg) && P->ReleaseDeg>P->ToleranceDeg &&
        isfinite(P->PickupDeg) && P->PickupDeg>=P->ToleranceDeg &&
        isfinite(P->StoppedRpm) && P->StoppedRpm>0;
}
uint8_t BasePosition_Init(BasePosition_Controller *C,const BasePosition_Config *P)
{
    if(!C || !Valid(P))return 0;
    BasePosition_Config Copy=*P;memset(C,0,sizeof(*C));C->Config=Copy;return 1;
}
void BasePosition_Stop(BasePosition_Controller *C)
{C->Enabled=C->PickedUp=C->AtTarget=0;}
void BasePosition_Start(BasePosition_Controller *C,float MeasuredDeg,uint8_t RequirePickup)
{
    BasePosition_Stop(C);C->Invalid=0;
    if(!isfinite(MeasuredDeg)){C->Invalid=1;return;}
    C->ReferenceDeg=C->TargetDeg=MeasuredDeg;C->ErrorDeg=0;
    C->Enabled=1;C->PickedUp=!RequirePickup;
}
uint8_t BasePosition_SetTarget(BasePosition_Controller *C,float TargetDeg)
{
    if(!isfinite(TargetDeg)){C->Invalid=1;BasePosition_Stop(C);return 0;}
    C->TargetDeg=fmaxf(C->Config.MinDeg,fminf(C->Config.MaxDeg,TargetDeg));return 1;
}
float BasePosition_Step(BasePosition_Controller *C,float MeasuredDeg,float MeasuredRpm,float DtS)
{
    if(!Valid(&C->Config) || !isfinite(MeasuredDeg) || !isfinite(MeasuredRpm) ||
       !isfinite(DtS) || DtS<=0 || DtS>.1f)
    {C->Invalid=1;BasePosition_Stop(C);return 0;}
    if(!C->Enabled || C->Invalid)return 0;
    const BasePosition_Config *P=&C->Config;
    C->ErrorDeg=C->TargetDeg-MeasuredDeg;
    if(!C->PickedUp)
    {
        C->ReferenceDeg=MeasuredDeg;
        if(fabsf(C->ErrorDeg)<=P->PickupDeg && fabsf(MeasuredRpm)<=P->StoppedRpm)
            C->PickedUp=1;
        return 0; // Acquiring never moves on the same control tick.
    }
    float Change=C->TargetDeg-C->ReferenceDeg,Step=P->TargetRateDps*DtS;
    C->ReferenceDeg+=fmaxf(-Step,fminf(Step,Change));
    if(C->AtTarget && fabsf(C->ErrorDeg)<=P->ReleaseDeg)return 0;
    C->AtTarget=0;
    if(fabsf(C->ErrorDeg)<=P->ToleranceDeg)
    {
        C->AtTarget=fabsf(MeasuredRpm)<=P->StoppedRpm;
        return 0;
    }
    float Error=C->ReferenceDeg-MeasuredDeg;
    if(fabsf(Error)<=P->ToleranceDeg)return 0;
    float Rpm=fmaxf(P->MinRpm,fminf(P->MaxRpm,fabsf(Error)*P->KpRpmPerDeg));
    if(Error<0)Rpm=-Rpm;
    if((MeasuredDeg>=P->MaxDeg && Rpm>0) || (MeasuredDeg<=P->MinDeg && Rpm<0))return 0;
    return Rpm/P->MaxRpm;
}
