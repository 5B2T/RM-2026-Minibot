#include "actuator/base_position.h"
#include "actuator/base_motor.h"
#include "robot/robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static const BasePosition_Config P=ROBOT_ARM_BASE_POSITION_CONFIG;
static const BaseMotor_Config V=ROBOT_ARM_BASE_CONTROL_CONFIG;
static void PlantTarget(BasePosition_Controller *C,BaseMotor_Controller *M,float Goal,
                        float *Angle,float *Rpm,unsigned Cycles)
{
    assert(BasePosition_SetTarget(C,Goal));
    float Counts=0,Start=*Angle,Overshoot=0; // Quantized feedback and destination overshoot.
    for(unsigned i=0;i<Cycles;i++)
    {
        float Before=C->ReferenceDeg;
        float Rate=BasePosition_Step(C,*Angle,M->FilteredRpm,.01f);
        assert(fabsf(C->ReferenceDeg-Before)<=P.TargetRateDps*.01f+.001f);
        assert(isfinite(Rate) && fabsf(Rate)<=1);
        Counts+=*Rpm*ROBOT_ARM_BASE_CPR*.01f/60;
        int Delta=(int)Counts;Counts-=Delta;
        int Out=BaseMotor_Step(M,Rate,Delta*60/(ROBOT_ARM_BASE_CPR*.01f),.01f);
        float Speed=fmaxf(0,(fabsf((float)Out)-40)/28)*(Out<0?-1:1);
        if(fabsf(*Rpm)<.05f && abs(Out)<90)Speed=0;
        *Rpm+=(Speed-*Rpm)*.01f/.08f;*Angle+=*Rpm*.06f;
        Overshoot=fmaxf(Overshoot,Goal>=Start?*Angle-Goal:Goal-*Angle);
        assert(!M->Fault && !C->Invalid && fabsf(*Rpm)<17.5f);
    }
    assert(fabsf(*Angle-Goal)<P.ReleaseDeg && fabsf(*Rpm)<.1f && C->AtTarget && M->PWM==0);
    printf("POSITION MODEL goal=%.1f final=%.3f overshoot=%.2fdeg\n",Goal,*Angle,Overshoot);
    assert(Overshoot<5); // Quantized test plant; real loaded overshoot still needs measurement.
}
int main(void)
{
    BasePosition_Controller C;assert(BasePosition_Init(&C,&P));
    BasePosition_Start(&C,0,1);assert(BasePosition_SetTarget(&C,90));
    for(unsigned i=0;i<20;i++)assert(BasePosition_Step(&C,0,0,.01f)==0 && !C.PickedUp);
    assert(BasePosition_SetTarget(&C,0));assert(BasePosition_Step(&C,0,1,.01f)==0 && !C.PickedUp);
    assert(BasePosition_Step(&C,0,0,.01f)==0 && C.PickedUp);
    assert(BasePosition_SetTarget(&C,90));for(unsigned i=0;i<4;i++)BasePosition_Step(&C,0,0,.01f);
    assert(C.ReferenceDeg<=3.601f && C.ReferenceDeg>3.59f);
    BasePosition_Stop(&C);assert(BasePosition_Step(&C,0,0,.01f)==0 && !C.PickedUp);
    BasePosition_Start(&C,179,0);assert(BasePosition_SetTarget(&C,-179));
    for(unsigned i=0;i<4;i++)assert(BasePosition_Step(&C,179,0,.01f)<=0); // No modulo shortcut.
    BasePosition_Start(&C,0,0);assert(BasePosition_SetTarget(&C,0));
    assert(BasePosition_Step(&C,0,0,.01f)==0 && C.AtTarget);
    assert(BasePosition_Step(&C,.5f,0,.01f)==0 && C.AtTarget);
    assert(BasePosition_Step(&C,1.0f,0,.01f)<0 && !C.AtTarget);
    assert(BasePosition_SetTarget(&C,1000) && C.TargetDeg==180);
    assert(BasePosition_SetTarget(&C,-1000) && C.TargetDeg==-180);
    BasePosition_Start(&C,0,0);assert(BasePosition_SetTarget(&C,10));
    float Rate=0;for(unsigned i=0;i<30;i++)Rate=BasePosition_Step(&C,0,0,.01f);
    assert(Rate*P.MaxRpm>=5); // 10deg error used to request just2.5RPM.
    BasePosition_Start(&C,0,0);assert(BasePosition_SetTarget(&C,.5f));
    assert(BasePosition_Step(&C,0,0,.01f)>0); // Sub-degree nudge is no longer swallowed.
    assert(!BasePosition_SetTarget(&C,NAN) && C.Invalid && !C.Enabled);
    assert(BasePosition_Init(&C,&P));BasePosition_Start(&C,0,0);
    assert(BasePosition_Step(&C,0,0,NAN)==0 && C.Invalid);
    BaseMotor_Controller M;assert(BasePosition_Init(&C,&P) && BaseMotor_Init(&M,&V));
    float Angle=0,Rpm=0;BasePosition_Start(&C,0,0);
    PlantTarget(&C,&M,90,&Angle,&Rpm,1200);
    PlantTarget(&C,&M,-90,&Angle,&Rpm,1400);
    PlantTarget(&C,&M,180,&Angle,&Rpm,1600);
    PlantTarget(&C,&M,0,&Angle,&Rpm,1400);
    PlantTarget(&C,&M,.5f,&Angle,&Rpm,500);
    puts("PASS: pickup, slew, signed limits/no wrap, hold hysteresis, stop/invalid, cascaded +/-90/180/zero plant");
    return 0;
}
