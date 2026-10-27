#include "actuator/base_motor.h"
#include "robot/robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static const BaseMotor_Config P=ROBOT_ARM_BASE_CONTROL_CONFIG;
static float Rpm,Fraction;
static float Sample(void){
 Fraction+=Rpm*6600*.01f/60;int Count=(int)Fraction;Fraction-=Count;return Count*60/(6600*.01f);
}
static int Tick(BaseMotor_Controller *C,float Rate){
 int Out=BaseMotor_Step(C,Rate,Sample(),.01f);
 float Goal=fmaxf(0,(fabsf((float)Out)-100)/40.0f)*(Out<0?-1:1);
 if(fabsf(Rpm)<.05f && abs(Out)<150)Goal=0; // Loaded breakaway exceeds running friction.
 Rpm+=(Goal-Rpm)*.01f/.08f;assert(abs(Out)<=C->Config.OutputLimit);return Out;
}
int main(void){
 BaseMotor_Controller C;assert(BaseMotor_Init(&C,&P));assert(Tick(&C,0)==0);
 float Peak=0;
 for(int i=0;i<400;i++){float Before=C.SetpointRpm;Tick(&C,1);assert(C.SetpointRpm-Before<=P.AccelRpmS*.01f+.0001f);Peak=fmaxf(Peak,Rpm);}
 assert(!C.Fault && fabsf(C.FilteredRpm-P.MaxRpm)<.15f && Peak<P.MaxRpm+.6f);
 for(int i=0;i<12;i++)assert(Tick(&C,-1)==0);
 for(int i=0;i<400;i++)assert(Tick(&C,-1)<=0);
 assert(!C.Fault && fabsf(C.FilteredRpm+P.MaxRpm)<.15f);
 assert(Tick(&C,0)==0 && C.Integral==0);
 for(int i=0;i<40;i++)assert(Tick(&C,0)==0);
 assert(BaseMotor_Init(&C,&P));
 for(int i=0;i<200;i++)BaseMotor_Step(&C,1,0,.01f);
 assert(C.Fault==BASE_MOTOR_NO_ENCODER && C.PWM==0);
 assert(BaseMotor_Step(&C,0,0,.01f)==0 && C.Fault==BASE_MOTOR_NO_ENCODER);
 assert(BaseMotor_Init(&C,&C.Config) && !C.Fault);
 for(int i=0;i<80;i++)BaseMotor_Step(&C,1,-2,.01f);
 assert(C.Fault==BASE_MOTOR_REVERSED && C.PWM==0);
 assert(BaseMotor_Init(&C,&P));assert(BaseMotor_Step(&C,1,NAN,.01f)==0 && C.Fault);
 assert(BaseMotor_Init(&C,&P));assert(BaseMotor_Step(&C,1,0,0)==0 && C.Fault);
 puts("PASS: quantized plant tracking, ramp, output bound, reversal gap, STOP, missing/reversed/invalid feedback, fault latch/clear");
 return 0;
}
