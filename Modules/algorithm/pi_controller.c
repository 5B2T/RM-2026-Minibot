#include "algorithm/pi_controller.h"
void PI_Reset(PI_Controller *Controller)
{
    Controller->Integral=0;
}
float PI_Update(PI_Controller *Controller, float Error, float Dt)
{
    float Integral=Controller->Integral+Error*Dt;
    float Output=Controller->Kp*Error+Controller->Ki*Integral;
    if(Output>Controller->OutputLimit)Output=Controller->OutputLimit;
    if(Output<-Controller->OutputLimit)Output=-Controller->OutputLimit;
    // Preserve existing conditional integration behavior.
    if((Output>=Controller->OutputLimit && Error>0) ||
       (Output<=-Controller->OutputLimit && Error<0))Integral=Controller->Integral;
    Controller->Integral=Integral;
    return Output;
}
