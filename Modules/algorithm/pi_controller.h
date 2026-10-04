#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H
typedef struct
{
    float Kp;
    float Ki;
    float Integral;
    float OutputLimit;
} PI_Controller;
// Numerical module: caller validates finite input and positive dt/configuration.
void PI_Reset(PI_Controller *Controller);
float PI_Update(PI_Controller *Controller, float Error, float Dt);
#endif
