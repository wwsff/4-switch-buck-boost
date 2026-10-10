#ifndef PD_PID_H
#define PD_PID_H
#include <stdint.h>
typedef struct
{
    float kp, ki, kd, tau, out_min, out_max;
} PD_PIDConfig;
typedef struct
{
    float integral, derivative, previous_y;
    uint8_t initialized;
} PD_PIDState;
int PD_PIDValid(const PD_PIDConfig *p);
void PD_PIDReset(PD_PIDState *s);
float PD_PIDStep(const PD_PIDConfig *p, PD_PIDState *s, float r, float y, float dt);
#endif
