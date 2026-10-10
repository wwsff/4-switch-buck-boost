#include "pd_pid.h"
#include <math.h>
#include <string.h>
static int finite_value(float x)
{
    return x == x && x <= 1.0e9f && x >= -1.0e9f;
}
int PD_PIDValid(const PD_PIDConfig *p)
{
    return finite_value(p->kp) && finite_value(p->ki) && finite_value(p->kd) &&
           finite_value(p->tau) && finite_value(p->out_min) && finite_value(p->out_max) &&
           p->kp >= 0 && p->ki >= 0 && p->kd >= 0 && p->tau >= 0 && p->out_min < p->out_max &&
           (p->kd == 0 || p->tau > 0);
}
void PD_PIDReset(PD_PIDState *s)
{
    memset(s, 0, sizeof(*s));
}
float PD_PIDStep(const PD_PIDConfig *p, PD_PIDState *s, float r, float y, float dt)
{
    float e, inc, candidate, u;
    if (!PD_PIDValid(p) || !finite_value(r) || !finite_value(y) || !(dt > 0) || !finite_value(dt))
    {
        PD_PIDReset(s);
        return 0;
    }
    if (!s->initialized)
    {
        s->previous_y = y;
        s->initialized = 1;
    }
    e = r - y;
    s->derivative =
        p->kd > 0 ? (p->tau * s->derivative - p->kd * (y - s->previous_y)) / (p->tau + dt) : 0;
    s->previous_y = y;
    inc = p->ki * dt * e;
    candidate = s->integral + inc;
    u = p->kp * e + candidate + s->derivative;
    if (!((u > p->out_max && inc > 0) || (u < p->out_min && inc < 0)))
        s->integral = candidate;
    u = p->kp * e + s->integral + s->derivative;
    if (!finite_value(u))
    {
        PD_PIDReset(s);
        return 0;
    }
    return u > p->out_max ? p->out_max : (u < p->out_min ? p->out_min : u);
}
