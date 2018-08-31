#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

#include "config.h"

float PID( float target, float measurement, float *sum, float *old, float kp, 
                    float ki, float kd, float maxim );

void PIDReset( float *sum, float *old );

#endif /* __PIDCONTROLLER_H */