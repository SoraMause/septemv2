#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

#include <float.h>

#include "config.h"

float PID( float target, float measurement, float *sum, float *old, float kp, 
                    float ki, float kd, float maxim, float minmam );

void PIDReset( float *sum, float *old );

#endif /* __PIDCONTROLLER_H */