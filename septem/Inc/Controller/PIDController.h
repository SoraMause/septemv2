#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

#include "config.h"

float PID( float target, float measurement, float *sum, float *old, float kp, 
                    float ki, float kd, float maxim );


#endif /* __PIDCONTROLLER_H */