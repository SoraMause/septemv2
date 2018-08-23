#ifndef __TIMER_H
#define __TIMER_H

#include "global_var.h"
#include "Geometry.h"

void interrupt();

//*********************************************************************
// debug
//*********************************************************************
void motorControllercheckConst();
EncValue checkEncValue();
Velocity checkVelocity();

#endif /* __TIMER_H */