#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H

#include "global_var.h"
#include "Geometry.h" 

void calcMotorConst( void );
void updateMotorDuty( void );
Velocity updateMotorData( void );

#endif /* __MOTORCONTROLLER_H */