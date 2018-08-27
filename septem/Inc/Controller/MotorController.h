#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H

#include "global_var.h"
#include "Geometry.h" 

void calcMotorConst( void );
void updateMotorDuty( void );
Velocity updateMotorData( const EncValue enc_value );

#endif /* __MOTORCONTROLLER_H */