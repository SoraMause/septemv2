#ifndef __TIMER_H
#define __TIMER_H

#include "global_var.h"
#include "Geometry.h"

#include "motorController.h"

void interrupt( void );
void machineRadCalculation( void );
void setControl( int8_t _in );

//*********************************************************************
// debug
//*********************************************************************
Velocity checkVelocity( void );
MotorDuty checkMotorDuty( void );

#endif /* __TIMER_H */