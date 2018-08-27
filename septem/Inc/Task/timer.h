#ifndef __TIMER_H
#define __TIMER_H

#include "global_var.h"
#include "Geometry.h"

void interrupt( void );

//*********************************************************************
// debug
//*********************************************************************
EncValue checkEncValue( void );
Velocity checkVelocity( void );

#endif /* __TIMER_H */