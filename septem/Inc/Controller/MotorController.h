#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H

#include "global_var.h"
#include "Geometry.h" 

typedef struct {
  int32_t left;
  int32_t right;
}MotorDuty;

MotorDuty updateMotorDuty( void );
Velocity updateMotorData( void );

#endif /* __MOTORCONTROLLER_H */