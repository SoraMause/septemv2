#ifndef __MOTION_H
#define __MOTION_H

#include <stdint.h>

//---------------------------------------------------------------------
// macro define 
//---------------------------------------------------------------------
// motion name 
#define DELAY_TIME          300 //(ms)

//---------------------------------------------------------------------
// function protrype
//---------------------------------------------------------------------
void speedTrapezoid( float L ,float accele, float target, float start, float end );
float speedNext( float distance );

void yawrateTrapezoid( float L ,float accele, float target );
float yawrateNext( float rad );

void setSlarom( float rad, float accele, float rad_target, float speed_target, float before_distance, float after_distance );
float slaromNext( float distance, float rad );

void motionDelay( void );

void setMotionEnd( int8_t _end );
int8_t checkEndMotion( void );

#endif /* __MOTION_H */