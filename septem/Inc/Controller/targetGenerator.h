#ifndef __TARGETGENERATOR_H
#define __TARGETGENERATOR_H

#include <stdint.h>

void resetMotion( void );
void resetRadParam( void );

void updateTargetVelocity( void );

void wallOutCorrection( void );

void setControlWallPD( int8_t _able );
float wallSidePD( float kp, float kd, float maxim );

float updateVelocityAccele( float measured );
float updateAngularAccele( void );

#endif /* __TARGETGENERATOR_H */