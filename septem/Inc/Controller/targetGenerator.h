#ifndef __TARGETGENERATOR_H
#define __TARGETGENERATOR_H

void resetMotion( void );
void resetRadParam( void );

void updateTargetVelocity( void );

void wallOutCorrection( void );

float updateVelocityAccele( float measured );
float updateAngularAccele( void );

#endif /* __TARGETGENERATOR_H */