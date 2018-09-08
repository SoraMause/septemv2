#ifndef __TARGETGENERATOR_H
#define __TARGETGENERATOR_H

#include <stdint.h>

void setSearchGain( void );
void setFastGain( void );

void resetMotion( void );
void resetRadParam( void );

void setMotionDistance( float _L_motion );

void updateTargetVelocity( void );

void setMazeWallUpdate( int8_t _able );
void certainMazeUpdateFlag( void );
int8_t checkMazeUpdateFlag( void );

void wallOutStraightCorrection( void );

void setControlWallPD( int8_t _able );
float wallSidePD( float kp, float kd, float maxim );

float updateVelocityAccele( float measured );
float updateAngularAccele( void );

#endif /* __TARGETGENERATOR_H */