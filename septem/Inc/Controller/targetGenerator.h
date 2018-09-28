#ifndef __TARGETGENERATOR_H
#define __TARGETGENERATOR_H

#include <stdint.h>

void setSearchGain( void );
void setFastGain( void );

int8_t checkEmergyncyFlag( void );

void resetMotion( void );
void resetRadParam( void );

void setMotionDistance( float _L_motion );

void updateSearchTargetVelocity( float measurement );
void updateFastRunTargetVelocity( float measurement );

void setMazeWallUpdate( int8_t _able );
void certainMazeUpdateFlag( void );
int8_t checkMazeUpdateFlag( void );

void wallOutCorrection( void );

void setControlWallPD( int8_t _able );
float wallSidePD( float kp, float kd, float maxim );

float updateVelocityAccele( float measured );
float updateAngularAccele( void );

float PID( float target, float measurement, float *sum, float *old, float kp, 
                    float ki, float kd, float maxim );
                    
float PID2( float target, float measurement, float target2, float measurement2,  float *sum, 
            float *old, float *sum2, float kp, float ki, float kd, float ki2, float maxim );

void setControlFrontPD( int8_t _able );
float wallFrontPD( float kp, float kd, float maxim );
#endif /* __TARGETGENERATOR_H */