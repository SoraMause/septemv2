#include "PIDController.h"

///////////////////////////////////////////////////////////////////////
// PID
// [argument] (float)target,measurement,*sum,*old,kp,ki,kd,max,min
// [Substitutiong] nothing
// [return] p + i + d
// [contents] caluculate the pid controller ( feedback )
///////////////////////////////////////////////////////////////////////
float PID( float target, float measurement, float *sum, float *old, float kp, 
                    float ki, float kd, float maxim, float minmam )
{
  float p, i, d, error, sum2;

  sum2 = *sum;

  error = target - measurement;

  p = error * kp;

  *sum += error * dt; 
    i = *sum * ki;

  d =  ( error - *old ) / dt * kd; 
  *old = error;

  // リセットワインドアップ対策
  if( maxim < (p+i+d) ){
    p = maxim;
    i = 0.0f;
    d = 0.0f;
    *sum = sum2;
  }

  if( (p+i+d) < minmam ){
    p = minmam;
    i = 0.0f;
    d = 0.0f;
    *sum = sum2;
  }

  return ( p+i+d );
}

void PIDReset( float *sum, float *old )
{
  *sum = 0.0f;
  *old = 0.0f;
}