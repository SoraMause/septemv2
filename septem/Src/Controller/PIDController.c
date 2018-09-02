#include "PIDController.h"

///////////////////////////////////////////////////////////////////////
// PID
// [argument] (float)target,measurement,*sum,*old,kp,ki,kd,max,min
// [Substitutiong] nothing
// [return] p + i + d
// [contents] caluculate the pid controller ( feedback )
///////////////////////////////////////////////////////////////////////
float PID( float target, float measurement, float *sum, float *old, float kp, 
                    float ki, float kd, float maxim )
{
  float p, i, d, error, sum2;

  sum2 = *sum;

  error = target - measurement;

  p = error * kp;

  *sum += error * dt; 
  i = *sum * ki;

  d =  ( measurement - *old ) * kd; 
  *old = measurement;

  // リセットワインドアップ対策
  if( (p+i-d) > maxim ){
    p = maxim;
    i = 0.0f;
    d = 0.0f;
    *sum = sum2;
  }

  if( (p+i-d) < -maxim ){
    p = -maxim;
    i = 0.0f;
    d = 0.0f;
    *sum = sum2;
  }

  return ( p+i-d );
}
