#include "PIDController.h"

PIDController::PIDController()
{

}

PIDController::PIDController( const PIDParam& _param )
  :param( _param )
{

}

void PIDController::set_param(const PIDParam& _param) 
{ 
  param = _param; 
}

float PIDController::update( float measured, float target )
{
  float p,i,d;
  const float error = target - measured;
  float error_diff = error - prev_error;

  if ( is_first ){
    is_first = false;
    error_diff = 0.0f;
  }

  // 積分の値の計算
  error_sum = error_sum + error * param.T;
  
  // 偏差の積分は飽和させる
	if (error_sum > param.saturation) error_sum = param.saturation;
	else if (error_sum < -param.saturation) error_sum = param.saturation;

  p = param.kp * error; 
  d = param.kd * error_diff; 
  i = param.ki * error_sum;
  
  prev_error = error;

  if ( (p+i+d) > param.max ){
    p = param.max;
    d = 0.0f;
    i = 0.0f;
  }

  if ( (p+i+d) < -param.max ){
    p = -param.max;
    d = 0.0f;
    i = 0.0f;
  }

  return (p+i+d);
}

void PIDController::reset()
{
	error_sum = 0.0f;
	prev_error = 0.0f;
	is_first = true;
}