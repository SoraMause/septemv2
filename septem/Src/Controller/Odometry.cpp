#include "stm32f4xx_hal.h"  // it is need to use arm_math.h
#include "arm_math.h"

#include "global_var.h"

#include "Odometry.h"
#include <cmath>


Odometry::Odometry( float _T )
:T(_T)
{

}

void Odometry::update( const Velocity &v )
{
  //if ( pos.theta > 2.0f * PI ) pos.theta -= 2.0f * PI;
  //else if ( pos.theta < -2.0f * PI ) pos.theta += 2.0f * 3.14;

  //const float dx = -arm_sin_f32( pos.theta ) * v.v;
  //const float dy = arm_cos_f32( pos.theta ) * v.v;

  if ( is_first ){
    //prev_dx = dx;
    //prev_dy = dy;
    prev_omega = v.omega;
    is_first = false;
  }

  // 台形積分
  pos.distance += v.v * T;
  pos.rad += ( v.omega + prev_omega)/2 * T;
  //pos.theta = pos.rad;
  //pos.x += ( dx + prev_dx)/2 * T;
  //pos.y += ( dx + prev_dy)/2 * T;
  

  //prev_dx = dx;
  //prev_dy = dy;
  prev_omega = v.omega;

}

void Odometry::reset()
{
  //pos.x = pos.y = 0.0f;
  //pos.theta = 0.0f;
  pos.distance = 0.0f;
  pos.rad = 0.0f;
  //prev_dx = prev_dy = 0.0f;
  prev_omega = 0.0f;
  is_first = true;
}