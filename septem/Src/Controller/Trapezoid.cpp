#include "Trapezoid.h"

//#include <cstdio>

Trapezoid::Trapezoid( float _T, float _L, float _accele, float _v_target, float _v_start, float _v_end )
:T(_T), L(_L),accele(_accele), v_target(_v_target), v_start(_v_start), v_end(_v_end)
{

  odo = new Odometry( T );
  odo->reset();

  // a = m / s^2 
  // v = m / s 
  // x = mm

  float _t_start = 0.0f;
  float _t_end = 0.0f;

  _t_start = ( v_target - v_start ) / ( accele * T );
  _t_end = ( v_target - v_end ) / ( accele * T );

  accele_distance = ( v_target - v_start ) * _t_start * 0.5 * T + v_start * _t_start * T;

  decele_distance = ( v_target - v_end ) * _t_end * 0.5 * T + v_end * _t_end * T;

  if ( L >= 0 ){
    if ( accele_distance + decele_distance > _L ){
      accele_distance = L - decele_distance;
      constant_distance = accele_distance;
    } else {  
      constant_distance = L - decele_distance;
    }
  } else {
    if ( accele_distance + decele_distance < _L ){
      accele_distance = L - decele_distance;
      constant_distance = accele_distance;
    } else {  
      constant_distance = L - decele_distance;
    }
  }


  decele_distance = L;

  //std::printf( "距離, 加速, 一定速度, 減速,%f,%f,%f,%f\r\n",L, accele_distance, constant_distance, decele_distance );

  reset();
  
}

Trapezoid::~Trapezoid()
{
  delete odo;
}

float Trapezoid::next_v()
{
  Position now_pos = odo->get_pos();

  if ( L > 0.0f ){
    if ( now_pos.distance <= accele_distance ){
      v += accele * dt; 
    } else if ( now_pos.distance <= constant_distance ){
      v = v_target;
    } else if ( now_pos.distance <= decele_distance && v > 0.0f ){
      v -= accele * dt;
    } else {
      v = v_end;
      end = true;
    }
  } else {
    if ( now_pos.distance >= accele_distance ){
      v += accele * dt; 
    } else if ( now_pos.distance >= constant_distance ){
      v = v_target;
    } else if ( now_pos.distance >= decele_distance && v < 0.0f ){
      v -= accele * dt;
    } else {
      v = v_end;
      end = true;
    }
  }

  return v;
}

float Trapezoid::next_rad()
{
  Position now_pos = odo->get_pos();

  if ( L > 0.0f){
    if ( now_pos.rad <= accele_distance ){
      v += accele * dt; 
    } else if ( now_pos.rad <= constant_distance ){
      v = v_target;
    } else if ( now_pos.rad <= decele_distance && v > 0.0f ){
      v -= accele * dt;
    } else {
      v = v_end;
      end = true;
    }
  } else {
    if ( now_pos.rad >= accele_distance ){
      v += accele * dt; 
    } else if ( now_pos.rad >= constant_distance ){
      v = v_target;
    } else if ( now_pos.rad >= decele_distance && v < 0.0f ){
      v -= accele * dt;
    } else {
      v = v_end;
      end = true;
    }  
  }

  return v;
}

void Trapezoid::reset()
{
  v = 0.0f;
	end = false;
}