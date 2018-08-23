#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "Geometry.h"

class Odometry {
public:
  Odometry( float _T );

  ~Odometry(){}

  void update( const Velocity &v );

  void reset();

  void set_pos( const Position &_pos );

  inline const Position& get_pos() const { return pos; }

private:
  const float T;
  Position pos;

	//float prev_dx = 0.0f;
	//float prev_dy = 0.0f;
	float prev_omega = 0.0f;
	bool is_first = true;
};

#endif /* __ODOMETRY_H */