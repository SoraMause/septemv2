#ifndef __TRAPEZOID_H
#define __TRAPEZOID_H

#include "Odometry.h"
#include "Geometry.h"
#include "config.h"

class Trapezoid {
public:
  Trapezoid( float _T, float _L, float _accele, float _v_target, float _v_start, float _v_end );
  ~Trapezoid();

public:
  float next_v();
	float next_rad();
  void reset();
  inline bool is_end() const { return end; }
	inline Odometry& get_odo() { return *odo; }
	inline float get_v_end() const { return v_end; }

	Odometry *odo;	// オドメトリクラスを使用

private:
	const float T;
	const float L;
	const float accele;
	float v_target;
	const float v_start;
	const float v_end;
	float ax;

	float accele_distance;
	float constant_distance;
	float decele_distance;
	float v;

	bool end;

};

#endif /* __TRAPEZOID_H */