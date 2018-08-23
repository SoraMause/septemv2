#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

#include <cfloat>

#include "config.h"

struct PIDParam{
  float T = dt;
  float kp = 0.0f;
  float kd = 0.0f;
  float ki = 0.0f;
  float saturation = FLT_MAX;
  float max;
};

class PIDController {
public:
  PIDController();
  PIDController( const PIDParam& _param );

  virtual ~PIDController(){}

	void set_param(const PIDParam& _param);
	
  inline const PIDParam &get_param() { return param; }

  float update( float measured, float target );

  void reset();

private:
  // 設定パラメータ
  PIDParam param;

  // 状態の保持
  float error_sum = 0.0f;
  float prev_error = 0.0f;

  bool is_first = true;

};

#endif /* __PIDCONTROLLER_H */