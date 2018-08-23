#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H

#include "Geometry.h"
#include "config.h"
#include "global_var.h"

class MotorController {
public:
  MotorController();

  ~MotorController();

  // To do この関数に目標加速度を与える必要あり
  void updateMotorDuty();

  Velocity updateMotorData( const EncValue& enc_value );

  void checkConst();

private:

  float vectoryConst = 0.0f;
  float motorRpmConst = 0.0f;
  float angleConst = 0.0f;
  float voltageTorqueConst = 0.0f;
  float torque_vectory = 0.0f;
  float torque_angle = 0.0f;
  float torque_left = 0.0f;
  float torque_right = 0.0f;
  float motor_rpm = 0.0f;

};

#endif /* __MOTORCONTROLLER_H */