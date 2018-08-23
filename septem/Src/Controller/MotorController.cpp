#include "MotorController.h"

#include "stm32f4xx_hal.h"  // it is need to use arm_math.h
#include "arm_math.h"

#include "tim.h"

#include <cstdio>

MotorController::MotorController()
{
  vectoryConst = MACHINE_WEIGHT * MACHINE_WHEEL_RADIUS / 2.0f;

  angleConst = INERTIAL_MOMENT / MACHINE_TREAD_WIDTH;

  motorRpmConst = 60.0f * GEAR_RATION / 2.0f * PI * MACHINE_WHEEL_RADIUS;

  voltageTorqueConst = MOTOR_RESISTOR / MOTOR_TORQUE_CONSTANT; 

}

Velocity MotorController::updateMotorData( const EncValue& enc_value )
{
  float enc_left_omega, enc_right_omega;
  Velocity v;

  enc_left_omega = (float)enc_value.left / MACHINE_ENC_CNT_PER_ROT * 2.0f * PI / dt ;
  enc_right_omega = (float)enc_value.right / MACHINE_ENC_CNT_PER_ROT * 2.0f * PI / dt;

  v.v =  (enc_left_omega + enc_right_omega) / 2.0f * MACHINE_WHEEL_RADIUS;
	v.omega = (-enc_left_omega + enc_right_omega) / 2.0f * MACHINE_WHEEL_RADIUS / MACHINE_TREAD_WIDTH;

  // 左右のタイヤの角速度を計算 ( 実際の制御では gyro sensor の値を使う )
  //speed.left_omega = ( speed.measument - speed.omega * MACHINE_TREAD_WIDTH ) / MACHINE_WHEEL_RADIUS;
  //speed.right_omega = ( speed.measument + speed.omega * MACHINE_TREAD_WIDTH ) / MACHINE_WHEEL_RADIUS;

  motor_rpm = v.v * motorRpmConst;

  return v;
}

void MotorController::updateMotorDuty()
{
  float accele = 0.0f;
  float angular_accele = 0.0f;
  float duty_left_buff = 0.0f;
  float duty_right_buff = 0.0f;
  int32_t duty_left = 0;
  int32_t duty_right = 0;

  //  * accele
  torque_vectory = accele * vectoryConst;
  torque_angle =  angular_accele * angleConst;

  torque_left = ( torque_vectory - torque_angle ) / GEAR_RATION;
  torque_right = ( torque_vectory + torque_angle ) / GEAR_RATION;

  duty_left_buff = ( voltageTorqueConst * torque_left + MOTOR_REVERSE_VOLTAGE_CONSTANT * motor_rpm ) / batt_monitor;
  duty_right_buff = ( voltageTorqueConst * torque_right + MOTOR_REVERSE_VOLTAGE_CONSTANT * motor_rpm ) / batt_monitor;

  if ( duty_left_buff > 0.99f ) duty_left_buff = 0.99f;
  if ( duty_left_buff < -0.99f ) duty_left_buff = -0.99f;

  if ( duty_right_buff > 0.99f ) duty_right_buff = 0.99f;
  if ( duty_right_buff < -0.99f ) duty_right_buff = -0.99f;
   

  duty_left_buff *= MOTOR_CONTROL_PERIOD;
  duty_right_buff *= MOTOR_CONTROL_PERIOD;

  duty_left  = (int32_t)duty_left_buff;
  duty_right = (int32_t)duty_right_buff;

  motorControl( duty_left, duty_right );
  
}

void MotorController::checkConst()
{
  std::printf( "\r\n vectoryconst, angleconst, motorrpmconst, voltagetorqueconst, %5.5f,%5.5f, %5.5f, %5.5f\r\n",
          vectoryConst, angleConst, motorRpmConst, voltageTorqueConst );
}