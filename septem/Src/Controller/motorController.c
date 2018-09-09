#include "motorController.h"

#include "config.h"   // motor の定数を使用

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "tim.h"  // motorのdutyセット

#include "targetGenerator.h"

#include <stdio.h>

static float v = 0.0f;

///////////////////////////////////////////////////////////////////////
// calcMotorConst
// [argument] nothing
// [Substitutiong] nothing
// [return] nothing
// [contents] caluculate the motor data
///////////////////////////////////////////////////////////////////////
Velocity updateMotorData( void )
{
  float enc_left_omega, enc_right_omega;
  Velocity now;

  enc_left_omega = (float)enc_value.left / MACHINE_ENC_CNT_PER_ROT * 2.0f * PI / dt ;
  enc_right_omega = (float)enc_value.right / MACHINE_ENC_CNT_PER_ROT * 2.0f * PI / dt;

  now.v =  (enc_left_omega + enc_right_omega) / 2.0f / dt * MACHINE_WHEEL_RADIUS;
	now.omega = (-enc_left_omega + enc_right_omega) / 2.0f * MACHINE_WHEEL_RADIUS / MACHINE_TREAD_WIDTH;

  v = now.v;

  // 左右のタイヤの角速度を計算 ( 実際の制御では gyro sensor の値を使う )
  //speed.left_omega = ( speed.measument - speed.omega * MACHINE_TREAD_WIDTH ) / MACHINE_WHEEL_RADIUS;
  //speed.right_omega = ( speed.measument + speed.omega * MACHINE_TREAD_WIDTH ) / MACHINE_WHEEL_RADIUS;

  return now;
}

///////////////////////////////////////////////////////////////////////
// updateMotorDuty
// [argument] nothing
// [Substitutiong] nothing
// [return] nothing
// [contents] update motor duty
///////////////////////////////////////////////////////////////////////
MotorDuty updateMotorDuty( void )
{
  float velocity_duty = 0.0f;
  float angular_duty = 0.0f;
  float duty_left_buff = 0.0f;
  float duty_right_buff = 0.0f;

  MotorDuty duty;

  velocity_duty = updateVelocityAccele( v );
  angular_duty = updateAngularAccele();

  // 左側のトルクと右側のトルクをそれぞれ求める
  duty_left_buff = ( velocity_duty - angular_duty ) / batt_monitor;
  duty_right_buff = ( velocity_duty + angular_duty ) / batt_monitor;

  if ( duty_left_buff > 1599 ) duty_left_buff = 1599;
  if ( duty_right_buff > 1599 ) duty_right_buff = 1599;
  if ( duty_left_buff < -1599 ) duty_left_buff = -1599;
  if ( duty_right_buff < -1599 ) duty_right_buff = -1599;

  // 出力電圧を計算
  duty.left  = (int32_t)duty_left_buff;
  duty.right = (int32_t)duty_right_buff;

  return duty;
  
}