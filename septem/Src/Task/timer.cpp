#include "timer.h"

#include "spi.h"
#include "tim.h"

#include "buzzer.h"

#include "MotorController.h"

EncValue enc_value;
Velocity v;

MotorController *motor_controller = new MotorController();

void interrupt()
{
  if ( MPU6500_calc_check() == 0 ) {
    MPU6500_z_axis_offset_calc();
  } else {
    gyro_z_measured = MPU6500_read_gyro_z();
    machineRadCalculation( gyro_z_measured );
  }

  // エンコーダの値の取得
  enc_value = update_encoder();

  // エンコーダから速度を計算
  v = motor_controller->updateMotorData( enc_value );

  // To do motionのアップデート 
  // ( To do motion push, pop )
  // To do 目標値の値を得る
  // To do モーターに与える加速度を決定する
  
  // モーターの出力を行う
  motor_controller->updateMotorDuty();

  buzzerOutPut();
}

//*********************************************************************
// debug
//*********************************************************************
void motorControllercheckConst()
{
  motor_controller->checkConst();
}

EncValue checkEncValue()
{
  return enc_value;
}

Velocity checkVelocity()
{
  return v;
}