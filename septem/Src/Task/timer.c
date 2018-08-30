#include "timer.h"

#include "spi.h"
#include "tim.h"

#include "buzzer.h"

#include "motorController.h"

static Velocity v;

void interrupt()
{
  if ( MPU6500_calc_check() == 0 ) {
    MPU6500_z_axis_offset_calc();
  } else {
    gyro_z_measured = MPU6500_read_gyro_z();
    machineRadCalculation( gyro_z_measured );
  }

  // エンコーダの値の取得
  update_encoder();

  // エンコーダから速度を計算
  v = updateMotorData();

  // To do motionのアップデート 

  // To do 目標値の値を得る
  
  // To do モーターに与える加速度を決定する
  
  // モーターの出力を行う
  //updateMotorDuty();

  buzzerOutPut();
}

//*********************************************************************
// debug
//*********************************************************************
Velocity checkVelocity( void )
{
  return v;
}