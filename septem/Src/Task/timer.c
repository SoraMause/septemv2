#include "timer.h"

#include "spi.h"
#include "tim.h"

#include "config.h" 

#include "buzzer.h"

#include "trackMotion.h"

#include "targetGenerator.h"

static Velocity v;
static MotorDuty duty;
static float gyro_z_before = 0.0f;
static uint8_t controll_flag = 0;

void interrupt()
{
  if ( MPU6500_calc_check() == 0 ) {
    MPU6500_z_axis_offset_calc();
  } else {
    gyro_z_measured = MPU6500_read_gyro_z();
    machineRadCalculation();
  }

  // エンコーダの値の取得
  update_encoder();

  // エンコーダから速度を計算
  v = updateMotorData();

  if ( controll_flag == 1 && checkUpdateMotionEnd() == 0 ){
    // To do motionのアップデート 
    updateMotion();

    // To do 目標値の値を得る
    updateTargetVelocity();
    
    // モーターの出力を行う
    duty = updateMotorDuty();

    motorControl( duty.left, duty.right );
  } else {
    motorControl( 0, 0 );
  }


  buzzerOutPut();
}

void setControl( int8_t _in )
{
  controll_flag = _in;
}

///////////////////////////////////////////////////////////////////////
// machineRadCalculation
// [argument] nothing
// [Substitutiong] machine_rad
// [return] nothing
// [contents] caluculate the machine rad
///////////////////////////////////////////////////////////////////////
void machineRadCalculation( void )
{
  float gyro = 0.0f;
  gyro = gyro_z_measured;
  machine_rad += (float)( (gyro_z_before + gyro) * dt / 2.0f );
  gyro_z_before = gyro;
}

//*********************************************************************
// debug
//*********************************************************************
Velocity checkVelocity( void )
{
  return v;
}

MotorDuty checkMotorDuty( void )
{
  return duty;
}