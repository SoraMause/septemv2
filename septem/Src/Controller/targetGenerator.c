#include "targetGenerator.h"

#include "config.h"

#include "trackMotion.h"

#include "motion.h"

#include "PIDController.h"

#include "global_var.h"

// motion 用変数
static float v = 0.0f;                 // 速度
static float v_previous = 0.0f;        // 一つ前の速度
static float omega = 0.0f;             // 角速度
static float omega_previous = 0.0f;    // 一つ前の角速度
static float distance = 0.0f;          // 距離
static float rad = 0.0f;               // 角度

// 制御の目標値　設定用変数
static float feedfoward_acccele = 0.0f;         // 速度のフィードフォワード
static float feedfoward_angular_accele = 0.0f;  // 角速度のフィードフォワード

// 速度のPID用変数
static float v_sum = 0.0f;
static float v_old = 0.0f;

// 角度のPID用変数
static float gyro_sum = 0.0f;
static float gyro_old = 0.0f;

// 距離、角度などモーションに必要なものを更新しておく
void resetMotion( void )
{
  // global 変数の machine rad を reset
  machine_rad = 0.0f;
  // 距離角度をリセット
  distance = 0.0f;
  rad = 0.0f;
  // pid 関連の偏差の積、偏差の値をリセット
  v_sum = 0.0f, 
  v_old = 0.0f;
  
  gyro_sum = 0.0f;
  gyro_old = 0.0f;

}

// velocity　の目標値を更新 
void updateTargetVelocity( void )
{

  v = 0.0f;
  omega = 0.0f;

  v = speedNext( distance );  // 速度を取得

  if ( checkNowMotion() == turn ){
    omega = yawrateNext( rad );   // 角速度を取得する
  } else if ( checkNowMotion() == slarom ){
    omega = slaromNext( distance, rad );
  }

  distance += v * dt;     // 理論値から距離を積算する
  rad += omega * dt;      // 理論値から角度を積算する

  // FF 制御 ( 今の値 - 一つ前の値 ) ( /dt) 
  // 先に加速度をかけておく
  feedfoward_acccele = ( v - v_previous ); 

  feedfoward_angular_accele = ( omega - omega_previous ) * 10.0f;

  // 今の値を保存
  v_previous = v;           
  omega_previous = omega;

  // to do 壁切れ補正
  //wallOutCorrection();

}

void wallOutCorrection( void )
{

}

float updateVelocityAccele( float measured )
{
  float velocity_accele = 0.0f;   // 加速度
  float feedback_accele = 0.0f;   // フィードバック

  // 超信地旋回のときはゲインを変更
  if ( checkNowMotion() == turn ){
    feedback_accele = PID( 0.0f, measured, &v_sum, &v_old, 0.3f, 0.0f, 0.1f, 15.0f );
  } else {
    feedback_accele = PID( v, measured, &v_sum, &v_old, 1.0f, 0.6f, 0.0f, 50.0f );
  }

  if ( checkNowMotion() == no_control ){
    velocity_accele = 0.0f;
    return velocity_accele;
  } else {
    velocity_accele = feedfoward_acccele + feedback_accele;
    return velocity_accele;
  }

}

float updateAngularAccele( void )
{
  float angular_accele = 0.0f;    // 角加速度
  float feedback_angular_accele = 0.0f; //フィードバック

  // 直進と直進以外でゲインを変化させる
  if ( checkNowMotion() == straight ){
    feedback_angular_accele = PID( 0.0f, gyro_z_measured, &gyro_sum, &gyro_old, 13.0f, 0.0f, 0.5f, 30.0f );
  } else {
    feedback_angular_accele = PID( rad, machine_rad, &gyro_sum, &gyro_old, 34.0f, 12.5f, 5.0f, 50.0f );
    //feedback_angular_accele = PID( omega, gyro_z_measured, &gyro_sum, &gyro_old, 35.0f, 1.0f, 1.0f, 30.0f );
  } 
  
  if ( checkNowMotion() == no_control || checkNowMotion() == delay ){
    angular_accele = 0.0f;
    return angular_accele;
  } else {
    angular_accele = feedfoward_angular_accele + feedback_angular_accele;
    return angular_accele;
  }

}
