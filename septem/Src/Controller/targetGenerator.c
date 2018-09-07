#include "targetGenerator.h"
// config 
#include "config.h"
#include "global_var.h"

// controller
#include "trackMotion.h"
#include "motion.h"
#include "PIDController.h"



// motion 用変数
static float v = 0.0f;                 // 速度
static float v_previous = 0.0f;        // 一つ前の速度
static float omega = 0.0f;             // 角速度
static float distance = 0.0f;          // 距離
static float rad = 0.0f;               // 角度

static float motion_distance = 0.0f;   // 今のモーションの距離を格納する変数

// 制御の目標値　設定用変数
static float feedfoward_acccele = 0.0f;         // 速度のフィードフォワード

// 速度のPID用変数
static float v_sum = 0.0f;
static float v_old = 0.0f;

// speed Gain
// 直線用
static float speed_p = 0.0f;
static float speed_i = 0.0f;

// 超信地旋回用
static float speed_turn_p = 0.0f;
static float speed_turn_d = 0.0f;

// 角度のPID用変数
static float gyro_sum = 0.0f;
static float gyro_old = 0.0f;
// yawrate gain
// 直線用
static float gyro_p = 0.0f;
static float gyro_d = 0.0f;

// turn,slarom 用
static float gyro_turn_p = 0.0f;
static float gyro_turn_i = 0.0f;
static float gyro_turn_d = 0.0f;

// 角度指定用の変数
static float rad_target = 0.0f;

// side sensor の pd 用 
static int8_t ctr_sidewall_flag = 0;
static int16_t sensor_error_before = 0.0f;

// wall side pd gain
static float wall_p = 0.0f;
static float wall_i = 0.0f;

// maze update flag 
static int8_t maze_wall_update_flag = 0;
static int8_t maze_update_flag = 0;

void setSearchGain( void )
{
  // speed Gain
  // 直線用
  speed_p = 1.5f;
  speed_i = 0.7f;

  // 超信地旋回用
  speed_turn_p = 0.45f;
  speed_turn_d = 0.1f;

  // yawrate gain
  // gyro 直線用
  gyro_p = 0.30f;
  gyro_d = 0.1f;

  // gyro turn,slarom 用
  gyro_turn_p = 5.0f;
  gyro_turn_i = 0.9f;
  gyro_turn_d = 25.0f;

  // wall side pd gain
  wall_p = 0.35f;
  wall_i = 0.1f;
}

// 距離、角度などモーションに必要なものを更新しておく
void resetMotion( void )
{
  // 距離角度をリセット
  distance = 0.0f;
  rad = 0.0f;
  // pid 関連の偏差の積、偏差の値をリセット
  v_sum = 0.0f, 
  v_old = 0.0f;

  // pid gain reset
  gyro_sum = 0.0f;
  gyro_old = 0.0f;

}

void resetRadParam( void )
{
  // global 変数の machine rad を reset
  machine_rad = 0.0f;

  // 角度指定用の変数 
  rad_target = 0.0f;

  // pid gain reset
  gyro_sum = 0.0f;
  gyro_old = 0.0f;

}

void setMotionDistance( float _L_motion )
{
  motion_distance = _L_motion;
}

// velocity　の目標値を更新 
void updateTargetVelocity( void )
{

  v = speedNext( distance );  // 速度を取得

  if ( checkNowMotion() == turn ){
    omega = yawrateNext( rad );   // 角速度を取得する
  } else if ( checkNowMotion() == slarom ){
    omega = slaromNext( distance, rad );
  } else {
    omega = 0.0f;
  }

  // 距離に関して最短走行のときは理論値ではなく実走行距離を目標値に代入にしたいなあって。
  distance += v * dt;     // 理論値から距離を積算する
  
  rad += omega * dt;      // 理論値から角度を積算する
  

  rad_target += omega * dt;   // 角度の目標値毎回リセットしないため積算をし続けておく

  // FF 制御 ( 今の値 - 一つ前の値 ) ( /dt) 
  // 先に加速度をかけておく
  feedfoward_acccele = ( v - v_previous ); 

  // 今の値を保存
  v_previous = v;           

  // to do 壁切れ補正
  
  //wallOutCorrection();

  // to do 迷路の更新タイミングを教える
  if ( maze_wall_update_flag == 1 && distance >= motion_distance - 10.0f ){
    maze_update_flag = 1;
    maze_wall_update_flag = 0;
  }
}

void wallOutStraightCorrection( void )
{
  if ( checkNowMotion() == straight && motion_distance == 180.0f ){
    // to do 左壁を読んだら左の壁切れをチェックするようになる
    // to do 右側も同様に
    // to do きれたら distance = 90.0mm に固定
  }
}

void setMazeWallUpdate( int8_t _able )
{
  maze_wall_update_flag = _able;
}

void certainMazeUpdateFlag( void )
{
  maze_update_flag = 0;
}

int8_t checkMazeUpdateFlag( void )
{
  return maze_update_flag;
}

void setControlWallPD( int8_t _able )
{
  ctr_sidewall_flag = _able;
}

float wallSidePD( float kp, float kd, float maxim )
{
  int16_t error = 0;
  int16_t error_buff = 0;
  float p,d;

  // sensor の状態によって偏差の取り方を変える
  if ( sensor_sidel.is_wall == 1 && sensor_sider.is_wall == 1 ){
    error = sensor_sider.error - sensor_sidel.error;
  } else if ( sensor_sidel.is_wall == 1 ){
    error = -2 * sensor_sidel.error;
  } else if ( sensor_sider.is_wall == 1 ){
    error = 2 * sensor_sider.error;
  } else {
    error = 0;
  }

  error_buff = error;

  if ( ( (error - sensor_error_before) > 300 ) || ( ( error - sensor_error_before ) < -300 ) ){
    // to do flag 一度制御をきる
    error = 0;
  } 

  sensor_error_before = error_buff;

  p = (float)( kp * error );
  d = (float)( ( error - sensor_error_before) * kd );
  
  if ( (p + d) > maxim ) {
    p = maxim;
    d = 0.0f;
  }

  if ( (p + d) < -maxim ) {
    p = -maxim;
    d = 0.0f;
  }

  if ( ctr_sidewall_flag == 1 ){
    return (p + d);
  } else {
    return 0.0f;
  }
  

}

float updateVelocityAccele( float measured )
{
  float velocity_accele = 0.0f;   // 加速度
  float feedback_accele = 0.0f;   // フィードバック

  // 超信地旋回のときはゲインを変更
  if ( checkNowMotion() == turn ){
    feedback_accele = PID( 0.0f, measured, &v_sum, &v_old, speed_turn_p, 0.0f, speed_turn_d, 15.0f );
  } else {
    feedback_accele = PID( v, measured, &v_sum, &v_old, speed_p, speed_i, 0.0f, 50.0f );
  }

  log_v = (int16_t)measured;
  log_v_target = (int16_t)v;

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
  float feedback_angular_accele = 0.0f; //角度のフィードバック
  float feedback_wall = 0.0f;

  // 直進と直進以外でゲインを変化させる
  if ( checkNowMotion() == straight ){
    feedback_angular_accele = PID( omega, gyro_z_measured, &gyro_sum, &gyro_old, gyro_p, 0.0f, gyro_d, 15.0f );
    feedback_wall = wallSidePD( wall_p, wall_i, 15.0f );
  } else {
    feedback_angular_accele = PID( rad_target, machine_rad, &gyro_sum, &gyro_old, gyro_turn_p, gyro_turn_i, gyro_turn_d, 50.0f );
  }

  log_omega = (int16_t)gyro_z_measured;
  log_rad = (int16_t)machine_rad;
  log_rad_target = (int16_t)rad_target; 
  
  if ( checkNowMotion() == no_control || checkNowMotion() == delay ){
    angular_accele = 0.0f;
    return angular_accele;
  } else {
    angular_accele = feedback_angular_accele + feedback_wall;
    return angular_accele;
  }

}
