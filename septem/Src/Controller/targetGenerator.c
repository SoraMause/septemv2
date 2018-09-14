#include "targetGenerator.h"

#include "config.h"

#include "trackMotion.h"

#include "motion.h"

#include "global_var.h"

#include "buzzer.h"

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
static float gyro_sum2 = 0.0f;

// turn,slarom 用
static float gyro_turn_p = 0.0f;
static float gyro_turn_i = 0.0f;
static float gyro_turn_d = 0.0f;
static float gyro_turn_i2 = 0.0f;

// 角度指定用の変数
static float rad_target = 0.0f;

// side sensor の pd 用 
static int8_t ctr_sidewall_flag = 0;
static int16_t sensor_error_before = 0.0f;

// wall side pd gain
static float wall_p = 0.0f;
static float wall_d = 0.0f;

// maze update flag 
static int8_t maze_wall_update_flag = 0;
static int8_t maze_update_flag = 0;

// 壁フラグ関連
static int8_t left_check_flag = 0;
static int8_t right_check_flag = 0;
static int8_t wall_out_flag = 0;

void setSearchGain( void )
{
  // speed Gain
  // 直線用
  speed_p = 150.0f;
  speed_i = 50.0f;

  // 超信地旋回用
  speed_turn_p = 30.0f;
  speed_turn_d = 7.0f;

  // gyro turn,slarom 用
  //gyro_turn_p = 35.0f;    // 宴会芸
  //gyro_turn_i = 0.5f;     // 宴会芸
  //gyro_turn_d = 60.0f;    // 宴会芸
  // gyro turn,slarom 用
  gyro_turn_p = 16.0f;
  gyro_turn_i = 400.0f;
  gyro_turn_i2 = 500.0f;
  gyro_turn_d = 70.0f;

  // wall side pd gain
  wall_p = 40.0f;
  wall_d = 15.0f;
}

void setFastGain( void )
{
  // speed Gain
  // 直線用
  speed_p = 150.0f;
  speed_i = 50.0f;

  // 超信地旋回用
  speed_turn_p = 30.0f;
  speed_turn_d = 7.0f;

  // gyro turn,slarom 用
  gyro_turn_p = 15.0f;
  gyro_turn_i = 250.0f;
  gyro_turn_i2 = 400.0f;
  gyro_turn_d = 90.0f;

  // wall side pd gain
  wall_p = 20.0f;
  wall_d = 10.0f;
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

  left_check_flag = 0;
  right_check_flag = 0;
  wall_out_flag = 0;

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
  gyro_sum2 = 0.0f;

}

void setMotionDistance( float _L_motion )
{
  motion_distance = _L_motion;
}

// velocity　の目標値を更新 
void updateFastRunTargetVelocity( float measurement )
{

  v = speedNext( distance );  // 速度を取得

  if ( checkNowMotion() == turn ){
    omega = yawrateNext( rad );   // 角速度を取得する
  } else if ( checkNowMotion() == slarom ){
    omega = slaromNext( distance, rad );
  } else {
    omega = 0.0f;
  } 

  distance += measurement * dt;     // 理論値から距離を積算する
  rad += omega * dt;      // 理論値から角度を積算する

  rad_target += omega * dt;   // 角度の目標値毎回リセットしないため積算をし続けておく

  // FF 制御 ( 今の値 - 一つ前の値 ) ( /dt) 
  // 先に加速度をかけておく
  feedfoward_acccele = ( v - v_previous ); 

  // 今の値を保存
  v_previous = v;           

}

// velocity　の目標値を更新 
void updateSearchTargetVelocity( void )
{

  v = speedNext( distance );  // 速度を取得

  if ( checkNowMotion() == turn ){
    omega = yawrateNext( rad );   // 角速度を取得する
  } else if ( checkNowMotion() == slarom ){
    omega = slaromNext( distance, rad );
  } else {
    omega = 0.0f;
  } 

  distance += v * dt;     // 理論値から距離を積算する
  rad += omega * dt;      // 理論値から角度を積算する

  rad_target += omega * dt;   // 角度の目標値毎回リセットしないため積算をし続けておく

  // FF 制御 ( 今の値 - 一つ前の値 ) ( /dt) 
  // 先に加速度をかけておく
  feedfoward_acccele = ( v - v_previous ); 

  // 今の値を保存
  v_previous = v;           

  // to do 壁切れ補正
  wallOutCorrection();

  // to do 迷路の更新タイミングを教える
  if ( maze_wall_update_flag == 1 && distance >= motion_distance - 7.0f ){
    maze_update_flag = 1;
    maze_wall_update_flag = 0;
  }
}

void wallOutCorrection( void )
{
  if ( checkNowMotion() == straight && motion_distance == 180.0f ){
    if ( wall_out_flag == 0 ){
      // to do 壁を読んだら左の壁切れをチェックするようになる
      if ( sensor_sidel.is_wall == 1 ) left_check_flag = 1;
      if ( sensor_sider.is_wall == 1 ) right_check_flag = 1;
      // to do 切れた後の処理を入れること！
      if ( distance > 80.0f && distance < 100.0f ){

        if ( left_check_flag == 1 && sensor_sidel.is_wall == 0 ){
          wall_out_flag = 1; 
          distance = 92.0f;
          buzzerSetMonophonic( C_SCALE, 100 );
        }

        if ( right_check_flag == 1 && sensor_sider.is_wall == 0 ){
          wall_out_flag = 1;
          distance = 92.0f;
          buzzerSetMonophonic( C_SCALE, 100 );
        }
      }
    }
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

float updateVelocityAccele( float measured )
{
  float velocity_accele = 0.0f;   // 加速度
  float feedback_accele = 0.0f;   // フィードバック

  // 超信地旋回のときはゲインを変更
  if ( checkNowMotion() == turn ){
    feedback_accele = PID( 0.0f, measured, &v_sum, &v_old, speed_turn_p, 0.0f, speed_turn_d, 4000.0f );
  } else {
    feedback_accele = PID( v, measured, &v_sum, &v_old, speed_p, speed_i, 0.0f, 12000.0f );
  }

  log_v = (int16_t)measured;
  log_v_target = (int16_t)v;
  log_distance = distance;

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

#if 0
  if ( checkNowMotion() == straight ){
    feedback_angular_accele = PID( 0.0f, gyro_z_measured, &gyro_sum, &gyro_old, gyro_p, 0.0f, gyro_d, 3000.0f );
    feedback_wall = wallSidePD( wall_p, wall_d, 3000.0f );
  } else {
    feedback_angular_accele = PID2( omega, gyro_z_measured, rad_target, machine_rad, &gyro_sum, &gyro_old,&gyro_sum2, 
                                    gyro_turn_p, gyro_turn_i, gyro_turn_d,gyro_turn_i2, 10000.0f );
  }
#endif

  feedback_angular_accele = PID2( omega, gyro_z_measured, rad_target, machine_rad, &gyro_sum, &gyro_old,&gyro_sum2, 
                                gyro_turn_p, gyro_turn_i, gyro_turn_d,gyro_turn_i2, 10000.0f );
  feedback_wall = wallSidePD( wall_p, wall_d, 3000.0f );

  log_omega_target = (int16_t)omega;
  log_omega = (int16_t)gyro_z_measured;
  log_rad = (int16_t)machine_rad;
  
  if ( checkNowMotion() == no_control || checkNowMotion() == delay ){
    angular_accele = 0.0f;
    return angular_accele;
  } else {
    angular_accele = feedback_angular_accele + feedback_wall;
    return angular_accele;
  }

}

float PID( float target, float measurement, float *sum, float *old, float kp, 
                    float ki, float kd, float maxim )
{
  float p, i, d, error, sum2;

  sum2 = *sum;

  error = target - measurement;

  p = error * kp;

  *sum += error * dt; 
  i = *sum * ki;

  d =  ( measurement - *old ) * kd; 
  *old = measurement;

  // リセットワインドアップ対策
  if( (p+i+d) > maxim ){
    p = maxim;
    i = 0.0f;
    d = 0.0f;
    *sum = sum2;
  }

  if( (p+i+d) < -maxim ){
    p = -maxim;
    i = 0.0f;
    d = 0.0f;
    *sum = sum2;
  }

  return ( p+i+d );
}

float PID2( float target, float measurement, float target2, float measurement2,  float *sum, 
            float *old, float *sum2, float kp, float ki, float kd, float ki2, float maxim )
{
  float p, i, i2, d, error, sum_buff;

  sum_buff = *sum;

  error = target - measurement;

  p = error * kp;

  *sum += error * dt; 
  i = *sum * ki;

  *sum2 += ( target2 - measurement2 ) * dt;
  i2 = *sum2 * ki2;

  d =  ( measurement - *old ) * kd; 
  *old = measurement;

  // リセットワインドアップ対策
  if( (p+i+d+i2) > maxim ){
    p = maxim;
    i = 0.0f;
    i2 = 0.0f;
    d = 0.0f;
    *sum = sum_buff;
  }

  if( (p+i+d+i2) < -maxim ){
    p = -maxim;
    i = 0.0f;
    i2 = 0.0f;
    d = 0.0f;
    *sum = sum_buff;
  }

  return ( p+i+d+i2 );
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
  
  if ( ( (error - sensor_error_before) > 20 ) || ( ( error - sensor_error_before ) < -20 ) ){
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