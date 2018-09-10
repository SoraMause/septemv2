#include "motion.h"

#include "targetGenerator.h"

#include "config.h"

//#include <stdio.h>

// 共用
static float accele_L = 0.0f;           // 加速距離
static float decele_L = 0.0f;           // 減速距離
static float constant_L = 0.0f;         // 一定速度区間の距離
static float ax = 0.0f;                 // 加速度
static float target_L = 0.0f;           // 目標距離 

// speed trape
static uint8_t speed_trape_flag = 1;    // 終了確認用フラグ( speed trape )
static float v_target = 0.0f;           // 目標速度
static float v_end = 0.0f;              // 終了速度

static float v  = 0.0f;                 // 速度( speed next　)の返り値

// yawrate trape
static uint8_t yawrate_trape_flag = 1;  // 終了確認用フラグ( yawrate trape )
static float omega_target = 0.0f;       // 目標速度
static float omega_end = 0.0f;          // 終了速度

static float omega = 0.0f;              // 角速度( yawrate next )の返り値

// slarom 
static uint8_t slarom_flag = 1;         // 終了確認用フラグ( slarom )
static float before_L = 0.0f;           // スラロームの曲がる前のオフセット走行距離
static float after_L = 0.0f;            // スラロームの終了距離
static float slarom_trape_L = 0.0f;     // スラロームの曲がるのに必要な距離

// delay motion
static uint16_t cnt_delay = 0;

// 終了フラグ
static uint8_t end_flag = 0;

// 台形加速( 速度 )
void speedTrapezoid( float L ,float accele, float target, float start, float end )
{
  // accele [m / ss]
  // target, start, end  [mm/s]
  // L = [mm]
  
  float _t_start, _t_end;

  _t_start = ( target - start ) / accele;
  _t_end = ( target - end ) / accele;

  accele_L = ( target - start ) * _t_start * 0.5 * dt + start * _t_start * dt;

  decele_L = ( target - end ) * _t_end * 0.5 * dt + end * _t_end * dt;

  constant_L = L - decele_L;

  decele_L = L;

  v_target = target;  // 目標値セット
  v_end = end;        // 終了値セット
  target_L = L;       // 距離をセット
  ax = accele;  // 加速度をセット
  speed_trape_flag = 0;
  end_flag = 0;

  resetMotion();
  setMotionDistance( L );

  //printf( "セクションの時間　加速, 減速, %f, %f \r\n",_t_start, _t_end );
  //printf( "距離, 加速, 一定速度, 減速,%f,%f,%f,%f\r\n",L, accele_L, constant_L, decele_L );

}
// 台形加速の次の速度を返す
float speedNext( float distance )
{
  if ( speed_trape_flag == 0 ){
    if ( target_L > 0 ){
      if ( distance < accele_L ){
        v += ax;
      } else if ( distance < constant_L ){
        v = v_target;
      } else if ( distance < decele_L && v > 0.0f ){
        v -= ax;
      } else {
        v = v_end;
        speed_trape_flag = 1;
        end_flag = 1;
      }
    } else {
      if ( distance > accele_L ){
        v += ax;
      } else if ( distance > constant_L ){
        v = v_target;
      } else if ( distance > decele_L && v < 0.0f ){
        v -= ax;
      } else {
        v = v_end;
        speed_trape_flag = 1;
        end_flag = 1;
      }
    }
  } else {
    v = v_end;
  }

  return v;

}
// 台形加速( 角速度 )
void yawrateTrapezoid( float L ,float accele, float target )
{
  // accele [rad / ss]
  // target, start, end  [rad/s]
  // L [rad]
  
  float _t_start_end;

  _t_start_end = target / ( accele * dt );

  accele_L = target * _t_start_end * 0.5 * dt;

  decele_L = accele_L;

  constant_L = L - decele_L;

  decele_L = L;

  omega_target = target;  // 目標値セット
  omega_end = 0.0f;        // 終了値セット
  target_L = L;       // 距離をセット
  ax = accele;  // 加速度をセット
  yawrate_trape_flag = 0;
  end_flag = 0;

  resetMotion();

  //printf( "セクションの時間　加速, 減速, %f, %f \r\n",_t_start_end, _t_start_end );
  //printf( "角度, 加速, 一定速度, 減速,%f,%f,%f,%f\r\n",L, accele_L, constant_L, decele_L );

}
// 台形加速の次の角速度を返す
float yawrateNext( float rad )
{
  if ( yawrate_trape_flag == 0 ){
    if ( target_L > 0 ){
      if ( rad < accele_L ){
        omega += ax * dt;
      } else if ( rad < constant_L ){
        omega = omega_target;
      } else if ( rad < decele_L && omega > 0.0f ){
        omega -= ax * dt;
      } else {
        omega = omega_end;
        yawrate_trape_flag = 1;
        end_flag = 1;
      }
    } else {
      if ( rad > accele_L ){
        omega += ax * dt;
      } else if ( rad > constant_L ){
        omega = omega_target;
      } else if ( rad > decele_L && omega < 0.0f ){
        omega -= ax * dt;
      } else {
        omega = omega_end;
        yawrate_trape_flag = 1;
        end_flag = 1;
      }
    }
  } else {
    omega = omega_end;
  }

  return omega;

}

void setSlarom( float L, float accele, float rad_target, float speed_target, float before_distance, float after_distance )
{
  // rad [rad]
  // accele [rad/ss]
  // speed_target [mm/s]
  // distance [mm]

  float _t_start_end, _t_constant;
  float yawrate_trape_time = 0.0f;
  float slarom_yawrate_trape_distance = 0.0f;
  float slarom_distance = 0.0f;

  _t_start_end = rad_target / ( accele * dt );

  accele_L = rad_target * _t_start_end * 0.5 * dt;    // targetから加速に必要な角度をだす

  decele_L = accele_L;  // 台形加速の加速度が変わらないため計算をせずに代入でおしまい

  constant_L = L - decele_L;  // 一定角速度の終了角度

  _t_constant = ( constant_L - accele_L ) / rad_target / dt;  //　一定加速度の時間を算出

  decele_L = L;  // 終了角度を決めておく

  yawrate_trape_time = _t_constant + 2.0f * _t_start_end;  // 角速度の台形加速に必要な時間を算出

  slarom_yawrate_trape_distance = speed_target * yawrate_trape_time * dt; // ターンの距離

  slarom_trape_L = before_distance + slarom_yawrate_trape_distance;        // スラロームの曲がり終わるまでの距離

  slarom_distance = slarom_trape_L + after_distance; // スラローム全体の移動距離

  before_L = before_distance;   // スラロームの前距離
  
  after_L = slarom_distance;    // スラロームの終了距離

  ax = accele;                  // 角加速度を代入 

  target_L = L;               // 追従目標角度を追加
  omega_target = rad_target;    // 目標値を代入

  slarom_flag = 0;              // slarom の終了フラグを0にする             

  end_flag = 0;

  resetMotion();
  setMotionDistance( slarom_distance );

  //printf( "時間　: 角加速全体 %f, 加減速: %f\r\n",yawrate_trape_time, _t_start_end *2 );
  //printf( "オフセット走行距離 前　: %f, 後ろ : %f\r\n",before_distance, after_distance );
  //printf( "全体の走行距離 : %f, ターンの走行距離 : %f\r\n",after_L, slarom_yawrate_trape_distance );

}

float slaromNext( float distance, float rad )
{
    
  if ( slarom_flag == 0 ){
    if ( distance < before_L ){
      omega = 0.0f;
    }else if ( distance < slarom_trape_L ){
      if ( target_L > 0 ){
        if ( rad < accele_L ){
            omega += ax * dt;
          } else if ( rad < constant_L ){
            omega = omega_target;
          } else if ( rad < decele_L && omega > 0.0f ){
            omega -= ax * dt;
          } else {
            omega = 0.0f;
          }
      } else if ( target_L < 0 ){
        if ( rad > accele_L ){
          omega += ax * dt;
        } else if ( rad > constant_L ){
          omega = omega_target;
        } else if ( rad > decele_L && omega < 0.0f ){
          omega -= ax * dt;
        } else {
          omega = 0.0f;
        }
      }
    } else if ( distance < after_L ){
      omega = 0.0f;
      setControlWallPD( 1 );
    } else {
      omega = 0.0f;
      slarom_flag = 1;
      end_flag = 1;
    }
  } else {
    omega = 0.0f;
  }

  return omega;

}

void motionDelay( void )
{
  if ( cnt_delay < DELAY_TIME ){
    end_flag = 0;
    cnt_delay++;
  } else {
    //printf( "end delay\r\n" );
    cnt_delay = 0;
    end_flag = 1;
  }
}

void setMotionEnd( int8_t _end )
{
  end_flag = _end;
}

int8_t checkEndMotion( void )
{
  return end_flag;
}