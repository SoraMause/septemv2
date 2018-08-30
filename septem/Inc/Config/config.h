#ifndef __CONFIG_H
#define __CONFIG_H

// macro define
#define dt 0.001f
//---------------------------------------------------------------------
// マシンデータ( 物理パラメータ )
//---------------------------------------------------------------------
#define MACHINE_WHEEL_RADIUS      0.01225f
// マシンのトレッド
#define MACHINE_TREAD_WIDTH       0.066f
// マシンの重さ
#define MACHINE_WEIGHT            0.106f
// ギア比
#define GEAR_RATION               5.25f // 42 / 8 = 5.25
// マシンのモーメント
#define INERTIAL_MOMENT           0.00005565f

//---------------------------------------------------------------------
// モーターデータ ( 1717 3v version ie 512 )
//---------------------------------------------------------------------
#define MOTOR_RESISTOR                  1.07f   // R
#define MOTOR_TORQUE_CONSTANT           0.00198f   // Kt
#define MOTOR_REVERSE_VOLTAGE_CONSTANT  0.000207f  // ke

#define MOTOR_CONTROL_PERIOD 1600

// タイヤが一回転するまでのエンコーダの値
#define MACHINE_ENC_CNT_PER_ROT   10752 // 512 * 4 * 5.25 

//---------------------------------------------------------------------
// enum 定義
//---------------------------------------------------------------------
// wall data
typedef enum {
  exist = 1,
  not_exist = 0,
}t_wall_data;

#endif /* __CONFIG_H */