#include "global_var.h"

// tim5 interrupt
uint8_t timer125;

// gyro data
float gyro_z_measured;
float machine_rad;

// machine_action time
uint32_t machine_act_time;

// sensor data
int16_t batt_analog;
float batt_monitor;

int16_t sensorL[4];
int16_t sensorH[4];
int16_t sensor[4];

// motion
int8_t motion_queue[4096];

// sensor struct
t_sensor sensor_sider;
t_sensor sensor_frontr;
t_sensor sensor_frontl;
t_sensor sensor_sidel;

// encoder struct
EncValue enc_value;

// log 関連
Logger logger;

// 外部ファイルから値を回収する用のグローバル変数
// 外部ファイルから値を回収する用のグローバル変数
int16_t log_v_target;
int16_t log_v;
int16_t log_omega;
int16_t log_omega_target;
int16_t log_rad;
float log_distance;
int16_t log_sensorfl;
int16_t log_sensorfr;
int16_t log_sensorsl;
int16_t log_sensorsr;

// 最短走行用
t_fast_path fast_path[256];