#ifndef __GLOBAL_VAR_H
#define __GLOBAL_VAR_H

#include <stdint.h>

// tim5 interrupt
extern uint8_t timer125;

// gyro data
extern float gyro_z_measured;
extern float machine_rad;

// sensor data
extern int16_t batt_analog;
extern float batt_monitor;

extern int16_t sensorL[4];
extern int16_t sensorH[4];
extern int16_t sensor[4];

// motion
extern int8_t motion_queue[4096];

// sensor struct
typedef struct {
  uint8_t is_wall;
  int16_t error;
}t_sensor;

extern t_sensor sensor_sider;     // sensor side right
extern t_sensor sensor_sidel;     // sensor side left
extern t_sensor sensor_frontr;    // sensor front right
extern t_sensor sensor_frontl;    // sensor front left

// encoder-------------------------------------------------------------
typedef struct {
  int32_t left;
  int32_t right;
  int32_t center;
}EncValue;

extern EncValue enc_value;

// logger -------------------------------------------------------------
typedef struct {
  int16_t v_target[2048];
  int16_t v[2048];
  float distance[2048];
  int16_t omega[2048];
  int16_t omega_target[2048];
  int16_t rad[2048];
  int16_t sensor_fl[2048];
  int16_t sensor_fr[2048];
  int16_t sensor_sl[2048];
  int16_t sensor_sr[2048];
  float batt[2048];
}Logger;

extern Logger logger;

// 外部ファイルから値を回収する用のグローバル変数
extern int16_t log_v_target;
extern int16_t log_v;
extern int16_t log_omega;
extern int16_t log_omega_target;
extern int16_t log_rad;
extern float log_distance;
extern float log_batt;
extern int16_t log_sensorfl;
extern int16_t log_sensorfr;
extern int16_t log_sensorsl;
extern int16_t log_sensorsr;

// 最短走行用
typedef struct {
  float distance;
  float start_speed;
  float end_speed;
  float speed;
}t_fast_path;

extern t_fast_path fast_path[256];

#endif /* __GLOBAL_VAR_H */