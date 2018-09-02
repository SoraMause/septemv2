#ifndef __GLOBAL_VAR_H
#define __GLOBAL_VAR_H

#include <stdint.h>

// pattern
extern int16_t pattern;

// tim5 interrupt
extern uint8_t timer125;

// gyro data
extern float gyro_z_measured;
extern float machine_rad;

// sensor data
extern int16_t batt_analog;
extern float batt_buff[4];
extern float batt_monitor;

extern int16_t sensorL[4];
extern int16_t sensorH[4];
extern int16_t sensor[4];

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
  float omega_target[2048];
  float omega[2048];
  float rad_target[2048];
  float rad[2048];
}Logger;

extern Logger logger;

// 外部ファイルから値を回収する用のグローバル変数
extern int16_t log_v_target;
extern int16_t log_v;
extern float log_omega_tareget;
extern float log_omega;
extern float log_rad_target;
extern float log_rad;

#endif /* __GLOBAL_VAR_H */