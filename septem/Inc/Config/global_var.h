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
}ENC_VALUE;

extern ENC_VALUE enc_value;

#endif /* __GLOBAL_VAR_H */