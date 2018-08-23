#include "global_var.h"

// tim5 interrupt
uint8_t timer125;

// gyro data
float gyro_z_measured;
float machine_rad;

// sensor data
int16_t batt_analog;
float batt_buff[4];
float batt_monitor;

int16_t sensorL[4];
int16_t sensorH[4];
int16_t sensor[4];

// sensor struct
t_sensor sensor_sider;
t_sensor sensor_frontr;
t_sensor sensor_frontl;
t_sensor sensor_sidel;

// enc struct
ENC_VALUE enc_value;
