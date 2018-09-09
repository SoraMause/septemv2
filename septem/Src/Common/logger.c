#include "logger.h"

#include "global_var.h"

#include <stdio.h>

static uint8_t logger_flag = 0;
static int16_t cnt_log = 0;
static uint8_t cnt_log_timer = 0;

void updateLogger( void )
{
  if ( logger_flag == 1 ){
    if ( cnt_log < 2048 && cnt_log_timer > 4 ){
      logger.v[cnt_log] = log_v;
      logger.v_target[cnt_log] = log_v_target;
      logger.distance[cnt_log] = log_distance;
      logger.omega[cnt_log] = log_omega;
      logger.rad[cnt_log] = log_rad;
      logger.omega_target[cnt_log] = log_omega_target;
      logger.batt[cnt_log] = log_batt;
      logger.sensor_fl[cnt_log] = log_sensorfl;
      logger.sensor_sl[cnt_log] = log_sensorsl;
      logger.sensor_fr[cnt_log] = log_sensorfr;
      logger.sensor_sr[cnt_log] = log_sensorsr;
      cnt_log++;
      cnt_log_timer = 0;
    } else {
      cnt_log_timer++;
    }
  } else {
    cnt_log_timer = 0;
     logger_flag = 0;
  }
}

void setLogFlag( uint8_t _flag )
{
  logger_flag = _flag;
}

void showLog( void )
{
  if( logger_flag == 1 ) logger_flag = 0;
  printf( "v_target, v, distance, omega_target, omega, rad, sensor fl, sl, sr, fr,batt\r\n");  
  for ( int i = 0; i < cnt_log; i++ ){
      printf( "%d,%d,%f,%d,%d,%d,%d,%d,%d,%d,%f\r\n", 
      logger.v_target[i],
      logger.v[i],
      logger.distance[i],
      logger.omega_target[i],
      logger.omega[i],
      logger.rad[i],
      logger.sensor_fl[i], 
      logger.sensor_sl[i], 
      logger.sensor_sr[i], 
      logger.sensor_fr[i], 
      logger.batt[i]  );
  }
}