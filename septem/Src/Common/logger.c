#include "logger.h"

#include "global_var.h"

#include <stdio.h>

static uint8_t logger_flag = 0;
static int16_t cnt_log = 0;
static uint8_t cnt_log_timer = 0;

void updateLogger( void )
{
  if ( logger_flag == 1 ){
    if ( cnt_log < 2048 && cnt_log_timer > 10 ){
      logger.v[cnt_log] = log_v;
      logger.v_target[cnt_log] = log_v_target;
      logger.omega[cnt_log] = log_omega;
      logger.rad[cnt_log] = log_rad;
      logger.rad_target[cnt_log] = log_rad_target;
      logger.batt[cnt_log] = log_batt;
      logger.sensor_fl[cnt_log] = log_sensorfl;
      logger.sensor_sl[cnt_log] = log_sensorsl;
      logger.sensor_fr[cnt_log] = log_sensorfr;
      logger.sensor_sr[cnt_log] = log_sensorsr;
      cnt_log++;
    } else {
      cnt_log_timer++;
      logger_flag = 0;
    }
  } else {
    cnt_log_timer = 0;
  }
}

void setLogFlag( uint8_t _flag )
{
  logger_flag = _flag;
}

void showLog( void )
{

  printf( "v_target, v, omega, rad_target, rad, sensor fl, sl, sr, fr\r\n");  
  for ( int i = 0; i < cnt_log; i++ ){
      printf( "%d,%d,%f,%d,%d,%d,%d,%d,%d,%f\r\n", 
      logger.v_target[i],logger.v[i],logger.omega[i],logger.rad_target[i],logger.rad[i],
      logger.sensor_fl[i], logger.sensor_sl[i], logger.sensor_sr[i], logger.sensor_fr[i],
      logger.batt[i]  );
  }
}