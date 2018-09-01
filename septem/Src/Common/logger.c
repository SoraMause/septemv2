#include "logger.h"

#include "global_var.h"

#include <stdio.h>

static uint8_t logger_flag = 0;
static int16_t cnt_log = 0;

void updateLogger( void )
{
  if ( logger_flag == 1 ){
    if ( cnt_log < 2048 ){
      logger.v[cnt_log] = log_v;
      logger.v_target[cnt_log] = log_v_target;
      logger.omega[cnt_log] = log_omega;
      logger.omega_target[cnt_log] = log_omega_tareget;
      logger.rad[cnt_log] = log_rad;
      logger.rad_target[cnt_log] = log_rad_target;
      cnt_log++;
    } else {
      logger_flag = 0;
    }
  }
}

void setLogFlag( uint8_t _flag )
{
  logger_flag = _flag;
}

void showLog( void )
{

  printf( "v_target, v, omega_target, omega, rad_target, rad\r\n");  
  for ( int i = 0; i < cnt_log; i++ ){
      printf( "%d,%d,%f,%f,%f,%f\r\n", 
      logger.v_target[i],logger.v[i],logger.omega_target[i],
      logger.omega[i],logger.rad_target[i],logger.rad[i] );
  }
}