#include "systemOperator.h"
// task
#include "timer.h"

// hal libraly
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
// standard libraly
#include <stdio.h>
#include <stdint.h>
// config
#include "global_var.h"
#include "config.h"
// common
#include "led.h"
#include "flash.h"
#include "function.h"
#include "logger.h"
#include "buzzer.h"

// controller
#include "trackMotion.h"
#include "motion.h"
#include "targetGenerator.h"

static int16_t pattern = 0;

void changePattern( int16_t _pattern )
{
  pattern = _pattern;
}

void MauseSystem( void )
{
  switch( pattern ){
    case 0:
      if ( batt_monitor < 7.4f ) fullColorLedOut( LED_RED );
      else fullColorLedOut( LED_BLUE );

      if ( getLeftPushsw() ){
        HAL_Delay( 300 );
        setIrledPwm( IRLED_ON );
        changePattern( 1 );
      }
      break;

    case 1:
      fullColorLedOut( LED_MAGENTA );
      if ( sensor[0] > 2000 && sensor[3] > 2000 ){
        fullColorLedOut( LED_CYAN );
        buzzerSetMonophonic( A_SCALE, 200 );
        HAL_Delay( 200 );
        fullColorLedOut( LED_YELLOW );
        MPU6500_z_axis_offset_calc_start();
        changePattern( 2 );
      }
      break;

    case 2:
      if ( MPU6500_calc_check() == 1 ){
        fullColorLedOut( LED_CYAN );
        HAL_Delay( 1000 );
        pushMotion( ADJ_FRONT );
        pushMotion( ONE_BLOCK_CHECK );
        pushMotion( ONE_BLOCK_CHECK );
        pushMotion( HALF_BLOCK_STOP );
        pushMotion( DELAY );
        pushMotion( END_MOTION );
        setLogFlag( 1 );
        setMotionEnd( 1 );
        setControl( 1 );
        setIrledPwm( IRLED_ON );
        changePattern( 3 );
      }
      break;

    case 3:
      if ( checkUpdateMotionEnd() == 1 ){
        changePattern( 4 );
      }
      break;

    case 4:
      fullColorLedOut( LED_WHITE );
      if( getRightPushsw() ){
        showLog();
      }
  }
}