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
#include "maze.h"

static int16_t pattern = 0;

void changePattern( int16_t _pattern )
{
  pattern = _pattern;
}

void MauseSystem( void )
{
  uint8_t next_dir = front;

  switch( pattern ){
    case 0:
      if ( batt_monitor < 7.4f ) fullColorLedOut( LED_RED );
      else fullColorLedOut( LED_BLUE );

      if ( getLeftPushsw() ){
        HAL_Delay( 300 );
        setIrledPwm( IRLED_ON );
        changePattern( 1 );
      }

      if ( getRightPushsw() ){
        certainLedOut( LED_FRONT );
        HAL_Delay( 300 );
        setIrledPwm( IRLED_ON );
        changePattern( 3 );
      }

      break;

    case 1:
      fullColorLedOut( LED_MAGENTA );
      if ( sensor[0] > 2000 && sensor[3] > 2000 ){
        setIrledPwm( IRLED_OFF );
        fullColorLedOut( LED_CYAN );
        buzzerSetMonophonic( A_SCALE, 200 );
        HAL_Delay( 300 );
        mazePosition_init();
        mazeWall_init();
        mazeUpdateMap( MAZE_GOAL_X, MAZE_GOAL_Y, MASK_SEARCH );
        fullColorLedOut( LED_YELLOW );
        MPU6500_z_axis_offset_calc_start();
        changePattern( 2 );
      }
      break;

    case 2:
      if ( MPU6500_calc_check() == 1 ){
        fullColorLedOut( LED_GREEN );
        HAL_Delay( 1000 );
        fullColorLedOut( LED_CYAN );
        HAL_Delay( 1000 );
        pushMotion( ADJ_FRONT );
        mazeUpdatePosition( front );
        setLogFlag( 1 );
        setMotionEnd( 1 );
        setControl( 1 );
        fullColorLedOut( LED_OFF );
        setIrledPwm( IRLED_ON );
        changePattern( 20 );
      }
      break;

    case 3:
      printf( "0:%4d,1:%4d,2:%4d,3:%4d\r",sensor[0], sensor[1], sensor[2], sensor[3] );
      break;

      
    //---------------------------------------------------------------
    // 足立法( slarom 探索 ) pattern 20 ~ 
    //---------------------------------------------------------------
    case 20:
      if ( checkMazeUpdateFlag() == 1 ){
        certainMazeUpdateFlag();
        mazeSetWall( mypos.x, mypos.y );
        mazeUpdateMap( MAZE_GOAL_X, MAZE_GOAL_Y, MASK_SEARCH );
        next_dir = getNextdir( MASK_SEARCH );
        switch( next_dir ){
          case front:
            pushMotion( ONE_BLOCK_CHECK );
            break;

          case left:
            pushMotion( SEARCH_SLAROM_LEFT );
            break;

          case right:
            pushMotion( SEARCH_SLAROM_RIGHT );
            break;

          case rear:
            pushMotion( HALF_BLOCK_STOP );
            pushMotion( DELAY );
            pushMotion( ROTATION );
            pushMotion( DELAY );
            pushMotion( ADJ_BACK );
            pushMotion( DELAY );
            pushMotion( ADJ_FRONT ); 
            break;
        }
        mazeUpdatePosition( next_dir );
        if ( mypos.x == MAZE_GOAL_X && mypos.y == MAZE_GOAL_Y ){
          changePattern( 21 );
        }
      }
      break;

    case 21:
      pushMotion( HALF_BLOCK_STOP );
      mazeSetWall( mypos.x, mypos.y );
      pushMotion( DELAY );
      pushMotion( END_MOTION );
      certainMazeUpdateFlag();
      changePattern( 22 );
      break;

    case 22:
      if ( checkUpdateMotionEnd() == 1 ){
        setIrledPwm( IRLED_OFF );
        setLogFlag( 0 );
        fullColorLedOut( LED_WHITE );
        buzzerSetMonophonic( NORMAL, 300 );
        HAL_Delay( 300 );
        changePattern( 23 );
      }
      break;

    case 23:
      if ( getLeftPushsw() ){
        certainLedOut( LED_FRONT );
        mazeUpdateMap( MAZE_GOAL_X, MAZE_GOAL_Y, MASK_SHORT );
        mazeWallOutput( 0 );
        certainLedOut( LED_OFF );
      }

      if ( getRightPushsw() ){
        certainLedOut( LED_REAR );
        showLog();
        certainLedOut( LED_OFF );
      }
      break;
  }
}