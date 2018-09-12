#include "systemOperator.h"
// task
#include "timer.h"

// hal libraly
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
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

// maze
#include "maze.h"
#include "agent.h"

static int16_t pattern = 0;
static uint8_t next_dir = front;
static uint8_t return_flag = 0;
static uint8_t gx = 0;
static uint8_t gy = 0;

void changePattern( int16_t _pattern )
{
  pattern = _pattern;
}

void MauseSystem( void )
{

  switch( pattern ){
    case 0:
      mazeStore_init();
      loadFlash( start_address, (uint8_t*)&maze_store, sizeof( t_maze ) );
      changePattern( 1 );
      break;

    case 1:
      if ( batt_monitor < 7.4f ) fullColorLedOut( LED_RED );
      else fullColorLedOut( LED_BLUE );

      printf( "batt_monitor = %f\r", batt_monitor );

      if ( getLeftPushsw() ){
        buzzerSetMonophonic( F_SCALE, 200 );
        HAL_Delay( 300 );
        changePattern( 10 );
      }

      if ( getRightPushsw() ){
        certainLedOut( LED_FRONT );
        HAL_Delay( 300 );
        setIrledPwm( IRLED_ON );
        changePattern( 100 );
      }

      break;

    case 100:
      printf( "0:%4d,1:%4d,2:%4d,3:%4d\r",
                sensor[0], sensor[1], sensor[2], sensor[3] );
      if ( getLeftPushsw() ){
        buzzerSetMonophonic( NORMAL, 100 );
        fullColorLedOut( LED_WHITE );
        changePattern( 101 );
      }
      //printf( "0:%4d,1:%4d,2:%4d,3:%4d\r",sensor[0], sensor[1], sensor[2], sensor[3] );
      break;

    case 101:
      if ( sensor[0] > 750 && sensor[3] > 750 ){
        MPU6500_z_axis_offset_calc_start();
        changePattern( 102 ); 
      }
      break;

    case 102:
      if ( MPU6500_calc_check() == 1 ){
        setSearchGain();
        fullColorLedOut( LED_GREEN );
        HAL_Delay( 1000 );
        fullColorLedOut( LED_CYAN );
        buzzerSetMonophonic( E_SCALE, 300 );
        HAL_Delay( 1000 );
        pushMotion( ADJ_FRONT );
        pushMotion( ONE_BLOCK_CHECK );
        pushMotion( ONE_BLOCK_CHECK );
        //pushMotion( HALF_BLOCK_STOP );
        //pushMotion( SEARCH_SLAROM_LEFT );
        pushMotion( HALF_BLOCK_STOP );
        pushMotion( DELAY );
        pushMotion( END_MOTION );
        setLogFlag( 1 );
        setMotionEnd( 1 );
        setControl( 1 );
        fullColorLedOut( LED_OFF );
        //setIrledPwm( IRLED_ON );
        changePattern( 43 );
      }
      break;

    //---------------------------------------------------------------
    // 足立法( 各種確認 ) pattern 10 ~ 
    //---------------------------------------------------------------
    case 10:
      fullColorLedOut( LED_WHITE );
      // 探索モードへ
      if ( getLeftPushsw() ){
        buzzerSetMonophonic( E_SCALE, 100 );
        HAL_Delay( 150 );
        buzzerSetMonophonic( E_SCALE, 100 );
        HAL_Delay( 150 );
        changePattern( 11 );
      }

      // 最短走行モードへ
      if ( getRightPushsw() && maze_store.save_flag == 1 ){
        buzzerSetMonophonic( C_SCALE, 100 );
        HAL_Delay( 150 );
        buzzerSetMonophonic( C_SCALE, 100 );
        HAL_Delay( 150 );
        mazeSubstituteData();
        changePattern( 40 );
      }

      break;

    case 11:
      if ( batt_monitor < 7.4f ) fullColorLedOut( LED_RED );
      else fullColorLedOut( LED_BLUE );

      if ( getLeftPushsw() ) {
        buzzerSetMonophonic( NORMAL, 100 );
        setIrledPwm( IRLED_ON );
        HAL_Delay( 300 );
        changePattern( 12 ); 
      }
      // map を表示
      if ( getRightPushsw() ){
        certainLedOut( LED_REAR );
        mazeUpdateMap( MAZE_GOAL_X, MAZE_GOAL_Y, MASK_SEARCH );
        certainLedOut( LED_OFF );
      }
      break;
    
    case 12:
      if ( getRightPushsw() ) return_flag = 1;
      fullColorLedOut( LED_MAGENTA );
      if ( sensor[0] > 750 && sensor[3] > 750 ){
        setSearchGain();
        setRunModde( search );
        setIrledPwm( IRLED_OFF );
        fullColorLedOut( LED_CYAN );
        buzzerSetMonophonic( A_SCALE, 200 );
        HAL_Delay( 300 );
        mazePosition_init();  // マシンの座標状況を初期化
        mazeWall_init();  // 壁情報を初期化
        mazeStore_init(); // store data を初期化
        motion_init();      // queueの中身をからにする
        gx = MAZE_GOAL_X;
        gy = MAZE_GOAL_Y;
        mazeUpdateMap( gx, gy, MASK_SEARCH );
        fullColorLedOut( LED_YELLOW );
        MPU6500_z_axis_offset_calc_start();
        changePattern( 13 );
      }
      break;

    case 13:
      if ( MPU6500_calc_check() == 1 ){
        fullColorLedOut( LED_GREEN );
        HAL_Delay( 1000 );
        fullColorLedOut( LED_CYAN );
        HAL_Delay( 1000 );
        pushMotion( ADJ_FRONT );
        mazeUpdatePosition( front );
        resetRadParam();
        setLogFlag( 1 );
        setMotionEnd( 1 );
        setControl( 1 );
        fullColorLedOut( LED_OFF );
        setIrledPwm( IRLED_ON );
        changePattern( 20 );
      }
      break;      
    //---------------------------------------------------------------
    // 足立法( slarom 探索 ) pattern 20 ~ 
    //---------------------------------------------------------------
    case 20:
      if ( checkMazeUpdateFlag() == 1 ){
        certainMazeUpdateFlag();
        mazeSetWall( mypos.x, mypos.y );
        mazeUpdateMap( gx, gy, MASK_SEARCH );
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
        maze.search[mypos.x][mypos.y] = 1;
        mazeUpdatePosition( next_dir );
        if ( mypos.x == gx && mypos.y == gy ){
          changePattern( 21 );
        }
      }
      break;

    case 21:
      if ( checkMazeUpdateFlag() == 1 ){
        maze.search[mypos.x][mypos.y] = 1;
        mazeSetWall( mypos.x, mypos.y );
        pushMotion( HALF_BLOCK_STOP );
        pushMotion( DELAY );
        if ( return_flag == 0 ){
          pushMotion( ROTATION );
          pushMotion( DELAY );
          pushMotion( ADJ_BACK );
        }
        pushMotion( END_MOTION );
        certainMazeUpdateFlag();
        changePattern( 22 );
      }
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
      fullColorLedOut( LED_BLUE );
      mazeStoreData();
      HAL_Delay( 300 );
      writeFlash( start_address, (uint8_t*)&maze_store, sizeof( t_maze ) );
      fullColorLedOut( LED_WHITE );
      changePattern( 24 );
      break;

    case 24:
      if ( return_flag == 1 ){
        changePattern( 26 );
      } else {
        changePattern( 25 );
      }
      break;

    case 25:
      buzzerSetMonophonic( G_SCALE, 100 );
      HAL_Delay( 150 );
      buzzerSetMonophonic( G_SCALE, 100 );
      motion_init();
      mazeInvertedDirection();
      gx = MAZE_START_X;
      gy = MAZE_START_Y;
      mazeUpdateMap( gx, gy, MASK_SEARCH );
      MPU6500_z_axis_offset_calc_start();
      return_flag = 1;
      changePattern( 13 );
      break;

    case 26:
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

    //---------------------------------------------------------------
    // 最短走行 ( 各種確認 ) pattern 40 ~ 
    //---------------------------------------------------------------
    case 40:
      if ( batt_monitor < 7.4f ) fullColorLedOut( LED_RED );
      else fullColorLedOut( LED_GREEN );
      
      if ( getLeftPushsw() ){
        fullColorLedOut( LED_CYAN );
        agentSetShortRoute( MAZE_GOAL_X, MAZE_GOAL_Y, 0 );
        HAL_Delay( 300 );
        setIrledPwm( IRLED_ON );
        changePattern( 41 );
      }

      if ( getRightPushsw() ){
        
        certainLedOut( LED_FRONT );
        agentSetShortRoute( MAZE_GOAL_X, MAZE_GOAL_Y, 1 );
        certainLedOut( LED_OFF );
      }

      break;

    case 41:
      fullColorLedOut( LED_MAGENTA );
      if ( sensor[0] > 750 && sensor[3] > 750 ){
        setFastGain();
        setRunModde( fast_run );
        setIrledPwm( IRLED_OFF );
        fullColorLedOut( LED_CYAN );
        buzzerSetMonophonic( G_SCALE, 200 );
        HAL_Delay( 300 );
        mazePosition_init();  // マシンの座標状況を初期化
        fullColorLedOut( LED_YELLOW );
        MPU6500_z_axis_offset_calc_start();
        changePattern( 42 );
      }
      break;

    case 42:
      if ( MPU6500_calc_check() == 1 ){
        fullColorLedOut( LED_GREEN );
        HAL_Delay( 1000 );
        fullColorLedOut( LED_CYAN );
        HAL_Delay( 1000 );
        setLogFlag( 1 );
        setMotionEnd( 1 );
        setControl( 1 );
        fullColorLedOut( LED_OFF );
        setIrledPwm( IRLED_ON );
        changePattern( 43 );
      }
      break;  

    case 43:
      if ( checkUpdateMotionEnd() == 1 ){
        setIrledPwm( IRLED_OFF );
        setLogFlag( 0 );
        fullColorLedOut( LED_WHITE );
        buzzerSetMonophonic( NORMAL, 300 );
        HAL_Delay( 300 );
        changePattern( 44 );
      }
      break;

    case 44:
      if ( getRightPushsw() ){
        certainLedOut( LED_REAR );
        showLog();
        certainLedOut( LED_OFF );
      }
      break;
  }
}