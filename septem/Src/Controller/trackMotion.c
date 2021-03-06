#include "trackMotion.h" 
#include "targetGenerator.h"
#include "motion.h"
#include "led.h"
#include "global_var.h"

#include <stdio.h>

static int16_t head = 0;
static int16_t last = 0;
static int8_t motion = 0;
static int8_t end_flag = 0;

void motion_init( void )
{
  head = 0;
  last = 0;
  end_flag = 0;
  for ( int i = 0; i < 4096; i++ ){
    motion_queue[i] = no_control;
  }
}

void pushMotion( int8_t motion_in )
{
  if ( last < 4000 ){
    motion_queue[last] = motion_in;
    last++;
  } else {
    last = 0;
    motion_queue[last] = motion_in;
    last++;
  }
}

void showMotion( void )
{
  printf( "head = %4d, last = %4d\r\n",head, last );
  for ( int i = 0; i < last; i++ ){
    printf( "motion_queue[%4d] = %4d\r\n",i,motion_queue[i] );
  }
}

void updateMotion( void )
{
  if ( head > 4000 ) head = 0;

  if ( checkEndMotion() == 1 ){
    switch( motion_queue[head] ){
      case NO_CONTROL:
        fullColorLedOut( LED_OFF );
        certainLedOut( LED_OFF );
        motion = no_control;
        break;

      case END_MOTION:
        motion = no_control;
        end_flag = 1;
        // to do 終了フラグを立てモーターを止め処理を終了する
        break;

      case DELAY:
        fullColorLedOut( LED_YELLOW );
        certainLedOut( LED_OFF );
        setControlWallPD( 0 );
        resetMotion();
        motion = delay;
        head++;
        break;

      case FRONTPD_DELAY:
        fullColorLedOut( LED_YELLOW );
        certainLedOut( LED_OFF );
        setControlWallPD( 0 );
        resetMotion();
        setControlFrontPD( 1 );
        motion = delay;
        head++;
        break;

      case HALF_BLOCK:
        motion = straight;
        setControlWallPD( 0 );
        speedTrapezoid( 39.0f, 12.0f, 700.0f, 0.0f, 700.0f );
        head++;
        break;
      
      case HALF_BLOCK_SEARCH:
        motion = straight;
        setControlWallPD( 1 );
        fullColorLedOut( LED_RED );
        certainLedOut( LED_OFF );
        setMazeWallUpdate( 0 ); 
        speedTrapezoid( 90.0f, 4.0f, 500.0f, 500.0f, 0.0f );
        head++;
        break;

      case ONE_BLOCK:
        motion = straight;
        setControlWallPD( 0 );
        speedTrapezoid( 180.0f, 4.0f, 500.0f, 500.0f, 500.0f );
        head++;
        break;

      case ONE_BLOCK_CHECK:
        motion = straight;
        fullColorLedOut( LED_BLUE );
        certainLedOut( LED_OFF );
        setControlWallPD( 1 );
        setMazeWallUpdate( 1 ); 
        speedTrapezoid( 180.0f, 4.0f, 500.0f, 500.0f, 500.0f );
        head++;
        break;

      case ADJ_FRONT:
        motion = straight;
        fullColorLedOut( LED_YELLOW );
        certainLedOut( LED_REAR );
        setControlWallPD( 1 );
        setMazeWallUpdate( 1 ); 
        speedTrapezoid( 129.0f, 4.0f, 500.0f, 0.0f, 500.0f );
        head++;
        break;

      case ADJ_BACK:
        motion = straight;
        fullColorLedOut( LED_OFF );
        certainLedOut( LED_REAR );
        setControlWallPD( 0 );
        speedTrapezoid( -50.0f, -4.0f, -300.0f, 0.0f, 0.0f );
        head++;
        break;

      case HALF_BLOCK_STOP:
        motion = straight;
        fullColorLedOut( LED_RED );
        certainLedOut( LED_FRONT );
        setControlWallPD( 1 );
        speedTrapezoid( 90.0f, 4.0f, 500.0f, 500.0f, 0.0f );
        head++;
        break;

      case SET_FRONT_PD_STRAIGHT:
        motion = straight;
        fullColorLedOut( LED_RED );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setControlFrontPD( 1 );
        speedTrapezoid( fast_path[head].distance, 16.0f, fast_path[head].speed, fast_path[head].start_speed, fast_path[head].end_speed );
        head++;
        break;

      case SET_STRAIGHT:
        motion = straight;
        fullColorLedOut( LED_RED );
        certainLedOut( LED_FRONT );
        setControlWallPD( 1 );
        speedTrapezoid( fast_path[head].distance, 12.0f, fast_path[head].speed, fast_path[head].start_speed, fast_path[head].end_speed );
        head++;
        break;         

      case SET_DIA_STRAIGHT:
        motion = straight;
        fullColorLedOut( LED_RED );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        speedTrapezoid( fast_path[head].distance, 12.0f, fast_path[head].speed, fast_path[head].start_speed, fast_path[head].end_speed );
        head++;
        break;     

      case TURN_LEFT:
        fullColorLedOut( LED_GREEN );
        certainLedOut( LED_REAR );
        motion = turn;
        setControlWallPD( 0 );
        yawrateTrapezoid( 90.0f, 1080.0f, 270.0f );
        head++;
        break;

      case TURN_RIGHT:
        motion = turn;
        fullColorLedOut( LED_GREEN );
        certainLedOut( LED_OFF );
        setControlWallPD( 0 );
        yawrateTrapezoid( -90.0f, -1080.0f, -270.0f );
        head++;
        break;

      case ROTATION:
        motion = turn;
        fullColorLedOut( LED_GREEN );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        yawrateTrapezoid( 180.0f, 1080.0f, 360.0f );
        head++;
        break;

      case SEARCH_SLAROM_LEFT:
        motion = slarom;
        fullColorLedOut( LED_CYAN );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setAfterWallPD();
        setMazeWallUpdate( 1 );
        setSlarom( 90.0f, 7200.0f, 630.0f, 500.0f, 21.0f, 20.0f );
        head++;
        break;

      case SEARCH_SLAROM_RIGHT:
        motion = slarom;
        fullColorLedOut( LED_MAGENTA );
        certainLedOut( LED_REAR );
        setControlWallPD( 0 );
        setMazeWallUpdate( 1 );
        setAfterWallPD();
        setSlarom( -90.0f, -7200.0f, -630.0f, 500.0f, 21.0f, 20.0f );
        head++;
        break;

      // 最短　斜めなし
      case SLAROM_LEFT:
        motion = slarom;
        fullColorLedOut( LED_CYAN );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setAfterWallPD();
        setSlarom( 90.0f, 7200.0f, 630.0f, 500.0f, 21.0f, 20.0f );
        head++;
        break;

      case SLAROM_RIGHT:
        motion = slarom;
        fullColorLedOut( LED_MAGENTA );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setAfterWallPD();
        setSlarom( -90.0f, -7200.0f, -630.0f, 500.0f, 21.0f, 20.0f );
        head++;
        break;

      // 最短斜めあり
      case DIR_ONE_BLOCK:
        motion = straight;
        fullColorLedOut( LED_RED );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        speedTrapezoid( 127.0f, 8.0f, 700.0f, 700.0f, 700.0f );
        head++;
        break;  

      // 45度ターン斜めになる
      case DIA_CENTER_LEFT:
        motion = slarom;
        setSlarom( 45.0f, 12000.0f, 630.0f, 700.0f, 44.5f, 81.0f );
        setControlWallPD( 0 );
        head++;
        break;

      case DIA_CENTER_RIGHT:
        motion = slarom;
        setSlarom( -45.0f, -12000.0f, -630.0f, 700.0f, 44.5f, 81.0f );
        setControlWallPD( 0 );
        head++;
        break;

      // 135度ターンから斜めになる
      case DIA_CENTER_LEFT_135:
        motion = slarom;
        setSlarom( 135.0f, 5400.0f, 540.0f, 700.0f, 50.0f, 32.0f );
        setControlWallPD( 0 );
        head++;
        break;

      case DIA_CENTER_RIGHT_135:
        motion = slarom;
        setSlarom( -135.0f, -5400.0f, -540.0f, 700.0f, 50.0f, 32.0f );
        setControlWallPD( 0 );
        head++;
        break;

      // 135度ターンから復帰
      case RETURN_DIA_LEFT_135:
        motion = slarom;
        setSlarom( 135.0f, 5760.0f, 630.0f, 700.0f, 54.0f, 68.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break;

      case RETURN_DIA_RIGHT_135:
        motion = slarom;
        setSlarom( -135.0f, -5760.0f, -630.0f, 700.0f, 54.0f, 68.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break;

      // 45度ターンから直線復帰
      case RETURN_DIA_LEFT:
        motion = slarom;
        setSlarom( 45.0f, 10000.0f, 630.0f, 700.0f, 78.0f, 40.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break;

      case RETURN_DIA_RIGHT:
        motion = slarom;
        setSlarom( -45.0f, -10000.0f, -630.0f, 700.0f, 78.0f, 40.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break; 

      // 中心から90度ターン
      case CENRTER_SLAROM_LEFT:
        motion = slarom;
        setSlarom( 90.0f, 6740.0f, 540.0f, 700.0f, 76.5f, 75.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break;

      case CENRTER_SLAROM_RIGHT:
        motion = slarom;
        setSlarom( -90.0f, -6740.0f, -540.0f, 700.0f, 76.5f, 75.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break;

      // 中心から180度ターン
      case SLAROM_LEFT_180:
        motion = slarom;
        setSlarom( 180.0f, 6300.0f, 450.0f, 700.0f, 39.0f, 37.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break;

      case SLAROM_RIGHT_180:
        motion = slarom;
        setSlarom( -180.0f, -6300.0f, -450.0f, 700.0f, 39.0f, 37.0f );
        setControlWallPD( 0 );
        setAfterWallPD();
        head++;
        break;

      // 斜め大回り90度ターン
      case DIA_LEFT_TURN:
        motion = slarom;
        setSlarom( 90.0f, 6300.0f, 630.0f, 700.0f, 26.0f, 25.0f );
        setControlWallPD( 0 );
        head++;
        break;

      case DIA_RIGHT_TURN:
        motion = slarom;
        setSlarom( -90.0f, -6300.0f, -630.0f, 700.0f, 26.0f, 25.0f );
        setControlWallPD( 0 );
        head++;
        break;

      // 宴会芸
      case ENKAIGEI:
        motion = turn;
        fullColorLedOut( LED_OFF );
        certainLedOut( LED_BOTH );
        setControlWallPD( 0 );
        setMotionEnd( 0 );
        break;

      default:
        break;
    }
  }

  // motiom == delay なら　motionDelay を呼ぶ
  if ( motion == delay ) motionDelay();  

}

int8_t checkUpdateMotionEnd( void )
{
  return end_flag;
}

int8_t checkNowMotion( void )
{
  return motion;
}