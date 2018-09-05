#include "trackMotion.h" 
#include "targetGenerator.h"
#include "motion.h"
#include "led.h"

#include <stdio.h>

static int8_t motion_queue[4096];
static int16_t head = 0;
static int16_t last = 0;
static int8_t motion = 0;
static int8_t end_flag = 0;

void motion_init( void )
{
  head = 0;
  last = 0;
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
        resetRadParam();
        resetMotion();
        motion = delay;
        head++;
        break;

      case HALF_BLOCK:
        motion = straight;
        setControlWallPD( 1 );
        speedTrapezoid( 90.0f, 4.0f, 500.0f, 500.0f, 500.0f );
        head++;
        break;
      
      case HALF_BLOCK_SEARCH:
        motion = straight;
        setControlWallPD( 1 );
        fullColorLedOut( LED_RED );
        certainLedOut( LED_OFF );
        setMazeWallUpdate( 1 ); 
        speedTrapezoid( 90.0f, 4.0f, 500.0f, 500.0f, 500.0f );
        head++;
        break;

      case ONE_BLOCK:
        motion = straight;
        setControlWallPD( 1 );
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
        setControlWallPD( 1 );
        speedTrapezoid( -60.0f, -4.0f, -300.0f, 0.0f, 0.0f );
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

      case SET_STRAIGHT:
        motion = straight;
        fullColorLedOut( LED_RED );
        certainLedOut( LED_FRONT );
        setControlWallPD( 1 );
        //speedTrapezoid( set_distance[head], 4.0f, set_speed[head], set_start_speed[head], set_end_speed[head] );
        head++;
        break;

      case TURN_LEFT:
        fullColorLedOut( LED_GREEN );
        certainLedOut( LED_REAR );
        motion = turn;
        setControlWallPD( 0 );
        yawrateTrapezoid( 90.0f, 1540.0f, 360.0f );
        head++;
        break;

      case TURN_RIGHT:
        motion = turn;
        fullColorLedOut( LED_GREEN );
        certainLedOut( LED_OFF );
        setControlWallPD( 0 );
        yawrateTrapezoid( -90.0f, -1540.0f, -360.0f );
        head++;
        break;

      case ROTATION:
        motion = turn;
        fullColorLedOut( LED_GREEN );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        yawrateTrapezoid( 180.0f, 1540.0f, 450.0f );
        head++;
        break;

      case SEARCH_SLAROM_LEFT:
        motion = slarom;
        fullColorLedOut( LED_CYAN );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setMazeWallUpdate( 1 );
        setSlarom( 90.0f, 7080.0f, 720.0f, 500.0f, 23.5f, 21.5f );
        head++;
        break;

      case SEARCH_SLAROM_RIGHT:
        motion = slarom;
        fullColorLedOut( LED_MAGENTA );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setMazeWallUpdate( 1 );
        setSlarom( -90.0f, -7080.0f, -720.0f, 500.0f, 23.5f, 21.5f );
        head++;
        break;

      case SLAROM_LEFT:
        motion = slarom;
        fullColorLedOut( LED_CYAN );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setSlarom( 90.0f, 7080.0f, 720.0f, 500.0f, 23.5f, 21.5f );
        head++;
        break;

      case SLAROM_RIGHT:
        motion = slarom;
        fullColorLedOut( LED_MAGENTA );
        certainLedOut( LED_FRONT );
        setControlWallPD( 0 );
        setSlarom( -90.0f, -7080.0f, -720.0f, 500.0f, 23.5f, 21.5f );
        head++;
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