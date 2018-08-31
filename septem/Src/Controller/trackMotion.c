#include "trackMotion.h" 

#include "motion.h"

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

void updateMotion( void )
{
  if ( head > 4000 ) head = 0;

  if ( checkEndMotion() == 1 ){
    switch( motion_queue[head] ){
      case NO_CONTROL:
        motion = no_control;
        break;

      case END_MOTION:
        motion = no_control;
        end_flag = 1;
        // to do 終了フラグを立てモーターを止め処理を終了する
        break;

      case DELAY:
        motion = delay;
        head++;
        break;

      case HALF_BLOCK:
        motion = straight;
        speedTrapezoid( 90.0f, 4.0f, 500.0f, 500.0f, 500.0f );
        head++;
        break;

      case ONE_BLOCK:
        motion = straight;
        speedTrapezoid( 180.0f, 4.0f, 500.0f, 500.0f, 500.0f );
        head++;
        break;

      case ONE_BLOCK_CHECK:
        motion = straight;
        speedTrapezoid( 180.0f, 4.0f, 500.0f, 500.0f, 500.0f );
        head++;
        break;

      case ADJ_FRONT:
        motion = straight;
        speedTrapezoid( 128.0f, 4.0f, 500.0f, 0.0f, 500.0f );
        head++;
        break;

      case ADJ_BACK:
        motion = straight;
        speedTrapezoid( -38.0f, -4.0f, -400.0f, 500.0f, 0.0f );
        head++;
        break;

      case HALF_BLOCK_STOP:
        motion = straight;
        speedTrapezoid( 90.0f, 4.0f, 500.0f, 500.0f, 0.0f );
        head++;
        break;

      case TURN_LEFT:
        motion = turn;
        yawrateTrapezoid( 1.5708f, 62.832f, 7.85398f );
        head++;
        break;

      case TURN_RIGHT:
        motion = turn;
        yawrateTrapezoid( -1.5708f, -62.832f, -7.85398f );
        head++;
        break;

      case ROTATION:
        motion = turn;
        yawrateTrapezoid( 3.14f, 62.8f, 12.566f );
        head++;
        break;

      case SEARCH_SLAROM_LEFT:
        motion = slarom;
        setSlarom( 1.5708f, 125.664f, 12.5664f, 500.0f, 23.5f, 22.5f );
        head++;
        break;

      case SEARCH_SLAROM_RIGHT:
        motion = slarom;
        setSlarom( -1.5708f, -125.664f, -12.5664f, 500.0f, 23.5f, 22.5f );
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