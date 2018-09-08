#include "agent.h"
#include "maze.h"

#include "global_var.h"
#include "trackMotion.h"

#include <stdio.h>

void fast_path_init( void )
{
  for ( int i = 0; i < 256; i++ ){
    fast_path[i].speed = 0.0f;
    fast_path[i].end_speed = 0.0f;
    fast_path[i].start_speed = 0.0f;
    fast_path[i].distance = 0.0f;
  }
}

int8_t agentGetShortRoute( uint8_t gx, uint8_t gy, float *all_time, uint8_t method, uint8_t outflag )
{
  uint8_t nextdir = front;
  uint8_t count = 1;
  int8_t path[256];
  uint8_t motion_buff[256];
  int16_t path_number = 0;
  float rec_time = 0.0f;
  uint8_t counter = 0;

  for ( int i = 0; i < 256; i++ ){
    motion_queue[i] = 0;
    path[i] = 0;
    motion_buff[i] = 0;
  }

  motion_queue[0] = front;
  path[0] = -1;
  motion_buff[0] = front;

  // 初期化処理
  mazePosition_init();  // 迷路のポジションを初期化
  mazeWall_init();      // 壁情情報を初期化
  fast_path_init();
  mazeSubstituteData();

  if ( method == 0 ){
    mazeUpdateMap( gx, gy, MASK_SHORT ); // マップを展開
  } else {
    mazeUpdateStraightWeightMap( gx, gy );
  }
  
  if ( outflag == 1 ) mazeWallOutput(0);

  mazeUpdatePosition( front );

  while( 1 ){
    nextdir = getNextdir( MASK_SHORT );

    mazeUpdatePosition( nextdir );

    if ( nextdir != rear ){
      motion_queue[count] = nextdir;
      count++;
    } else {
      counter++;
    }

    if ( mypos.x == gx && mypos.y == gy ) break;

    if ( counter == 5 ){
      break;
    }
  }

  if ( counter == 5 ){
    return 0;
  } else {
    mazePosition_init();

    if ( outflag == 1 ){
      for( int i= 0; i < count; i++ ){
        switch( motion_queue[i] ){
          case front:
            printf( "data : %3d front ",i );
            break;

          case left:
            printf( "data : %3d left  ",i );
            break;

          case right:
            printf( "data : %3d right ",i );
            break;
        }

        if ( i % 4 == 0 ) printf( "\r\n" );
      }

      printf( "\r\n");
    }



    for ( int i = 0; i < count; i++ ){
      if ( motion_queue[i] == front ){
        if ( i == 0 ) {
          path[0] = 0;
        } else {
          path[path_number] += 1;
        }
        
      } else  {
        if ( motion_queue[i-1] == front ) {
          path_number++;
        }
        if ( motion_queue[i] == left ){
          path[path_number] = -1;
        } else if ( motion_queue[i] == right ){
          path[path_number] = -8;
        }
        
        path_number++;
      }
    }

    path_number++;

    for ( int i = 0; i < path_number; i++ ){
      if ( i == 0 && path[0] == 0 ){
            fast_path[0].distance = 130.0f;
            rec_time += 0.4f;
      } else if ( path[i] > 0 ){
        if ( i == 0 ){
          fast_path[i].distance = ONE_BLOCK_DISTANCE * (path[i]) + 130.0f;
          if ( path[i] < 2 ){
            rec_time += 0.18f * path[i] + 0.4f;
          } else {
            rec_time += 0.18f * path[i] + 0.4f;
          }

        } else {
          fast_path[i].distance = ONE_BLOCK_DISTANCE * path[i];
          if ( path[i] < 3 ){
            rec_time += 0.18f * path[i];
          } else {          
            rec_time += 0.18f * path[i];
          }
          
        }
        motion_buff[i] = front;
      } else if ( path[i] == -1 ){
        fast_path[i].distance = 0.0f;
        motion_buff[i] = left;
        rec_time += 0.3f;
      } else if ( path[i] == -8 ){
        fast_path[i].distance = 0.0f;
        motion_buff[i] = right;
        rec_time += 0.3f;
      }
    }

    // motion_queue の中の更新した部分を全て初期化する
    for( int i= 0; i < 256; i++ ){
      motion_queue[i] = 0;
    }
    
    for ( int i = 0; i < path_number; i++ ){
      if ( motion_buff[i] == front ){
        if ( i == 0 ) {
            fast_path[i].start_speed = 0.0f;
            if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 4.0f ){
              fast_path[i].speed = 2000.0f;
              if ( fast_path[i].speed > 2500.0f ) fast_path[i].speed = 2500.0f;
            } else if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 2.0f ){
              fast_path[i].speed = 1500.0f;
              if ( fast_path[i].speed > 2000.0f ) fast_path[i].speed = 2000.0f;
            } else if ( path[i] == 0 ){
              fast_path[i].speed = 500.0f;
            }
            fast_path[i].end_speed = 500.0f;
        } else {
          if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 4.0f ){
            fast_path[i].start_speed = fast_path[i-1].end_speed;
            fast_path[i].speed = 2000.0f;
            if ( fast_path[i].speed > 2500.0f ) fast_path[i].speed = 2500.0f;
            fast_path[i].end_speed = 500.0f;
          } else if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 2.0f ){
            fast_path[i].start_speed = fast_path[i-1].end_speed;
            fast_path[i].speed = 1500.0f;
            if ( fast_path[i].speed > 2000.0f ) fast_path[i].speed = 2000.0f;
            fast_path[i].end_speed = 500.0f;
          } else {
            fast_path[i].start_speed = fast_path[i-1].end_speed;
            fast_path[i].speed = 700.0f;
            fast_path[i].end_speed = 500.0f;
          }
        }

        motion_queue[i] = SET_STRAIGHT;
      } else if ( motion_buff[i] == left ){
          fast_path[i].start_speed = fast_path[i-1].end_speed;
          fast_path[i].speed = 500.0f;
          fast_path[i].end_speed = 500.0f;
          motion_queue[i] = SLAROM_LEFT;
      } else if ( motion_buff[i] == right ){
          fast_path[i].start_speed = fast_path[i-1].end_speed;
          fast_path[i].speed = 500.0f;
          fast_path[i].end_speed = 500.0f;
          motion_queue[i] = SLAROM_RIGHT;
      }
    }

    // 最後に半区画ストップを入れておく
    if (motion_queue[path_number-1] == SET_STRAIGHT ) {
      fast_path[path_number-1].distance = HALF_BLOCK_DISTANCE;
      fast_path[path_number-1].start_speed = fast_path[path_number-1].end_speed;
      fast_path[path_number-1].speed = 500.0f;
      fast_path[path_number-1].end_speed = 0.0f;
    } else {
      motion_queue[path_number] = SET_STRAIGHT;
      fast_path[path_number].distance = HALF_BLOCK_DISTANCE;
      fast_path[path_number].start_speed = fast_path[path_number-1].end_speed;
      fast_path[path_number].speed = 500.0f;
      fast_path[path_number].end_speed = 0.0f;
      path_number++;
    }
    motion_queue[path_number] = END_MOTION;
    path_number++;

    if ( outflag == 1 ){
      for ( int i = 0; i < path_number; i++ ){
        if ( motion_queue[i] == SET_STRAIGHT  ){
          printf( "fast_path[%3d] : distance : %5.5f, speed start : %5.5f, end : %5.5f, speed : %5.5f\r\n",i,
          fast_path[i].distance, fast_path[i].start_speed, fast_path[i].end_speed, fast_path[i].speed );
        } else if ( motion_queue[i] == SLAROM_LEFT ){
          printf( "fast_path[%3d] : slarom : left , speed start : %5.5f, end : %5.5f, speed : %5.5f\r\n",i,
                      fast_path[i].start_speed, fast_path[i].end_speed, fast_path[i].speed );
        } else if ( motion_queue[i] == SLAROM_RIGHT ){
          printf( "fast_path[%3d] : slarom : right , speed start : %5.5f, end : %5.5f, speed : %5.5f\r\n",i,
                      fast_path[i].start_speed, fast_path[i].end_speed, fast_path[i].speed );
        } else if ( motion_queue[i] == DELAY ){
          printf( "fast_path[%3d] : delay \r\n",i); 
        } else {
          printf( "motion end\r\n");
        }
      }
      
      printf( "record_time : %f \r\n",rec_time );
    }

    *all_time = rec_time;

    return 1;
  }
}

void agentSetShortRoute( uint8_t gx, uint8_t gy, uint8_t outflag )
{
  uint8_t checkA = 0;
  uint8_t checkB = 0;
  float timeA = 0.0f;
  float timeB = 0.0f;

  checkA = agentGetShortRoute( gx, gy,&timeA, 0, outflag );
  checkB = agentGetShortRoute( gx, gy,&timeB, 1, outflag );

  if ( checkA == 1 && checkB == 1 ){
    if ( timeA < timeB ){
      agentGetShortRoute( gx, gy,&timeA, 0, outflag );
    } else {
      agentGetShortRoute( gx, gy,&timeB, 1, outflag );
    }
  } else if ( checkA == 0 ){
    agentGetShortRoute( gx, gy,&timeB, 1, outflag );
  } else if ( checkB == 0 ){
    agentGetShortRoute( gx, gy,&timeA, 0, outflag );
  } else {
    return;
  }
}