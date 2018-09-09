#ifndef __TRACKMOTION_H
#define __TRACKMOTION_H

#include <stdint.h>

//---------------------------------------------------------------------
// macro define 
//---------------------------------------------------------------------
// motion 関連
// general
#define NO_CONTROL          0
#define END_MOTION          1
#define DELAY               2

// straight
#define HALF_BLOCK          4
#define HALF_BLOCK_SEARCH   5
#define ONE_BLOCK           6 
#define ONE_BLOCK_CHECK     7
#define ADJ_FRONT           8
#define ADJ_BACK            9
#define HALF_BLOCK_STOP     10
#define SET_STRAIGHT        11

// turn 
#define TURN_LEFT           16
#define TURN_RIGHT          17
#define ROTATION            18

// slarom
#define SEARCH_SLAROM_LEFT  24
#define SEARCH_SLAROM_RIGHT 25
#define SLAROM_LEFT         26
#define SLAROM_RIGHT        27

// 宴会芸
#define ENKAIGEI            127

//---------------------------------------------------------------------
//enum
//---------------------------------------------------------------------
// motion name
typedef enum {
  no_control = 0,
  delay = 1,
  straight = 2,
  turn = 3,
  slarom = 4,
}t_motion_name;

//---------------------------------------------------------------------
// function protrype
//---------------------------------------------------------------------
void motion_init( void );
void pushMotion( int8_t motion_in );
void updateMotion( void );
int8_t checkUpdateMotionEnd( void );
int8_t checkNowMotion( void );

void showMotion( void );
#endif /* __TRACKMOTION_H */