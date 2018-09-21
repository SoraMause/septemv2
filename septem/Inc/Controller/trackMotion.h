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
// 探索用
#define HALF_BLOCK          4
#define HALF_BLOCK_SEARCH   5
#define ONE_BLOCK           6 
#define ONE_BLOCK_CHECK     7
#define ADJ_FRONT           8
#define ADJ_BACK            9
#define HALF_BLOCK_STOP     10
// 最短用
#define SET_STRAIGHT        11
#define SET_FRONT_PD_STRAIGHT 12

// other
#define DIR_ONE_BLOCK       13
#define DIR_FOUR_BLOCK    14

// turn 
#define TURN_LEFT           16
#define TURN_RIGHT          17
#define ROTATION            18

// slarom
// 探索用
#define SEARCH_SLAROM_LEFT  24
#define SEARCH_SLAROM_RIGHT 25

// 最短用
#define SLAROM_LEFT         26
#define SLAROM_RIGHT        27
// 区画の中心から斜め #0
#define DIA_CENTER_LEFT   28
#define DIA_CENTER_RIGHT  29
// 区画の中心から90度ターン #1
#define CENRTER_SLAROM_LEFT    30
#define CENRTER_SLAROM_RIGHT   31
// 斜め動作から直線復帰 #5
#define RETURN_DIA_RIGHT      32
#define RETURN_DIA_LEFT       33
// 180度大廻りターン #2
#define SLAROM_LEFT_180       34
#define SLAROM_RIGHT_180      35
// 斜め90度大廻ターン #3
#define DIA_LEFT_TURN       36
#define DIA_RIGHT_TURN      37
// 斜め135度ターンから直線復帰 #4
#define RETURN_DIA_LEFT_135   37
#define RETURN_DIA_RIGHT_135  37

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