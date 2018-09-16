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

// other
#define DIR_ONE_BLOCK       12
#define DIR_FOUR_BLOCK    13

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
// 区画の中心から45度ターン
#define SLAROM_CENTER_LEFT_45   28
#define SLAROM_CENTER_RIGHT_45  29
// 斜めから直線に戻る
// to do いくつかパターンが必要
#define SLAROM_LEFT_45     30

#define SLAROM_RIGHT_45    32

// 斜めから90度ターン
// いくつかパターンが必要!
#define SLAROM_DIA_LEFT_90     34

#define SLAROM_DIA_RIGHT_90     36

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