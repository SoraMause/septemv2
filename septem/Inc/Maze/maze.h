#ifndef __MAZE_H
#define __MAZE_H

#include <stdint.h>

#define MAZE_SIZE_X   15
#define MAZE_SIZE_Y   15

#define MAZE_START_X  0
#define MAZE_START_Y  0
#define MAZE_GOAL_X   1
#define MAZE_GOAL_Y   0

#define MAX_STEP      65535

#define HALF_BLOCK_DISTANCE   90.0f
#define ONE_BLOCK_DISTANCE    180.0f

#define SLATING_ONE_BLOCK_DISTANCE 127.0f 

#define MASK_SEARCH	0x01
#define MASK_SHORT	0x03

typedef enum
{
	front=0,		//前
	right=1,		//右
	rear=2,			//後
	left=3,			//左
	short_left = 8,
	short_right = 9,
	diagonal = 10,	// 斜め
	dir_left = 11,	// 斜め左
	dir_right = 12, // 斜め右
  left_return = 13, // 斜め左から戻る
  right_return = 14, // 斜め右から戻る
	dia_turn_left = 15,				// 斜め左90度大廻り
	dia_turn_right = 16,			// 斜め右90度大廻り
	left_return_135 = 17,	// 斜め左135度復帰
	right_return_135 = 18, // 斜め右135度復帰
	right_180 = 19,				// 右180度
	left_180 = 20,				// 左180度
	dir_left_135 = 21,	// 斜め左135
	dir_right_135 = 22, // 斜め右135
	end_maze = 32,
}t_local_dir;	//自分から見た方向を示す列挙型

typedef enum
{
	north = 0,
	east = 1,
	south = 2,
	west = 3,
}t_direction;

typedef enum {
	unknown = 2,
	exist = 1,
	nowall = 0,
}t_wall_state;

typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t direction;
}t_position;

extern t_position mypos;

typedef struct {								// 迷路情報
	uint8_t goal_x;									// 目標のゴール座標
	uint8_t goal_y;									// 目標のゴール座標
	uint8_t start_x;								// スタートマス
	uint8_t start_y;								// スタートマス
	uint16_t step[16][16];							// 歩数マップ
	uint8_t north_wall[16][16];						// 壁情報
	uint8_t east_wall[16][16];						// 壁情報
	uint8_t south_wall[16][16];						// 壁情報
	uint8_t west_wall[16][16];						// 壁情報
	uint8_t search[16][16];								// その区画を探索したかどうか
	uint8_t save_flag;                    // flashに保存するかどうかの判定フラグ
}t_maze;

extern t_maze maze;
extern t_maze maze_store;

// 関数のプロトタイプ宣言
void mazeWall_init(void);
void mazePosition_init(void);
void mazeSetGoal(uint8_t x, uint8_t y);
void mazeUpdatePosition(uint8_t dir);
void mazeClearStepMap(uint8_t x, uint8_t y);
void mazeUpdateMap(uint8_t x, uint8_t y, uint8_t mask);
int8_t is_search(uint8_t x, uint8_t y);
uint8_t get_priority(uint8_t x, uint8_t y, uint8_t direction);
uint8_t getNextdir(uint8_t mask);
void mazeInvertedDirection(void);

void mazeStore_init(void);
void mazeStoreData(void);
void mazeSubstituteData(void);

void mazeSetWall( uint8_t x, uint8_t y );
void mazeWallOutput( uint8_t mode );

void mazeUpdateStraightWeightMap( uint8_t gx, uint8_t gy );

#endif /*__MAZE_H*/