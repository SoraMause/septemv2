#include "maze.h"

#include <stdio.h>
// wall updateのためにインクルード
#include "global_var.h"

// 迷路のマシンのポジション
t_position mypos;

// 迷路データ
t_maze maze;

// 迷路保存用データ
t_maze maze_store;

// 迷路の壁を初期化
void mazeWall_init(void)
{

	// 配列の中身を全て0にしておく
	for (int i = 0; i <= MAZE_SIZE_X; i++) {
		for (int j = 0; j <= MAZE_SIZE_Y; j++) {
			maze.north_wall[i][j] = unknown;
			maze.east_wall[i][j] = unknown;
			maze.west_wall[i][j] = unknown;
			maze.south_wall[i][j] = unknown;
			maze.search[i][j] = 0;
		}
	}

	// スタートマスの右側の壁
	maze.east_wall[0][0] = exist;

	// 外壁の代入
	for (int i = 0; i <= MAZE_SIZE_X; i++) {
		maze.north_wall[i][MAZE_SIZE_Y] = exist; // 北壁
		maze.south_wall[i][0] = exist; // 南壁
	}

	for (int j = 0; j <= MAZE_SIZE_Y; j++) {
		maze.east_wall[MAZE_SIZE_X][j] = exist;
		maze.west_wall[0][j] = exist;
	}

	maze.search[0][0] = 1;	// スタートマスは探索済み

}

// マウスのポジションを初期化
void mazePosition_init(void)
{
	mypos.x = 0;
	mypos.y = 0;
	mypos.direction = north;
}

// ゴール座標を決定
void mazeSetGoal(uint8_t x, uint8_t y)
{
	maze.goal_x = x;
	maze.goal_y = y;
	maze.start_x = mypos.x;
	maze.start_y = mypos.y;
}

// update position 
void mazeUpdatePosition(uint8_t dir)
{
	// 次の動作で向き、座標を更新する	
	switch (dir) {
	case front:
		switch (mypos.direction) {
		case north:
			mypos.y++;
			break;
		case east:
			mypos.x++;
			break;
		case south:
			mypos.y--;
			break;
		case west:
			mypos.x--;
			break;
		default:
			break;
		}
		break;
	case left:
		switch (mypos.direction) {
		case north:
			mypos.x--;
			mypos.direction = west;
			break;
		case east:
			mypos.y++;
			mypos.direction = north;
			break;
		case south:
			mypos.x++;
			mypos.direction = east;
			break;
		case west:
			mypos.y--;
			mypos.direction = south;
			break;
		default:
			break;
		}
		break;
	case right:
		switch (mypos.direction) {
		case north:
			mypos.x++;
			mypos.direction = east;
			break;
		case east:
			mypos.y--;
			mypos.direction = south;
			break;
		case south:
			mypos.x--;
			mypos.direction = west;
			break;
		case west:
			mypos.y++;
			mypos.direction = north;
			break;
		default:
			break;
		}
		break;

	case rear:
		switch (mypos.direction) {
		case north:
			mypos.y--;
			mypos.direction = south;
			break;
		case east:
			mypos.x--;
			mypos.direction = west;
			break;
		case south:
			mypos.y++;
			mypos.direction = north;
			break;
		case west:
			mypos.x++;
			mypos.direction = east;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

void mazeSetWall( uint8_t x, uint8_t y )
{
	int8_t north_wall = 0;
	int8_t east_wall = 0;
	int8_t south_wall = 0;
	int8_t west_wall = 0;

	// 方向別に壁の状態を取得
	switch( mypos.direction ){
		case north:
			north_wall = sensor_frontl.is_wall | sensor_frontr.is_wall;
			east_wall = sensor_sider.is_wall;
			west_wall = sensor_sidel.is_wall;
			south_wall = 0;
			break;
		
		case east:
			east_wall = sensor_frontl.is_wall | sensor_frontr.is_wall;
			south_wall = sensor_sider.is_wall;
			north_wall = sensor_sidel.is_wall;
			west_wall = 0;
			break;

		case south:
			south_wall = sensor_frontl.is_wall | sensor_frontr.is_wall;
			west_wall = sensor_sider.is_wall;
			east_wall = sensor_sidel.is_wall;
			north_wall = 0;
			break;

		case west:
			west_wall = sensor_frontl.is_wall | sensor_frontr.is_wall;
			north_wall = sensor_sider.is_wall;
			south_wall = sensor_sidel.is_wall;
			east_wall = 0;
			break;

		default:
			break;
	}

	// 方向別に保存
	maze.north_wall[x][y] = north_wall;
	maze.east_wall[x][y] = east_wall;
	maze.south_wall[x][y] = south_wall;
	maze.west_wall[x][y] = west_wall;

	// 隣り合う壁の情報を保存
	if( y < MAZE_SIZE_Y-1 ){
		maze.south_wall[x][y+1] = north_wall;
	}

	if( x < MAZE_SIZE_X-1){
		maze.west_wall[x+1][y] = east_wall;
	}

	if(y > 0){
		maze.north_wall[x][y-1] = south_wall;
	}

	if(x > 0){
		maze.east_wall[x-1][y] = west_wall;
	}

}

void mazeClearStepMap(uint8_t x, uint8_t y)
{

	mazeSetGoal(x, y);

	for (int i = 0; i <= MAZE_SIZE_X; i++) {
		for (int j = 0; j <= MAZE_SIZE_Y; j++) {
			if (i == maze.goal_x && j == maze.goal_y) maze.step[i][j] = 0;
			else maze.step[i][j] = MAX_STEP;
		}
	}
}

void mazeUpdateMap(uint8_t x, uint8_t y, uint8_t mask)
{

	// input (x,y)ゴール座標
	uint8_t distPositionList[257];
	uint8_t head = 0, tail = 1;

	distPositionList[0] = x * 16 + y;

	mazeClearStepMap(x, y);

	while (head != tail) {
		//head = tail なら更新する区画がない
		int8_t Y = distPositionList[head] & 0x0f;
		int8_t X = (distPositionList[head] & 0xf0) >> 4;
		head++;

		// 北の情報を更新
		if (Y < MAZE_SIZE_Y) {
			if (((maze.north_wall[X][Y] & mask) == nowall) && (maze.step[X][Y + 1] == MAX_STEP)) {
				maze.step[X][Y + 1] = maze.step[X][Y] + 1;
				distPositionList[tail] = (X << 4) | (Y + 1);
				tail++;
			}
		}

		// 東の情報を更新
		if (X < MAZE_SIZE_X) {
			if (((maze.east_wall[X][Y] & mask) == nowall) && (maze.step[X + 1][Y] == MAX_STEP)) {
				maze.step[X + 1][Y] = maze.step[X][Y] + 1;
				distPositionList[tail] = ((X + 1) << 4) | (Y);
				tail++;
			}
		}

		// 南の情報を更新
		if (Y > 0) {
			if (((maze.south_wall[X][Y] & mask) == nowall) && (maze.step[X][Y - 1] == MAX_STEP)) {
				maze.step[X][Y - 1] = maze.step[X][Y] + 1;
				distPositionList[tail] = (X << 4) | (Y - 1);
				tail++;
			}
		}

		// 西の情報を更新
		if (X > 0) {
			if (((maze.west_wall[X][Y] & mask) == nowall) && (maze.step[X - 1][Y] == MAX_STEP)) {
				maze.step[X - 1][Y] = maze.step[X][Y] + 1;
				distPositionList[tail] = ((X - 1) << 4) | (Y);
				tail++;
			}
		}

	}

}

//探索済みか否か壁情報から判断
int8_t is_search(uint8_t x, uint8_t y)
{
	if ((maze.north_wall[x][y] == unknown) || (maze.east_wall[x][y] == unknown) || (maze.south_wall[x][y] == unknown) || (maze.west_wall[x][y] == unknown)) {
		return 0;  // 探索してないよ
	}
	else {
		return 1; // 探索済み
	}
}

//そのマスの情報から、優先度を算出する
uint8_t get_priority(uint8_t x, uint8_t y, uint8_t direction)
{
	//座標x,yと、向いている方角dirから優先度を算出する

	//未探索が一番優先度が高い.(4)
	//それに加え、自分の向きと、行きたい方向から、
	//前(2)横(1)後(0)の優先度を付加する。

	uint8_t priority;	//優先度を記録する変数

	priority = 0;


	if (mypos.direction == direction) {
		//行きたい方向が現在の進行方向と同じ場合
		priority = 2;
	}
	else if (((4 + mypos.direction - direction) % 4) == 2) {
		//行きたい方向が現在の進行方向と逆の場合
		priority = 0;
	}
	else {
		//それ以外(左右どちらか)の場合
		priority = 1;
	}


	if (is_search(x, y) == 0)
	{
		priority += 4;				//未探索の場合優先度をさらに付加
	}

	return priority;				//優先度を返す

}

uint8_t getNextdir(uint8_t mask)
{
	uint16_t step = MAX_STEP;
	uint8_t nextdir = rear;

	int8_t priority = 0;
	int8_t tmp_priority = 0;		//最小の値を探すために使用する変数


	switch (mypos.direction) {

	case north:

		if ((maze.north_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.y < MAZE_SIZE_Y) {
				tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
				if (maze.step[mypos.x][mypos.y + 1] < step) {
					step = maze.step[mypos.x][mypos.y + 1];
					nextdir = front;
					priority = tmp_priority;
				}
			}
		}

		if ((maze.west_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.x > 0) {
				tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
				if (maze.step[mypos.x - 1][mypos.y] < step) {
					step = maze.step[mypos.x - 1][mypos.y];
					nextdir = left;
					priority = tmp_priority;
				}
			}
		}

		if ((maze.east_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.x < MAZE_SIZE_X) {
				tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
				if (maze.step[mypos.x + 1][mypos.y] < step) {
					step = maze.step[mypos.x + 1][mypos.y];
					nextdir = right;
					priority = tmp_priority;
				}
			}
		}

		// serch mode のときだけ優先度による方向決めを行う　
		// 歩数が同じ場合は直進、左、右の順で優先順位をつける
		// もし、戻ってくる途中かつ見探索直進を行ける場合
		if (mask == MASK_SEARCH) {
			if (maze.goal_x == 0 && maze.goal_y == 0) {

				if ((maze.south_wall[mypos.x][mypos.y] & mask) == 0) {
					if (mypos.y > 0) {
						tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
						if (maze.step[mypos.x][mypos.y - 1] < step) {
							step = maze.step[mypos.x][mypos.y - 1];
							nextdir = rear;
							priority = tmp_priority;
						}
					}
				}
			}

			if ((maze.north_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.y < MAZE_SIZE_Y) {
					tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
					if (maze.step[mypos.x][mypos.y + 1] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = front;
						}
					}
				}
			}

			if ((maze.west_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.x > 0) {
					tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
					if (maze.step[mypos.x - 1][mypos.y] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = left;
						}
					}
				}
			}

			if ((maze.east_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.x < MAZE_SIZE_X) {
					tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
					if (maze.step[mypos.x + 1][mypos.y] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = right;
						}
					}
				}
			}

		}


		if (step == MAX_STEP) {
			nextdir = rear;
		}
		break;

	case east:
		if ((maze.east_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.x < MAZE_SIZE_X) {
				tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
				if (maze.step[mypos.x + 1][mypos.y] < step) {
					step = maze.step[mypos.x + 1][mypos.y];
					nextdir = front;
					priority = tmp_priority;
				}
			}
		}

		if ((maze.north_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.y < MAZE_SIZE_Y) {
				tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
				if (maze.step[mypos.x][mypos.y + 1] < step) {
					step = maze.step[mypos.x][mypos.y + 1];
					nextdir = left;
					priority = tmp_priority;
				}
			}
		}

		if ((maze.south_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.y > 0) {
				tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
				if (maze.step[mypos.x][mypos.y - 1] < step) {
					step = maze.step[mypos.x][mypos.y - 1];
					nextdir = right;
					priority = tmp_priority;
				}
			}
		}

		// serch mode のときだけ優先度による方向決めを行う　
		// 歩数が同じ場合は直進、左、右の順で優先順位をつける
		if (mask == MASK_SEARCH) {
			if (maze.goal_x == 0 && maze.goal_y == 0) {
				if ((maze.west_wall[mypos.x][mypos.y] & mask) == 0) {
					if (mypos.x > 0) {
						tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
						if (maze.step[mypos.x - 1][mypos.y] < step) {
							step = maze.step[mypos.x - 1][mypos.y];
							nextdir = rear;
							priority = tmp_priority;
						}
					}
				}
			}


			if ((maze.east_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.x < MAZE_SIZE_X) {
					tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
					if (maze.step[mypos.x + 1][mypos.y] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = front;
						}
					}
				}
			}

			if ((maze.north_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.y < MAZE_SIZE_Y) {
					tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
					if (maze.step[mypos.x][mypos.y + 1] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = left;
						}
					}
				}
			}

			if ((maze.south_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.y > 0) {
					tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
					if (maze.step[mypos.x][mypos.y - 1] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = right;
						}
					}
				}
			}
		}


		if (step == MAX_STEP) {
			nextdir = rear;
		}
		break;

	case south:
		if ((maze.south_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.y > 0) {
				tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
				if (maze.step[mypos.x][mypos.y - 1] < step) {
					step = maze.step[mypos.x][mypos.y - 1];
					nextdir = front;
					priority = tmp_priority;
				}
			}
		}

		if ((maze.east_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.x < MAZE_SIZE_X) {
				tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
				if (maze.step[mypos.x + 1][mypos.y] < step) {
					step = maze.step[mypos.x + 1][mypos.y];
					nextdir = left;
					priority = tmp_priority;
				}
			}
		}

		if ((maze.west_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.x > 0) {
				tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
				if (maze.step[mypos.x - 1][mypos.y] < step) {
					step = maze.step[mypos.x - 1][mypos.y];
					nextdir = right;
					priority = tmp_priority;
				}
			}
		}

		// serch mode のときだけ優先度による方向決めを行う　
		// 歩数が同じ場合は直進、左、右の順で優先順位をつける
		if (mask == MASK_SEARCH) {
			if (maze.goal_x == 0 && maze.goal_y == 0) {
				if ((maze.north_wall[mypos.x][mypos.y] & mask) == 0) {
					if (mypos.y < MAZE_SIZE_Y) {
						tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
						if (maze.step[mypos.x][mypos.y + 1] < step) {
							step = maze.step[mypos.x][mypos.y + 1];
							nextdir = rear;
							priority = tmp_priority;
						}
					}
				}
			}

			if ((maze.south_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.y > 0) {
					tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
					if (maze.step[mypos.x][mypos.y - 1] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = front;
						}
					}
				}
			}

			if ((maze.east_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.x < MAZE_SIZE_X) {
					tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
					if (maze.step[mypos.x + 1][mypos.y] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = left;
						}
					}
				}
			}

			if ((maze.west_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.x > 0) {
					tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
					if (maze.step[mypos.x - 1][mypos.y] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = right;
						}
					}
				}
			}
		}


		if (step == MAX_STEP) {
			nextdir = rear;
		}

		break;

	case west:
		if ((maze.west_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.x > 0) {
				tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
				if (maze.step[mypos.x - 1][mypos.y] < step) {
					step = maze.step[mypos.x - 1][mypos.y];
					nextdir = front;
					priority = tmp_priority;
				}
			}
		}

		if ((maze.south_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.y > 0) {
				tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
				if (maze.step[mypos.x][mypos.y - 1] < step) {
					step = maze.step[mypos.x][mypos.y - 1];
					nextdir = left;
					priority = tmp_priority;					
				}
			}
		}

		if ((maze.north_wall[mypos.x][mypos.y] & mask) == 0) {
			if (mypos.y < MAZE_SIZE_Y) {
				tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
				if (maze.step[mypos.x][mypos.y + 1] < step) {
					step = maze.step[mypos.x][mypos.y + 1];
					nextdir = right;
					priority = tmp_priority;
				}
			}
		}

		// serch mode のときだけ優先度による方向決めを行う　
		// 歩数が同じ場合は直進、左、右の順で優先順位をつける
		if (mask == MASK_SEARCH) {
			if (maze.goal_x == 0 && maze.goal_y == 0) {
				if ((maze.east_wall[mypos.x][mypos.y] & mask) == 0) {
					if (mypos.x < MAZE_SIZE_X) {
						tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
						if (maze.step[mypos.x + 1][mypos.y] <= step) {
							if (priority < tmp_priority) {
								priority = tmp_priority;
								nextdir = rear;
							}
						}
					}
				}
			}

			if ((maze.west_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.x > 0) {
					tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
					if (maze.step[mypos.x - 1][mypos.y] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = front;
						}
					}
				}
			}

			if ((maze.south_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.y > 0) {
					tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
					if (maze.step[mypos.x][mypos.y - 1] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = left;
						}
					}
				}
			}

			if ((maze.north_wall[mypos.x][mypos.y] & mask) == 0) {
				if (mypos.y < MAZE_SIZE_Y) {
					tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
					if (maze.step[mypos.x][mypos.y + 1] <= step) {
						if (priority < tmp_priority) {
							priority = tmp_priority;
							nextdir = right;
						}
					}
				}
			}

		}


		if (step == MAX_STEP) {
			nextdir = rear;
		}
		break;

	default:
		break;
	}
	return nextdir;
}


void mazeInvertedDirection(void)
{
	switch (mypos.direction) {
	case north:
		mypos.direction = south;
		break;

	case south:
		mypos.direction = north;
		break;

	case east:
		mypos.direction = west;
		break;

	case west:
		mypos.direction = east;
		break;

	default:
		break;
	}
}

void mazeStore_init(void)
{
	maze_store.save_flag = 0;
	// 配列の中身を全て初期化する
	for (int i = 0; i <= MAZE_SIZE_X; i++) {
		for (int j = 0; j <= MAZE_SIZE_Y; j++) {
			maze_store.north_wall[i][j] = 0;
			maze_store.east_wall[i][j] = 0;
			maze_store.west_wall[i][j] = 0;
			maze_store.south_wall[i][j] = 0;
			maze_store.search[i][j] = 0;
		}
	}
}

void mazeStoreData(void)
{
	maze_store.save_flag = 1;

	// 配列の中身を全て保存しなおす
	for (int i = 0; i <= MAZE_SIZE_X; i++) {
		for (int j = 0; j <= MAZE_SIZE_Y; j++) {
			maze_store.north_wall[i][j] = (maze_store.north_wall[i][j] & MASK_SEARCH);
			maze_store.east_wall[i][j] = (maze_store.east_wall[i][j] & MASK_SEARCH);
			maze_store.west_wall[i][j] = (maze_store.west_wall[i][j] & MASK_SEARCH);
			maze_store.south_wall[i][j] = (maze_store.south_wall[i][j] & MASK_SEARCH);

			maze_store.north_wall[i][j] = maze.north_wall[i][j];
			maze_store.east_wall[i][j] = maze.east_wall[i][j];
			maze_store.west_wall[i][j] = maze.west_wall[i][j];
			maze_store.south_wall[i][j] = maze.south_wall[i][j];
			
			maze_store.search[i][j] |= maze.search[i][j];
		}
	}
}

void mazeSubstituteData(void)
{
	maze.save_flag = maze_store.save_flag;
	// フラッシュに保存していた壁情報、探索情報を読み込み
	for (int i = 0; i <= MAZE_SIZE_X; i++) {
		for (int j = 0; j <= MAZE_SIZE_Y; j++) {
			maze.north_wall[i][j] = maze_store.north_wall[i][j];
			maze.east_wall[i][j] = maze_store.east_wall[i][j];
			maze.west_wall[i][j] = maze_store.west_wall[i][j];
			maze.south_wall[i][j] = maze_store.south_wall[i][j];
			maze.search[i][j] = maze_store.search[i][j];
		}
	}
}

// 壁情報を stdout に出力
void mazeWallOutput( uint8_t mode )
{
  
	printf("\r\nwall data\r\n");

	for( int j = MAZE_SIZE_Y; j >= 0; j-- ){
		//横壁
		for( int i = 0; i <= MAZE_SIZE_X; i++ ){
			printf("+");
			if( maze.north_wall[i][j] == exist ){
				printf("-----");
			}
			else {
				printf("     ");
			}
		}
		printf("+\r\n");

		//縦壁
		for( int i = 0; i <= MAZE_SIZE_X; i++ ){
			if( maze.west_wall[i][j] == exist ){
				printf("|");
			}
			else{
				printf(" ");
			}

			//マスの中央に表示するもの
			if( maze.step[i][j] == 0 ){
				printf("  G  ");
			}
			else if( (i == MAZE_START_X)&&(j == MAZE_START_Y) ){
				printf("  S  ");
			}
			else{
				printf("%5d", maze.step[i][j]);
			}
		}
		if( maze.east_wall[MAZE_SIZE_X][j] == exist ){
			printf("|");
		}
		else{
			printf(" ");
		}
		printf("\r\n");

		//外壁(南壁)の描画
		if( j == 0 ){
			for( int i = 0; i <= MAZE_SIZE_X; i++ ){
				printf("+");
				if( maze.south_wall[i][j] == exist ){
					printf("-----");
				}
				else{
					printf("     ");
				}
			}
			printf("+\r\n");
		}
	}
}

void mazeUpdateStraightWeightMap( uint8_t gx, uint8_t gy )
{
	uint8_t is_end = 1;
	uint8_t next_flag = 1;
	uint16_t step_number = 0;
	
	mazeClearStepMap( gx, gy );

	while( is_end == 1 ){
		is_end = 0;
		for( int i = 0; i <= MAZE_SIZE_X; i++ ){
			for( int j = 0; j <= MAZE_SIZE_Y; j++ ){
				
				// 北壁
				if ( maze.step[i][j] == MAX_STEP ){
					// 歩数が初期値なら更新
					if ( j < MAZE_SIZE_Y){	
						if ( maze.search[i][j] == 1 ){
							if ( (maze.north_wall[i][j] == 0)&&(maze.step[i][j+1] == step_number) ){
							// 南壁がないなら直進だから+1
								if ( maze.south_wall[i][j] == 0 ) {
									maze.step[i][j] = step_number + 1;
									is_end = 1;	
								} else {
									maze.step[i][j] = step_number + 2;
									is_end = 1;	
									next_flag = 1;
								} 
							}						
						}
					}

					// 東壁
					if ( maze.search[i][j] == 1 ){
					if ( i < MAZE_SIZE_X ){	
						if ( (maze.east_wall[i][j] == 0)&&(maze.step[i+1][j] == step_number) ){
								if ( maze.west_wall[i][j] == 0 ) {
									maze.step[i][j] = step_number + 1;
									is_end = 1;	
								} else {
									maze.step[i][j] = step_number + 2;
									is_end = 1;	
									next_flag = 1;
								} 
							}	
						}
					}

					// 南壁
					if ( maze.search[i][j] == 1 ){
					if ( j > 0 ) {					
						if ( (maze.south_wall[i][j] == 0)&&(maze.step[i][j-1] == step_number) ){
								if ( maze.north_wall[i][j] == 0 ) {
									maze.step[i][j] = step_number + 1;
									is_end = 1;	
								} else {
									maze.step[i][j] = step_number + 2;
									is_end = 1;	
									next_flag = 1;
								} 
							}
						}
					}

					// 西壁
					if ( maze.search[i][j] == 1 ){
					if ( i > 0 ){					
						if ( (maze.west_wall[i][j] == 0)&&(maze.step[i-1][j] == step_number) ){
								if ( maze.east_wall[i][j] == 0 ) {
									maze.step[i][j] = step_number + 1;
									is_end = 1;	
								} else {
									maze.step[i][j] = step_number + 2;
									is_end = 1;	
									next_flag = 1;
								} 
							}
						}
					}

				}
			} // end for j
		} // end for i
		if ( is_end == 0 && next_flag == 1 ){ 
			is_end = 1;
			next_flag = 0;
		}
		if ( is_end == 1 ) step_number++;
	} // end while

}
