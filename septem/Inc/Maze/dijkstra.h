#ifndef __DIJKSTRA_H
#define __DIJKSTRA_H

#include <stdint.h>

//コマンドリスト（GO1は１区画前進を意味する）
#define GO1    1//必ず１である必要がある
#define GO2    2
#define GO14  14
#define GO15  15
#define TURNR 16//GO15の次にあれば何でもよい
#define TURNL 17
#define DIA_TO_CLOTHOIDR 18
#define DIA_TO_CLOTHOIDL 19
#define DIA_FROM_CLOTHOIDR 20
#define DIA_FROM_CLOTHOIDL 21
#define DIA_TURNR 22
#define DIA_TURNL 23
#define DIA_GO1 24
#define DIA_GO31 55 
#define SNODE 56//ストップを意味する
 
//コマンドリストここまで

int8_t checkDijkstra( int16_t gx, int16_t gy );

int8_t getRouteArray( int16_t gx, int16_t gy, int16_t route[256], int8_t out_flag );

void inputMazeWallData( void );

#endif /* __DIJKSTRA_H */