#ifndef __AGENT_H
#define __AGENT_H

#include <stdint.h>

int8_t agentGetShortRoute( uint8_t gx, uint8_t gy, float *all_time, uint8_t method, uint8_t outflag );
void agentSetShortRoute( uint8_t gx, uint8_t gy, uint8_t outflag );
int8_t agentDijkstraRoute( int16_t gx, int16_t gy, int8_t out_flag );
#endif /* __AGENT_H */

