#ifndef __FUNCTION_H
#define __FUNCTION_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#define IRLED_ON  1
#define IRLED_OFF 0

void machine_init( void );
float battMonitor( int16_t data );
void setIrledPwm( uint8_t ired );
void irledOut( uint8_t liting );

#ifdef __cplusplus
 }
#endif

#endif /* __FUNCTION_H */