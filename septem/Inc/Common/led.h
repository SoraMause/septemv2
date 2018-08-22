#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

// led の光らせ方
#define LED_OFF         0x00
#define LED_FRONT       0x01
#define LED_REAR        0x08
#define LED_BOTH        0x0f
#define LED_RED         0x01
#define LED_GREEN       0x10
#define LED_BLUE        0x08
#define LED_YELLOW      0x11
#define LED_MAGENTA     0x09
#define LED_CYAN        0x18
#define LED_WHITE       0xff

uint8_t getLeftPushsw( void );
uint8_t getRightPushsw( void );
void fullColorLedOut( uint8_t led );
void certainLedOut( uint8_t led );

#ifdef __cplusplus
 }
#endif

#endif /* __LED_H */