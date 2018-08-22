#include "led.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "gpio.h"

///////////////////////////////////////////////////////////////////////
// read push switch(front) 
// [argument] nothing
// [Substitutiong] nothing
// [return] ON 1 OFF 0
///////////////////////////////////////////////////////////////////////
uint8_t getLeftPushsw( void )
{
  if ( HAL_GPIO_ReadPin( pushsw1_GPIO_Port, pushsw1_Pin ) == 0 ){
    return 1;
  } else {
    return 0;
  }
}

///////////////////////////////////////////////////////////////////////
// read push switch(tail) 
// [argument] nothing
// [Substitutiong] nothing
// [return] ON 1 OFF 0
///////////////////////////////////////////////////////////////////////
uint8_t getRightPushsw( void )
{
  if ( HAL_GPIO_ReadPin( pushsw2_GPIO_Port,pushsw2_Pin ) == 0 ){
    return 1;
  } else {
    return 0;
  }
}

///////////////////////////////////////////////////////////////////////
// certain led ( tail )
// [argument] LED_RIGHT_SIDE,LED_LEFT_SIDE,LED_BOTH,LED_OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void certainLedOut( uint8_t led ){
  switch ( led ){
    case LED_OFF:
      HAL_GPIO_WritePin( led1_GPIO_Port,led1_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( led2_GPIO_Port,led2_Pin,GPIO_PIN_SET );
      break;

    case LED_FRONT:
      HAL_GPIO_WritePin( led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( led2_GPIO_Port,led2_Pin,GPIO_PIN_SET );
      break;

    case LED_REAR:
      HAL_GPIO_WritePin( led1_GPIO_Port,led1_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET );
      break;

    case LED_BOTH:
      HAL_GPIO_WritePin( led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET );
      break;
    
    default:
      led = LED_OFF;
      break;
  }

}

///////////////////////////////////////////////////////////////////////
// certain led ( full color )
// [argument] 7 colors,LED_OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void fullColorLedOut( uint8_t led )
{
  switch( led ){
    case LED_OFF:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_SET );
      break;
    
    case LED_RED:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_SET );
      break;
    
    case LED_GREEN:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_SET );
      break;

    case LED_BLUE:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_RESET );
      break;

    case LED_YELLOW:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_SET );
      break;

    case LED_MAGENTA:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_RESET );
      break;
 
    case LED_CYAN:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_SET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_RESET );
      break;

    case LED_WHITE:
      HAL_GPIO_WritePin( fled_red_GPIO_Port,fled_red_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_green_GPIO_Port,fled_green_Pin,GPIO_PIN_RESET );
      HAL_GPIO_WritePin( fled_blue_GPIO_Port,fled_blue_Pin,GPIO_PIN_RESET );
      break;

    default:
      led = LED_OFF;
      break;
  }
}