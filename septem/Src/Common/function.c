#include "function.h"

#include "stm32f4xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"

#include "global_var.h"

#include "led.h"
#include "buzzer.h"
#include "flash.h"

static float batt_calc_const = 0.0f;
static uint8_t ctr_irled = 0;

///////////////////////////////////////////////////////////////////////
// machine_init
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
void machine_init( void )
{
  setbuf( stdout, NULL );
  setIrledPwm( IRLED_OFF );
  buzzerSetMonophonic( NORMAL, 100 );
  HAL_Delay( 300 );
  certainLedOut( LED_OFF );
  fullColorLedOut( LED_OFF );
  MPU6500_init();
  buzzerSetMonophonic( NORMAL, 100 );
  HAL_Delay( 300 );
  MPU6500_z_axis_offset_calc_start();
  HAL_TIM_Encoder_Start( &htim3, TIM_CHANNEL_ALL ); // encoder
  HAL_TIM_Encoder_Start( &htim4, TIM_CHANNEL_ALL ); // encoder
  HAL_TIM_Base_Start_IT( &htim5 );
  HAL_ADC_Start_DMA( &hadc2, (uint32_t *)&batt_analog,1 );
  batt_calc_const = 3.3f / 960.0f;
}

///////////////////////////////////////////////////////////////////////
// batt voltage calculation
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
float battMonitor( int16_t data )
{
  float batt_voltage;
  //batt_voltage = 3.3 * batt_analog / 1024.0;
  batt_voltage = (float)( batt_calc_const * data );
  return batt_voltage;
}

///////////////////////////////////////////////////////////////////////
// able ir led
// [argument] ired 1:paluse emit ON 0:OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void setIrledPwm( uint8_t able )
{
	ctr_irled = able;
}

///////////////////////////////////////////////////////////////////////
// emit ir led ( side )
// [argument] ired 1 : ON 0 : OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void irledSideOut( uint8_t liting )
{
	if ( liting == IRLED_ON ) {
		// パルス発光が有効なら点灯
		if ( ctr_irled == IRLED_ON ) {
      HAL_GPIO_WritePin(sensor_paluse2_GPIO_Port,sensor_paluse2_Pin,GPIO_PIN_SET );	// IRLED ON
      HAL_GPIO_WritePin(sensor_paluse3_GPIO_Port,sensor_paluse3_Pin,GPIO_PIN_SET );	// IRLED ON
		} else {
      HAL_GPIO_WritePin(sensor_paluse2_GPIO_Port,sensor_paluse2_Pin,GPIO_PIN_RESET ); // IRLED OFF
      HAL_GPIO_WritePin(sensor_paluse3_GPIO_Port,sensor_paluse3_Pin,GPIO_PIN_RESET ); // IRLED OFF
		}
	} else {
    HAL_GPIO_WritePin(sensor_paluse2_GPIO_Port,sensor_paluse2_Pin,GPIO_PIN_RESET ); // IRLED OFF
    HAL_GPIO_WritePin(sensor_paluse3_GPIO_Port,sensor_paluse3_Pin,GPIO_PIN_RESET ); // IRLED OFF
	}	
}

///////////////////////////////////////////////////////////////////////
// emit ir led 
// [argument] ired 1 : ON 0 : OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void irledFrontOut( uint8_t liting )
{
	if ( liting == IRLED_ON ) {
		// パルス発光が有効なら点灯
		if ( ctr_irled == IRLED_ON ) {
      HAL_GPIO_WritePin(sensor_paluse1_GPIO_Port,sensor_paluse1_Pin,GPIO_PIN_SET );	// IRLED ON
			HAL_GPIO_WritePin(sensor_paluse4_GPIO_Port,sensor_paluse4_Pin,GPIO_PIN_SET );	// IRLED ON
		} else {
      HAL_GPIO_WritePin(sensor_paluse1_GPIO_Port,sensor_paluse1_Pin,GPIO_PIN_RESET ); // IRLED OFF
			HAL_GPIO_WritePin(sensor_paluse4_GPIO_Port,sensor_paluse4_Pin,GPIO_PIN_RESET ); // IRLED OFF
		}
	} else {
    HAL_GPIO_WritePin(sensor_paluse1_GPIO_Port,sensor_paluse1_Pin,GPIO_PIN_RESET ); // IRLED OFF
	  HAL_GPIO_WritePin(sensor_paluse4_GPIO_Port,sensor_paluse4_Pin,GPIO_PIN_RESET ); // IRLED OFF
	}	
}