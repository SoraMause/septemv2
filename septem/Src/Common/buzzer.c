#include "tim.h"
#include "buzzer.h"

static uint16_t buzzer_counter = 1;
static uint16_t beep_time = 0;

void buzzerSetMonophonic( uint16_t scale, uint16_t time_beep )
{
    buzzerPwmOut( 99, scale ); 
    buzzer_counter = 0;
    beep_time = time_beep;
}

void buzzerOutPut( void )
{
  if ( buzzer_counter > beep_time ){
    buzzerPwmOut( 0, NORMAL );
  } else {
    buzzer_counter++;
  }

}