/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "arm_math.h" 
#include "global_var.h"
#include "config.h"
#include <stdio.h>
#include <math.h>

static int16_t gyro_offset_cnt = 0; 
static int8_t  gyro_calc_flag = 0;
static float gyro_z_offset = 0.0f;
static float gyro_z_before = 0.0f;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
///////////////////////////////////////////////////////////////////////
// spi read 1 byte
// [argument] Register
// [Substitutiong] nothing
// [return] 1byte data
///////////////////////////////////////////////////////////////////////
uint8_t read_byte( uint8_t reg )
{
  uint8_t ret,val;
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port, gyro_cs_Pin, GPIO_PIN_RESET ); //cs = 0;
  ret = reg | 0x80 ;
  HAL_SPI_Transmit( &hspi2, &ret,1,100 );
  HAL_SPI_Receive( &hspi2,&val,1,100 ); 
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port,gyro_cs_Pin, GPIO_PIN_SET );  //cs = 1;
  return val;
}

///////////////////////////////////////////////////////////////////////
// spi read 1 byte upper bit ( signed )
// [argument] Register
// [Substitutiong] nothing
// [return] 1byte upper bit data 
///////////////////////////////////////////////////////////////////////
int16_t read_shift_byte( uint8_t reg )
{
  uint8_t address,val;
  int16_t ret;
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port, gyro_cs_Pin, GPIO_PIN_RESET );
  address = reg | 0x80 ;
  HAL_SPI_Transmit( &hspi2, &address,1,100 );
  HAL_SPI_Receive( &hspi2,&val,1,100 );
  ret = (int16_t)(val<< 8);
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port,gyro_cs_Pin, GPIO_PIN_SET );
  return ret;
}

///////////////////////////////////////////////////////////////////////
// spi write 1 byte
// [argument] Register
// [Substitutiong] write data
// [return] nothong
///////////////////////////////////////////////////////////////////////
void write_byte( uint8_t reg, uint8_t val )
{
  uint8_t ret;
  ret = reg & 0x7F ;
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port, gyro_cs_Pin, GPIO_PIN_RESET );
  HAL_SPI_Transmit( &hspi2, &ret,1,100 ); // 書き込みアドレス
  HAL_SPI_Transmit( &hspi2, &val,1,100 ); // 書き込み
  HAL_GPIO_WritePin( gyro_cs_GPIO_Port,gyro_cs_Pin, GPIO_PIN_SET );
}

///////////////////////////////////////////////////////////////////////
// set up mpu-6500
// [argument] nothing
// [Substitutiong] nothing
// [return] nothong
///////////////////////////////////////////////////////////////////////
void MPU6500_init( void )
{
  uint8_t who_am_i;
  uint8_t recheck_who_am_i;
  // MPU-6500の??��?��?バイスチェ??��?��?ク
  
  who_am_i = read_byte( MPU6500_RA_WHO_AM_I );
  printf( "\r\n0x%x\r\n",who_am_i );

  if ( who_am_i != 0x70 ){
    HAL_Delay( 100 );
    recheck_who_am_i = read_byte( MPU6500_RA_WHO_AM_I );
    if ( recheck_who_am_i != 0x70 ){
      while(1){
        printf( "gyro_error\r");
      }
    } else {
      printf( "recheck_who_am_i = 0x%x\r\n", recheck_who_am_i );
    }
  }

  HAL_Delay( 100 );

  // スリープ解除
  write_byte( MPU6500_RA_PWR_MGMT_1, 0x00 );

  HAL_Delay( 100 );

	// DLPF_CFG = 0 : GyroのLPFを無効??��?��?
	// FIFOは使わな??��?��?
  write_byte( MPU6500_RA_CONFIG, 0x00 );

  HAL_Delay( 100 );

  // Gyroのフルスケール??��?��?+-2000dpsに設??��?��?
	write_byte(MPU6500_RA_GYRO_CONFIG, 0x18);

}


///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis data
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
float MPU6500_read_gyro_z( void )
{
  int16_t gyro_z;
  float omega;

  gyro_z = (int16_t)( read_shift_byte(MPU6500_RA_GYRO_ZOUT_H) | read_byte(MPU6500_RA_GYRO_ZOUT_L) );

  // GYRO FACTOR ( rad / sec )
  // 180 * PI ( rad/sec )
  
  omega = (float)( ( gyro_z - gyro_z_offset ) / GYRO_FACTOR / 180.0f * PI);

  return omega;
}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis offset start
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
void MPU6500_z_axis_offset_calc_start( void )
{
  gyro_z_offset = 0.0f;
  gyro_offset_cnt = 0; 
  gyro_calc_flag = 0;
}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis offset calc
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
void MPU6500_z_axis_offset_calc( void )
{
  int16_t gyro_z;
  //float omega;

  gyro_z = (int16_t)( read_shift_byte(MPU6500_RA_GYRO_ZOUT_H) | read_byte(MPU6500_RA_GYRO_ZOUT_L) );

  // GYRO FACTOR ( rad / sec )
  // 180 * PI ( rad/sec )
  
  //omega = (float)gyro_z / GYRO_FACTOR / 180.0f * PI;

  if ( gyro_offset_cnt < 1000 ){
    gyro_z_offset += (float)gyro_z;
    gyro_offset_cnt++;
  } else {
    gyro_z_offset /= 1000.0f;
    //gyro_z_offset = roundf( gyro_z_offset );
    gyro_calc_flag = 1;
    machine_rad = 0.0f;
  }
}

int8_t MPU6500_calc_check( void )
{
  return gyro_calc_flag;
}

///////////////////////////////////////////////////////////////////////
// machineRadCalculation
// [argument] nothing
// [Substitutiong] machine_rad
// [return] nothing
// [contents] caluculate the machine rad
///////////////////////////////////////////////////////////////////////
void machineRadCalculation( float gyro )
{
  machine_rad += (float)( (gyro_z_before + gyro) * dt / 2.0f );
  gyro_z_before = gyro;
}

// debug ON
float checkGyroOffset()
{
  return gyro_z_offset;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
