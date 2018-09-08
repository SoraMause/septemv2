/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include <stdint.h>

#include "config.h"
#include "global_var.h"

ADC_ChannelConfTypeDef sConfig;

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA3     ------> ADC1_IN3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();
  
    /**ADC2 GPIO Configuration    
    PA4     ------> ADC2_IN4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    /**ADC3 GPIO Configuration    
    PA1     ------> ADC3_IN1
    PA2     ------> ADC3_IN2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA3     ------> ADC1_IN3 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_3);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO Configuration    
    PA4     ------> ADC2_IN4 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PA1     ------> ADC3_IN1
    PA2     ------> ADC3_IN2 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2);

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
///////////////////////////////////////////////////////////////////////
// set up ad converter 1 ( side )
// [argument] nothing
// [Substitutiong] sensorH[4]
// [return] nothing
///////////////////////////////////////////////////////////////////////
void update_sidesensorH_data( void )
{
  //read object
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

  sConfig.Channel = ADC_CHANNEL_1;  // channel set
  HAL_ADC_ConfigChannel( &hadc3, &sConfig );  // setting store
  HAL_ADC_Start( &hadc3 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc3,100 ) != HAL_OK );  // trans
  sensorH[1] = HAL_ADC_GetValue( &hadc3 );   // get value
  HAL_ADC_Stop( &hadc3 );

  sConfig.Channel = ADC_CHANNEL_2;  // channel set
  HAL_ADC_ConfigChannel( &hadc3, &sConfig );  // setting store
  HAL_ADC_Start( &hadc3 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc3,100 ) != HAL_OK );  // trans
  sensorH[2] = HAL_ADC_GetValue( &hadc3 );   // get value
  HAL_ADC_Stop( &hadc3 );

}

///////////////////////////////////////////////////////////////////////
// set up ad converter 1 ( front )
// [argument] nothing
// [Substitutiong] sensorH[4]
// [return] nothing
///////////////////////////////////////////////////////////////////////
void update_frontsensorH_data( void )
{
  //read object
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  
  sConfig.Channel = ADC_CHANNEL_0;  // channel set
  HAL_ADC_ConfigChannel( &hadc1, &sConfig );  // setting store
  HAL_ADC_Start( &hadc1 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc1,100 ) != HAL_OK );  // trans
  sensorH[0] = HAL_ADC_GetValue( &hadc1 );   // get value
  HAL_ADC_Stop( &hadc1 );

  sConfig.Channel = ADC_CHANNEL_3;  // channel set
  HAL_ADC_ConfigChannel( &hadc1, &sConfig );  // setting store
  HAL_ADC_Start( &hadc1 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc1,100 ) != HAL_OK );  // trans
  sensorH[3] = HAL_ADC_GetValue( &hadc1 );   // get value
  HAL_ADC_Stop( &hadc1 );

}

///////////////////////////////////////////////////////////////////////
// set up ad converter 1 ( side )
// [argument] nothing
// [Substitutiong] sensorL[4]
// [return] nothing
///////////////////////////////////////////////////////////////////////
void update_sidesensorL_data( void )
{
  //read object
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

  sConfig.Channel = ADC_CHANNEL_1;  // channel set
  HAL_ADC_ConfigChannel( &hadc3, &sConfig );  // setting store
  HAL_ADC_Start( &hadc3 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc3,100 ) != HAL_OK );  // trans
  sensorL[1] = HAL_ADC_GetValue( &hadc3 );   // get value
  HAL_ADC_Stop( &hadc3 );

  sConfig.Channel = ADC_CHANNEL_2;  // channel set
  HAL_ADC_ConfigChannel( &hadc3, &sConfig );  // setting store
  HAL_ADC_Start( &hadc3 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc3,100 ) != HAL_OK );  // trans
  sensorL[2] = HAL_ADC_GetValue( &hadc3 );   // get value
  HAL_ADC_Stop( &hadc3 );

}

///////////////////////////////////////////////////////////////////////
// set up ad converter 1 ( front )
// [argument] nothing
// [Substitutiong] sensorL[4]
// [return] nothing
///////////////////////////////////////////////////////////////////////
void update_frontsensorL_data( void )
{
  //read object
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  
  sConfig.Channel = ADC_CHANNEL_0;  // channel set
  HAL_ADC_ConfigChannel( &hadc1, &sConfig );  // setting store
  HAL_ADC_Start( &hadc1 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc1,100 ) != HAL_OK );  // trans
  sensorL[0] = HAL_ADC_GetValue( &hadc1 );   // get value
  HAL_ADC_Stop( &hadc1 );

  sConfig.Channel = ADC_CHANNEL_3;  // channel set
  HAL_ADC_ConfigChannel( &hadc1, &sConfig );  // setting store
  HAL_ADC_Start( &hadc1 );     // ad convert start
  while( HAL_ADC_PollForConversion( &hadc1,100 ) != HAL_OK );  // trans
  sensorL[3] = HAL_ADC_GetValue( &hadc1 );   // get value
  HAL_ADC_Stop( &hadc1 );

}

///////////////////////////////////////////////////////////////////////
// update sensor data
// [argument] nothing
// [Substitutiong] sensor[4]
// [return] nothing
///////////////////////////////////////////////////////////////////////
void update_sensor_data( void )
{
  
  // sensor値の補正�?える�?
  sensor[0] = sensorH[0] - sensorL[0];  // Measures against external light

  if ( sensor[0] <= 2800 ){
    sensor_frontr.is_wall = 0;
  } else {
    sensor_frontr.is_wall = 1;
  }

  log_sensorfr = sensor[0];       // log buff

  sensor[1] = sensorH[1] - sensorL[1];  // Measures against external light

  if ( sensor[1] <= 2100 ){
    sensor_sider.is_wall = 0;
  } else {
    sensor_sider.is_wall = 1;
  }

  log_sensorsr = sensor[1];       // log_buff

  sensor[2] = sensorH[2] - sensorL[2];  // Measures against external light

  if ( sensor[2] <= 2700 ){
    sensor_sidel.is_wall = 0;
  } else {
    sensor_sidel.is_wall = 1;
  }

  log_sensorsl = sensor[2];     // log buff

  sensor[3] = sensorH[3] - sensorL[3];  // Measures against external light

  if ( sensor[3] <= 2900 ){
    sensor_frontl.is_wall = 0;
  } else {
    sensor_frontl.is_wall = 1;
  }

  log_sensorfl = sensor[3];     // log buff

  sensor_sider.error = sensor[1] - 2600;
  sensor_sidel.error = sensor[2] - 2900;

  //sensor_frontr.error = sensor[0] - 2300;
  //sensor_frontr.error = sensor[3] - 2300;

}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
