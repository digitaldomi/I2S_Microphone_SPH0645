/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SAI_HandleTypeDef hsai_BlockA2;
DMA_HandleTypeDef hdma_sai2_a;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint8_t counter;

#define samples_n 100
#define bytes_per_sample 8 //2 channels 32bit each
#define bytes_per_channel bytes_per_sample/2 // 4

uint8_t audio_output_byte_stream[bytes_per_channel*samples_n+1];

uint32_t sampling_index = 0;

uint8_t i2s_input_buffer_valid = 0;
uint8_t i2s_input_buffer[bytes_per_sample*samples_n+bytes_per_channel]; //2 times n samples of 24bit (3*8bit) + 3 bit for last data because 6byte is read each time L+R


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SAI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if (htim == &htim6)
   {
	   //HAL_GPIO_TogglePin(MIC_LR_GPIO_Port, MIC_LR_Pin);
	   HAL_SAI_Receive_IT(&hsai_BlockA2, i2s_input_buffer+(sampling_index*bytes_per_channel), bytes_per_sample);
	   HAL_TIM_Base_Start_IT(&htim6);
   }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
//	int32_t sample = i2s_input_buffer[0] << 24 |  i2s_input_buffer[1] << 16 |  i2s_input_buffer[2] << 8 | i2s_input_buffer[3];
//	int32_t sample2 = i2s_input_buffer[4] << 24 |  i2s_input_buffer[5] << 16 |  i2s_input_buffer[6] << 8 | i2s_input_buffer[7];
//
//	sample /= 256;
//	sample2 /= 256;
//
//	//sample /= 256;
//	HAL_GPIO_ReadPin(MIC_LR_GPIO_Port, MIC_LR_Pin) == sample;
// useless code, just use to check values' format...




	sampling_index++;

	if(sampling_index==samples_n){
		i2s_input_buffer_valid = 1;
	}else if(sampling_index >=2*samples_n){
		i2s_input_buffer_valid = 2;
		sampling_index = 0;
	}
}

void DMA_I2S_Callback(){

}

void DMA_UART2_Callback(){

}


void transmitBlock(uint8_t offset){
	/*uint32_t sample;
	for(int i = 0; i < samples_n; i++){
		sample = samples[i+offset];
		audio_byte_stream[3*i+1] = sample >> (24 -8);
		audio_byte_stream[3*i+2] = sample >> (16 -8);
		audio_byte_stream[3*i+3] = sample >> (8  -8);
	}
	HAL_UART_Transmit(&huart2, audio_byte_stream, 3*samples_n+1, 100);*/
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SAI2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */

  HAL_DMA_RegisterCallback(&hdma_sai2_a, 1, DMA_I2S_Callback);
  HAL_DMA_RegisterCallback(&hdma_usart2_tx, 2, DMA_UART2_Callback);


  HAL_TIM_Base_Start_IT(&htim6); //Start 1ms sampling timer


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*
	  if(nSamplesBuffered == 1){
		  nSamplesBuffered=0;
		  transmitBlock(0);
	  }else if(nSamplesBuffered == 2){
		  nSamplesBuffered=0;
		  transmitBlock(100);
	  }else{

	  }
	  HAL_Delay(1);
	  uint32_t store;
	  HAL_SAI_Receive_IT(&hsai_BlockA2, store, 4);
*/


	  if(i2s_input_buffer_valid == 1){
		  i2s_input_buffer_valid = 0;
		  audio_output_byte_stream[0] = 0xFE;
		  for(int i = 0; i < samples_n; i++){
			  //HAL_GPIO_TogglePin(MIC_LR_GPIO_Port, MIC_LR_Pin);
			  //HAL_GPIO_TogglePin(MIC_LR_READ_GPIO_Port, MIC_LR_READ_Pin);
			  audio_output_byte_stream[3*i+1] = i2s_input_buffer[bytes_per_channel*i]; //transmit 3 data bytes MSB
			  audio_output_byte_stream[3*i+2] = i2s_input_buffer[bytes_per_channel*i+1];
			  audio_output_byte_stream[3*i+3] = i2s_input_buffer[bytes_per_channel*i+2]; //LSB
		  }
		  HAL_UART_Transmit_IT(&huart2, audio_output_byte_stream, 3*samples_n+1);
		  //HAL_UART_Transmit_DMA(&huart2, buffer1, 3*samples_n+1);

	  }else if(i2s_input_buffer_valid == 2){
		  i2s_input_buffer_valid = 0;
		  audio_output_byte_stream[0] = 0xFE;
		  uint16_t offset = 3*samples_n;
		  for(int i = 0; i < samples_n; i++){
			  //HAL_GPIO_TogglePin(MIC_LR_GPIO_Port, MIC_LR_Pin);
			  //HAL_GPIO_TogglePin(MIC_LR_READ_GPIO_Port, MIC_LR_READ_Pin);
			  audio_output_byte_stream[3*i+1] = i2s_input_buffer[offset + bytes_per_channel*i]; //transmit 3 data bytes MSB
			  audio_output_byte_stream[3*i+2] = i2s_input_buffer[offset + bytes_per_channel*i+1];
			  audio_output_byte_stream[3*i+3] = i2s_input_buffer[offset + bytes_per_channel*i+2]; //LSB
		  }
		  HAL_UART_Transmit_IT(&huart2, audio_output_byte_stream, 3*samples_n+1);
	  }

	  //HAL_UART_Transmit(&huart2, '\n', 1, 10);
	  //HAL_GPIO_WritePin(MIC_LR_GPIO_Port, MIC_LR_Pin, 0);
	  //HAL_GPIO_TogglePin(MIC_LR_GPIO_Port, MIC_LR_Pin);



	  /*
	  counter++;
	  if(counter > 255){
		  counter = 0;
	  }


	  data[0] = 0xFE; //data frame leading 16bit

	  for(int i = 1; i < 301; i+=3){
		  counter++;
		  data[i]   = counter; //LSB
		  data[i+1] = counter >> 8;
		  data[i+2] = counter >> 16;
	  }

	  HAL_UART_Transmit(&huart2, data, 301, 100);
	  HAL_Delay(100);

*/



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_SAI2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_32K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MIC_LR_GPIO_Port, MIC_LR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MIC_LR_Pin */
  GPIO_InitStruct.Pin = MIC_LR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MIC_LR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
