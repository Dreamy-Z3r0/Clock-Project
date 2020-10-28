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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
	// DS3231 I2C Address
		uint8_t DS3231_ADDRESS = 0x68 << 1;

	/* I2C availability flag */
		_Bool I2C_available = 1;

	// Time data to write to DS3231
		uint8_t BCD_data_time[3];
		_Bool newTime = 0;

	// Date data to write to DS3231
		uint8_t BCD_data_date[7];
		_Bool newDate = 0;
		uint8_t BCD_data[7];

	/* I2C read request flag <- Pull time and date data from DS3231 */
		_Bool READ_NOW = 0;

	// Clock and Calendar data read from DS3231
		uint8_t BCD_readData[7];
	// Storage for time data to handle alarms
		uint8_t DEC_readData[2];

	/* Storage space for display digits */
		uint8_t dataOutput[3][4] =
		{
				{2, 0, 0, 0},		// YEAR
				{0, 0, 0, 0},		// TIME
				{0, 1, 0, 1}		// DATE
		};

	// Display digit handler
		uint8_t digitIndex = 0;

	// Alarm handler
		_Bool alarmON = 0;

	/* UART Busy Flag */
		_Bool UART_available = 1;

	/* UART1 transmit/receive command */
		uint8_t RECEIVED_MESSAGE = 0x00;

	/* Storage for data from UART1 */
		uint8_t RX_BUF[5];					// Received data
		uint8_t UART1_TRANSMIT_MESSAGE;		// Data to transmit

	/* UART data for transmission flags */
		_Bool Feedback_Message = 0;		// Availability of feedback message for ESP8266
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint8_t dec2bcd(uint8_t bin_input);
uint8_t bcd2dec(uint8_t bcd_input);

void displayDataUpdate(uint8_t digitToUpdate);
void singleDigitUpdate(void);

void DATA_EXTRACTION(void);
void clockAlarm(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
_Bool VOLUME_TEST = 0;
_Bool BUZZER_ON = 0;
uint8_t EXTI_INTERRUPT_COUNTER = 0;

_Bool ESP8266_RESET_FLAG = 0;
uint8_t ESP8266_RESET_COUNTER = 0;

uint16_t TIMER_PRELOAD[] = {10000, 819, 1168, 1104, 1435, 1175, 3232, 1336, 3234, 1300, 3317, 1035, 1091, 985, 1124, 1106};
uint8_t TIMER_INDEX = 0;
uint8_t TIMER_PRELOAD_SIZE = 0;

_Bool REPEATED = 0;

uint16_t COUNTER_PERIOD = 65535;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin, RESET);
  HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, SET);
  TIMER_PRELOAD_SIZE = sizeof(TIMER_PRELOAD) / sizeof(TIMER_PRELOAD[0]);

  // Initiate SQW at 1Hz on DS3231
  uint8_t SQW_INIT_COMMAND = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x0E, 1, &SQW_INIT_COMMAND, 1, 500);

  // Turn off display digits
  HAL_GPIO_WritePin(EN_DIGIT_1_GPIO_Port, EN_DIGIT_1_Pin, RESET);
  HAL_GPIO_WritePin(EN_DIGIT_2_GPIO_Port, EN_DIGIT_2_Pin, RESET);
  HAL_GPIO_WritePin(EN_DIGIT_3_GPIO_Port, EN_DIGIT_3_Pin, RESET);
  HAL_GPIO_WritePin(EN_DIGIT_4_GPIO_Port, EN_DIGIT_4_Pin, RESET);

  // Enable multiplexing clock
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

  // Enable UART1 to listen to data/requests from ESP8266, delay to neglect any boot message from ESP8266
  HAL_Delay(5000);
  HAL_UART_Receive_DMA(&huart1, RX_BUF, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (I2C_available)
	  {
		  if (newTime)
		  {
			  I2C_available = 0;
			  newTime = 0;
			  HAL_I2C_Mem_Write_DMA(&hi2c1, DS3231_ADDRESS, 0x00, 1, BCD_data_time, 3);
			  alarmON = 0;
		  }
		  else if (newDate)
		  {
			  I2C_available = 0;
			  newDate = 0;
			  HAL_I2C_Mem_Write_DMA(&hi2c1, DS3231_ADDRESS, 0x03, 1, BCD_data_date, 4);
		  }
		  else if (READ_NOW)
		  {
			  I2C_available = 0;
			  READ_NOW = 0;
			  HAL_I2C_Mem_Read_DMA(&hi2c1, DS3231_ADDRESS, 0x00, 1, BCD_readData, 7);
		  }
	  }

	  if (UART_available)
	  {
		  if (Feedback_Message)
		  {
			  UART_available = 0;
			  Feedback_Message = 0;
			  HAL_UART_Transmit_DMA(&huart1, &UART1_TRANSMIT_MESSAGE, 1);
		  }
		  else
		  {
			  UART_available = 0;
			  HAL_UART_Receive_DMA(&huart1, RX_BUF, 5);			// Continue listening to data/requests from ESP8266
		  }
	  }

	  if (VOLUME_TEST ^ BUZZER_ON)
	  {
		  if (VOLUME_TEST)
		  {
			  BUZZER_ON = 1;

			  TIMER_INDEX = 0;
			  COUNTER_PERIOD = TIMER_PRELOAD[TIMER_INDEX];

			  MX_TIM3_Init();
			  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

			  HAL_GPIO_WritePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin, RESET);
			  HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, SET);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin, RESET);
			  HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, SET);

			  HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
		  }
	  }
	  else if (alarmON & (255 == TIMER_INDEX))
	  {
		  TIMER_INDEX = 0;
		  COUNTER_PERIOD = TIMER_PRELOAD[TIMER_INDEX];

		  MX_TIM3_Init();
		  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

		  HAL_GPIO_WritePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin, RESET);
		  HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, SET);
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 57600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 57600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = COUNTER_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = COUNTER_PERIOD;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LATCH_Pin|CLOCK_Pin|YEAR_Pin|TIME_Pin
                          |DATE_Pin|DP_OUT_Pin|ALARM_PWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_DIGIT_1_Pin|EN_DIGIT_2_Pin|EN_DIGIT_3_Pin|EN_DIGIT_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUILTIN_LED_Pin */
  GPIO_InitStruct.Pin = BUILTIN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUILTIN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LATCH_Pin CLOCK_Pin YEAR_Pin TIME_Pin
                           DATE_Pin */
  GPIO_InitStruct.Pin = LATCH_Pin|CLOCK_Pin|YEAR_Pin|TIME_Pin
                          |DATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DP_OUT_Pin ALARM_PWM_Pin */
  GPIO_InitStruct.Pin = DP_OUT_Pin|ALARM_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP8266_RESET_Pin */
  GPIO_InitStruct.Pin = ESP8266_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESP8266_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_DIGIT_1_Pin EN_DIGIT_2_Pin EN_DIGIT_3_Pin EN_DIGIT_4_Pin */
  GPIO_InitStruct.Pin = EN_DIGIT_1_Pin|EN_DIGIT_2_Pin|EN_DIGIT_3_Pin|EN_DIGIT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VOLUME_TEST_Pin */
  GPIO_InitStruct.Pin = VOLUME_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VOLUME_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void displayDataUpdate(uint8_t digitToUpdate)
{
	HAL_GPIO_WritePin (LATCH_GPIO_Port, LATCH_Pin, RESET);	// Pull LATCH pin LOW to push data to shift registers

	for (uint8_t index = 0; index <= 3; index++)
	{
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, RESET);	// Prepares for next bit input by pulling CLOCK pin LOW
		HAL_GPIO_WritePin( YEAR_GPIO_Port,  YEAR_Pin, (dataOutput[0][digitToUpdate] & (1 << index)) >> index);
		HAL_GPIO_WritePin( TIME_GPIO_Port,  TIME_Pin, (dataOutput[1][digitToUpdate] & (1 << index)) >> index);
		HAL_GPIO_WritePin( DATE_GPIO_Port,  DATE_Pin, (dataOutput[2][digitToUpdate] & (1 << index)) >> index);
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, SET);		// Shifts next input bit by pulling CLOCK pin HIGH
	}
}

void singleDigitUpdate(void)
{
	// Pull LATCH pin HIGH to update output on shift registers
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, SET);

	// Turn on updated digits
	if (0 == digitIndex)
	{
		HAL_GPIO_WritePin(EN_DIGIT_4_GPIO_Port, EN_DIGIT_4_Pin, RESET);
		HAL_GPIO_WritePin(EN_DIGIT_1_GPIO_Port, EN_DIGIT_1_Pin, SET);
	}
	else if (1 == digitIndex)
	{
		HAL_GPIO_WritePin(EN_DIGIT_1_GPIO_Port, EN_DIGIT_1_Pin, RESET);
		HAL_GPIO_WritePin(EN_DIGIT_2_GPIO_Port, EN_DIGIT_2_Pin, SET);
	}
	else if (2 == digitIndex)
	{
		HAL_GPIO_WritePin(EN_DIGIT_2_GPIO_Port, EN_DIGIT_2_Pin, RESET);
		HAL_GPIO_WritePin(EN_DIGIT_3_GPIO_Port, EN_DIGIT_3_Pin, SET);
	}
	else if (3 == digitIndex)
	{
		HAL_GPIO_WritePin(EN_DIGIT_3_GPIO_Port, EN_DIGIT_3_Pin, RESET);
		HAL_GPIO_WritePin(EN_DIGIT_4_GPIO_Port, EN_DIGIT_4_Pin, SET);
	}

	// Prepare multiplexing on the next digits
	digitIndex += 1;
	if (4 == digitIndex)
		digitIndex = 0;
}

uint8_t dec2bcd(uint8_t bin_input)		// Converts number from decimal format to BCD format
{
	return (((bin_input / 10) << 4) | (bin_input % 10));
}

uint8_t bcd2dec(uint8_t bcd_input)		// Converts number from BCD format to decimal format
{
	return (((bcd_input & 0xF0) >> 4) * 10) + (bcd_input & 0x0F);
}

void DATA_EXTRACTION(void)		// Update dataOutput[][] every time data is pulled from DS3231
{
	// Extract year digits
	dataOutput[0][2] = (BCD_readData[6] & 0xF0) >> 4;	// Tens digit from DS3231 year data in BCD format
	dataOutput[0][3] =  BCD_readData[6] & 0x0F;			// Unit digit from DS3231 year data in BCD format

	// Extract hour digits
	dataOutput[1][0] = (BCD_readData[2] & 0x30) >> 4;	// Tens digit from DS3231 hour data in BCD format
	dataOutput[1][1] =  BCD_readData[2] & 0x0F;			// Unit digit from DS3231 hour data in BCD format

	// Extract minute digits
	dataOutput[1][2] = (BCD_readData[1] & 0xF0) >> 4;	// Tens digit from DS3231 minute data in BCD format
	dataOutput[1][3] =  BCD_readData[1] & 0x0F;			// Unit digit from DS3231 minute data in BCD format

	// Extract date digits
	dataOutput[2][0] = (BCD_readData[4] & 0xF0) >> 4;	// Tens digit from DS3231 date data in BCD format
	dataOutput[2][1] =  BCD_readData[4] & 0x0F;			// Unit digit from DS3231 date data in BCD format

	// Extract month digits
	dataOutput[2][2] = (BCD_readData[5] & 0x10) >> 4;	// Tens digit from DS3231 month data in BCD format
	dataOutput[2][3] =  BCD_readData[5] & 0x0F;			// Unit digit from DS3231 month data in BCD format

	// Update alarm checking storage
	DEC_readData[0] = bcd2dec(BCD_readData[2]);		// Hour value
	DEC_readData[1] = bcd2dec(BCD_readData[1]);		// Minute value
}

void clockAlarm(void)	// Handler of alarm flag and alarm request
{
	// Check if current time is exactly 7:00, 11:00, 13:00, or 17:00 (neglecting seconds)
	_Bool AlarmCondition = 1;
	AlarmCondition &= (DEC_readData[1] == 0);
	AlarmCondition &= (((BCD_readData[2] & 0x0F) == 7) | (DEC_readData[0] == 11) | (DEC_readData[0] == 13));

	if (AlarmCondition) 	// Current time is 7:00, 11:00, 13:00, or 17:00
	{
		if ((!alarmON) & (!VOLUME_TEST))	// Alarm is off and volume test condition is FALSE
		{
			// Enable alarm
			alarmON = 1;
			TIMER_INDEX = 255;
			REPEATED = 0;
		}
	}
	else if (alarmON)		// Current time has passed alarm point
	{
		alarmON = 0;	// Disable alarm flag
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim)
{
	if (htim == &htim2)			// Multiplexing with ~5ms interval
	{
		displayDataUpdate(digitIndex);	// Push data for the next digit to be updated
		singleDigitUpdate();			// Enable the updated digit
	}
	else if (htim == &htim3)
	{
		TIMER_INDEX += 1;
		if (TIMER_PRELOAD_SIZE > TIMER_INDEX)
		{
			HAL_GPIO_TogglePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin);
			HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
			COUNTER_PERIOD = TIMER_PRELOAD[TIMER_INDEX];

			MX_TIM3_Init();
			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
		}
		else if (VOLUME_TEST)
		{
			HAL_GPIO_WritePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin, RESET);
			HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, SET);

			TIMER_INDEX = 0;
			COUNTER_PERIOD = TIMER_PRELOAD[TIMER_INDEX];

			MX_TIM3_Init();
			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
		}
		else if (!REPEATED)
		{
			HAL_GPIO_WritePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin, RESET);
			HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, SET);

			REPEATED = 1;
			TIMER_INDEX = 0;
			COUNTER_PERIOD = TIMER_PRELOAD[TIMER_INDEX];

			MX_TIM3_Init();
			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
		}
		else
		{
			HAL_GPIO_WritePin(ALARM_PWM_GPIO_Port, ALARM_PWM_Pin, RESET);
			HAL_GPIO_WritePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin, SET);
			HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == CLK_IN_Pin)
	{
		HAL_GPIO_TogglePin(DP_OUT_GPIO_Port, DP_OUT_Pin);
		EXTI_INTERRUPT_COUNTER += 1;

		if (5 == EXTI_INTERRUPT_COUNTER)	// Pull clock and calendar data from DS3231 every 5s
		{
			EXTI_INTERRUPT_COUNTER = 0;		// Reset counter
			READ_NOW = 1;					// Enable read flag
		}

		if (ESP8266_RESET_FLAG)
		{
			ESP8266_RESET_COUNTER += 1;

			if (5 == ESP8266_RESET_COUNTER)
			{
				ESP8266_RESET_FLAG = 0;
				HAL_UART_DMAResume(&huart1);
			}
		}
	}
	else if (GPIO_Pin == ESP8266_RESET_Pin)
	{
		HAL_UART_DMAPause(&huart1);
		ESP8266_RESET_COUNTER = 0;
		ESP8266_RESET_FLAG = 1;
	}
	else if (GPIO_Pin == VOLUME_TEST_Pin)
	{
		if (VOLUME_TEST)
		{
			VOLUME_TEST = 0;
		}
		else
		{
			VOLUME_TEST = 1;
			BUZZER_ON = 0;
		}
	}
	else
	{
		__NOP();
	}
}

void HAL_I2C_MemRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	DATA_EXTRACTION();
	clockAlarm();

	I2C_available = 1;
}

void HAL_I2C_MemTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	I2C_available = 1;
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart)
{
	UART_available = 1;
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{
	UART_available = 1;

	if (huart == &huart1)
	{
		_Bool dataCheck = 1;	// Validates received data from ESP8266
		if ('T' == RX_BUF[0])	// Data received from ESP8266 is time data
		{
			dataCheck &= ((0 <= RX_BUF[1]) & (RX_BUF[1] <= 23));	// RX_BUF[1] should be hour value ranging from 0 to 23
			dataCheck &= ((0 <= RX_BUF[2]) & (RX_BUF[2] <= 59));	// RX_BUF[2] should be minute value ranging from 0 to 59

			if (dataCheck)	// Data is valid
			{
				newTime = 10;	// Force disabling availability flag to update new data
				UART1_TRANSMIT_MESSAGE = 'O';	// Feedback message is 'ACKNOWLEDGED'

				BCD_data_time[0] = 55;					// Second = 10
				BCD_data_time[1] = dec2bcd(RX_BUF[2]);	// Get minute value in BCD format to update DS3231
				BCD_data_time[2] = dec2bcd(RX_BUF[1]);	// Get hour value in BCD format to update DS3231

				newTime = 1;	// New time data has been issued
			}
			else			// Data received is not valid
			{
				UART1_TRANSMIT_MESSAGE = 'r';	// Feedback message is a request for ESP8266 to re-send data
			}

		}
		else if ('D' == RX_BUF[0])	// Data received from ESP8266 is date data
		{
			dataCheck &= ((1 <= RX_BUF[1]) & (RX_BUF[1] <= 31));	// RX_BUF[1] should be date value ranging from 1 to 31
			dataCheck &= ((1 <= RX_BUF[2]) & (RX_BUF[2] <= 12));	// RX_BUF[2] should be month value ranging from 1 to 12

			uint16_t temp = (((uint16_t)RX_BUF[3]) << 8) | ((uint16_t)RX_BUF[4]);	// Acquire 16-bit year value
			dataCheck &= ((1980 <= temp) & (temp <= 2099));			// Year value should be from 1980 to 2099 (supported by DS3231)

			if (dataCheck)	// Data is valid
			{
				newDate = 0;	// Force disabling availability flag to update new data
				UART1_TRANSMIT_MESSAGE = 'O';	// Feedback message is 'ACKNOWLEDGED'

				BCD_data_date[1] = dec2bcd(RX_BUF[1]);		// Get date of month value in BCD format to update DS3231
				BCD_data_date[2] = dec2bcd(RX_BUF[2]);		// Get month value in BCD format to update DS3231
				BCD_data_date[3] = dec2bcd(temp - 2000);	// Get year value in BCD format to update DS3231 (Only years from 2000 to 2099 are valid)

				uint8_t DoW = (RX_BUF[1]+=RX_BUF[2]<3?temp--:temp-2,23*RX_BUF[2]/9+RX_BUF[1]+4+temp/4-temp/100+temp/400)%7;
				BCD_data_date[0] = DoW + 1;					// Day of week: 1 = Sunday, 2 = Monday, etc.

				newDate = 1;	// New date data has been issued
			}
			else			// Data received is not valid
			{
				UART1_TRANSMIT_MESSAGE = 'r';	// Feedback message is a request for ESP8266 to re-send data
			}
		}
		else if ('r' == RX_BUF[0])	// Unable to recognise what kind of data/request transmitted by ESP8266
		{
			__NOP();
		}
		else
		{
			UART1_TRANSMIT_MESSAGE = 'r';	// Feedback message is a request for ESP8266 to re-send data
		}

		// Enable transmitting feedback message to ESP8266
		Feedback_Message = 1;
	}
}

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
