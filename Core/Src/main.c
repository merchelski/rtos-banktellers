/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "rng.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TOTAL_SIM_TIME_MIN (420)
#define SIM_MIN_TO_MS(minutes) ((minutes) * (100))
#define SIM_SEC_TO_MS(sec) ((sec) * (1.6667))
#define NUM_SEGMENT_DIGITS (4)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for updateSegment */
osThreadId_t updateSegmentHandle;
const osThreadAttr_t updateSegment_attributes = {
  .name = "updateSegment",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for genCustomer */
osThreadId_t genCustomerHandle;
const osThreadAttr_t genCustomer_attributes = {
  .name = "genCustomer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for teller01 */
osThreadId_t teller01Handle;
const osThreadAttr_t teller01_attributes = {
  .name = "teller01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for teller02 */
osThreadId_t teller02Handle;
const osThreadAttr_t teller02_attributes = {
  .name = "teller02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for teller03 */
osThreadId_t teller03Handle;
const osThreadAttr_t teller03_attributes = {
  .name = "teller03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for customerQueue */
osMessageQueueId_t customerQueueHandle;
const osMessageQueueAttr_t customerQueue_attributes = {
  .name = "customerQueue"
};
/* Definitions for Mutex01 */
osMutexId_t Mutex01Handle;
const osMutexAttr_t Mutex01_attributes = {
  .name = "Mutex01"
};
/* USER CODE BEGIN PV */
int32_t SIMULATED_TIME_START;
uint16_t CUSTOMER_QUEUE_COUNT = 1234;
uint8_t SEGMENT_DIGIT[4] = {
		0b00001000, // rightmost
		0b00000100, // left of rightmost
		0b00000010, // right of leftmost
		0b00000001	// leftmost
};
uint8_t SEGMENT_NUM[10] = {
		0b11000000, // 0
		0b11111001, // 1
		0b10100100, // 2
		0b10110000, // 3
		0b10011001, // 4
		0b10010010, // 5
		0b10000010, // 6
		0b11111000, // 7
		0b10000000, // 8
		0b10010000  // 9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartUpdateSegment(void *argument);
void StartGenCustomerTask(void *argument);
void StartTeller01(void *argument);
void StartTeller02(void *argument);
void StartTeller03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
* @brief Callback for S1-S3 SHIELD BUTTONS interrupt.
* @param GPIO_Pin: The GPIO_Pin of the button that generated the interrupt.
* @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t buffer[100];
	int data_size;
	data_size = sprintf((char*)buffer, "A BUTTON PRESSED!!\r\n");
	HAL_UART_Transmit(&huart2, buffer, data_size, 100U);
	if(GPIO_Pin == S1_SHLD_BUTTON_Pin)
	{
		data_size = sprintf((char*)buffer, "S1 BUTTON PRESSED!!\r\n");
		HAL_UART_Transmit(&huart2, buffer, data_size, 100U);
	}

	if(GPIO_Pin == S2_SHLD_BUTTON_Pin)
	{
		data_size = sprintf((char*)buffer, "S2 BUTTON PRESSED!!\r\n");
		HAL_UART_Transmit(&huart2, buffer, data_size, 100U);
	}

	if(GPIO_Pin == S3_SHLD_BUTTON_Pin)
	{
		data_size = sprintf((char*)buffer, "S3 BUTTON PRESSED!!\r\n");
		HAL_UART_Transmit(&huart2, buffer, data_size, 100U);
	}
}

/**
* @brief Sets 7-segment pin values using a shift register.
*
* @param data_port: The port of the GPIO pin associated with the data of the display.
* @param data_pin: The GPIO data pin.
* @param clock_port: The port of the GPIO pin associated with the clock used to operate shift register.
* @param clock_pin: The GPIO clock pin.
* @param value: The value to shift into the data pin.
* @retval None
*/
static inline void shiftOut(GPIO_TypeDef* data_port, uint16_t data_pin, GPIO_TypeDef* clock_port, uint16_t clock_pin, uint8_t value) {
	for(int ii=0x80; ii; ii>>=1) {
		HAL_GPIO_WritePin(clock_port, clock_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(data_port, data_pin, (value&ii)!=0);
		HAL_GPIO_WritePin(clock_port, clock_pin, GPIO_PIN_SET);
	}
}


/**
* @brief Helper function which activates appropriate segments for a particular digit in the 7-segment display.
*
* @param digit: The digit to set (four in total).
* @param value: The value to set.
* @retval None
*/
static inline void set_segment_digit(uint8_t digit, uint8_t value)
{
	HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
	shiftOut(SHLD_D8_SEG7_Data_GPIO_Port, SHLD_D8_SEG7_Data_Pin, SHLD_D7_SEG7_Clock_GPIO_Port, SHLD_D7_SEG7_Clock_Pin, value);
	shiftOut(SHLD_D8_SEG7_Data_GPIO_Port, SHLD_D8_SEG7_Data_Pin, SHLD_D7_SEG7_Clock_GPIO_Port, SHLD_D7_SEG7_Clock_Pin, digit);
	HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
}

/*
* @brief Sets the 7-segment display value.
*
* @param num: The four digit number to set the display to (MAX: 9999).
* @retval None
*/
void set_segment_display(uint16_t num)
{
	uint8_t digit = 0;
	uint8_t value;
	while((num > 0) && (digit < NUM_SEGMENT_DIGITS))
	{
		value = num % 10;
		set_segment_digit(SEGMENT_DIGIT[digit], SEGMENT_NUM[value]);
		num /= 10;
		digit++;
	}
	set_segment_digit(0, 0);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Mutex01 */
  Mutex01Handle = osMutexNew(&Mutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of customerQueue */
  customerQueueHandle = osMessageQueueNew (421, sizeof(uint16_t), &customerQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of updateSegment */
  updateSegmentHandle = osThreadNew(StartUpdateSegment, NULL, &updateSegment_attributes);

  /* creation of genCustomer */
  genCustomerHandle = osThreadNew(StartGenCustomerTask, NULL, &genCustomer_attributes);

  /* creation of teller01 */
  teller01Handle = osThreadNew(StartTeller01, NULL, &teller01_attributes);

  /* creation of teller02 */
  teller02Handle = osThreadNew(StartTeller02, NULL, &teller02_attributes);

  /* creation of teller03 */
  teller03Handle = osThreadNew(StartTeller03, NULL, &teller03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // Grab reference point to get an accurate reading of simulated time.
  SIMULATED_TIME_START = HAL_GetTick();

//  osThreadSuspend(genCustomerHandle);
  osThreadSuspend(teller01Handle);
  osThreadSuspend(teller02Handle);
  osThreadSuspend(teller03Handle);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();


  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SHLD_D7_SEG7_Clock_Pin|SHLD_D8_SEG7_Data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_SHLD_BUTTON_Pin S2_SHLD_BUTTON_Pin */
  GPIO_InitStruct.Pin = S1_SHLD_BUTTON_Pin|S2_SHLD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S3_SHLD_BUTTON_Pin */
  GPIO_InitStruct.Pin = S3_SHLD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(S3_SHLD_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_D7_SEG7_Clock_Pin SHLD_D8_SEG7_Data_Pin */
  GPIO_InitStruct.Pin = SHLD_D7_SEG7_Clock_Pin|SHLD_D8_SEG7_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D4_SEG7_Latch_Pin */
  GPIO_InitStruct.Pin = SHLD_D4_SEG7_Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SHLD_D4_SEG7_Latch_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  // S1_SHLD_BUTTON
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  // S2_SHLD_BUTTON
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  // S3_SHLD_BUTTON
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUpdateSegment */
/**
* @brief Function implementing the updateSegment thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdateSegment */
void StartUpdateSegment(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	set_segment_display(CUSTOMER_QUEUE_COUNT);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGenCustomerTask */
/**
* @brief Function implementing the genCustomer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGenCustomerTask */
void StartGenCustomerTask(void *argument)
{
  /* USER CODE BEGIN StartGenCustomerTask */
  /* Infinite loop */

  for(;;)
  {
	CUSTOMER_QUEUE_COUNT--; // testing purposes
	osDelay(100);
  }
  /* USER CODE END StartGenCustomerTask */
}

/* USER CODE BEGIN Header_StartTeller01 */
/**
* @brief Function implementing the teller01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTeller01 */
void StartTeller01(void *argument)
{
  /* USER CODE BEGIN StartTeller01 */
  /* Infinite loop */
  uint8_t buffer[100];
  int n;
  for(;;)
  {
	n = sprintf((char*)buffer, "HEY ITS TELLER 01!\r\n");
	osMutexAcquire(Mutex01Handle, osWaitForever);
	HAL_UART_Transmit(&huart2, buffer, n, 10U);
	osMutexRelease(Mutex01Handle);
    osDelay(100);
  }
  /* USER CODE END StartTeller01 */
}

/* USER CODE BEGIN Header_StartTeller02 */
/**
* @brief Function implementing the teller02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTeller02 */
void StartTeller02(void *argument)
{
  /* USER CODE BEGIN StartTeller02 */
  /* Infinite loop */
  uint8_t buffer[100];
  int n;
  for(;;)
  {
	n = sprintf((char*)buffer, "HEY ITS TELLER 02!\r\n");
	osMutexAcquire(Mutex01Handle, osWaitForever);
	HAL_UART_Transmit(&huart2, buffer, n, 10U);
	osMutexRelease(Mutex01Handle);
	osDelay(100);
  }
  /* USER CODE END StartTeller02 */
}

/* USER CODE BEGIN Header_StartTeller03 */
/**
* @brief Function implementing the teller03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTeller03 */
void StartTeller03(void *argument)
{
  /* USER CODE BEGIN StartTeller03 */
  /* Infinite loop */
  uint8_t buffer[100];
  int n;
  for(;;)
  {
	n = sprintf((char*)buffer, "HEY ITS TELLER 03!");
	osMutexAcquire(Mutex01Handle, osWaitForever);
	HAL_UART_Transmit(&huart2, buffer, n, 10U);
	osMutexRelease(Mutex01Handle);
    osDelay(100);
  }
  /* USER CODE END StartTeller03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
