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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

#include "rng.h"
#include "info.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TOTAL_TIME_IN_DAY_MIN ((7*60UL))

#define SIM_MIN_TO_MS(minutes) ((minutes) * (100UL))
#define SIM_SEC_TO_MS(sec) ((secUL) * (1.6667)

#define MS_TO_SIM_MIN(ms) ((ms) / 100UL)
#define MS_TO_SIM_HOURS(ms) ((ms) / (100*60UL))

#define TOTAL_SIM_TIME_MS (SIM_MIN_TO_MS(TOTAL_TIME_IN_DAY_MIN))

#define NUM_SEGMENT_DIGITS (4)

#define MAX(a,b) \
	({ __typeof__ (a) _a = (a); \
	   __typeof__ (b) _b = (b); \
	 _a > _b ? _a : _b; })
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
/* Definitions for simMonitorInfo */
osThreadId_t simMonitorInfoHandle;
const osThreadAttr_t simMonitorInfo_attributes = {
  .name = "simMonitorInfo",
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
uint32_t SIMULATED_TIME_START;

// 7-segment display digits
uint8_t SEGMENT_DIGIT[4] = {
		0b00001000, // Rightmost
		0b00000100, // Left of rightmost
		0b00000010, // Right of leftmost
		0b00000001	// Leftmost
};

// Display a single digit number
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


TELLER_INFO teller01_info;
TELLER_INFO teller02_info;
TELLER_INFO teller03_info;


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
void StartSimMonitorInfo(void *argument);

/* USER CODE BEGIN PFP */
void teller_functionality(TELLER_INFO* teller_info, osThreadId_t tellerHandler);
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
//	uint8_t buffer[100];
//	int data_size;
//	data_size = sprintf((char*)buffer, "A BUTTON PRESSED!!\r\n");
//	HAL_UART_Transmit(&huart2, buffer, data_size, 100U);

	TELLER_INFO* teller;
//	char* teller_name;

	// First button corresponds to teller01
	if(GPIO_Pin == S1_SHLD_BUTTON_Pin)
	{
		teller = &teller01_info;
//		teller_name = "TELLER01";
	}

	// Second button corresponds to teller02
	if(GPIO_Pin == S2_SHLD_BUTTON_Pin)
	{
		teller = &teller02_info;
//		teller_name = "TELLER02";
	}

	// Third button corresponds to teller03
	if(GPIO_Pin == S3_SHLD_BUTTON_Pin)
	{
		teller = &teller03_info;
//		teller_name = "TELLER03";
	}

	// Callback is triggered by both the rising AND falling edge, so we just toggle.
	teller->is_on_forced_break ^= 1;

//	if(teller->is_on_forced_break) // Means just toggled, so teller was told to go on break.
//	{
//		data_size = sprintf((char*)buffer, "%s IS BEING YELLED AT TO GO ON AND STAY ON A FORCED BREAK!!\r\n", teller_name);
//	}
//	else
//	{
//		data_size = sprintf((char*)buffer, "%s YELLING HAS STOPPED!!\r\n", teller_name);
//	}
//
//	// Send message - for testing purposes right now...
//	HAL_UART_Transmit(&huart2, buffer, data_size, 100U);
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
* @param digit: The digit to set (four in total)	.
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
	while(digit < NUM_SEGMENT_DIGITS)
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

  // Initialize tellers
  init_teller(&teller01_info);
  init_teller(&teller02_info);
  init_teller(&teller03_info);

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
  customerQueueHandle = osMessageQueueNew (421, sizeof(uint64_t), &customerQueue_attributes);

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

  /* creation of simMonitorInfo */
  simMonitorInfoHandle = osThreadNew(StartSimMonitorInfo, NULL, &simMonitorInfo_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // Grab reference point to get an accurate reading of simulated time.
  SIMULATED_TIME_START = HAL_GetTick();


  /* ----- TESTING THREADS -----*/
//  osThreadSuspend(genCustomerHandle);
//  osThreadSuspend(teller01Handle);
  osThreadSuspend(teller02Handle);
  osThreadSuspend(teller03Handle);
//  osThreadSuspend(updateSegmentHandle);

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
void teller_functionality(TELLER_INFO* teller_info, osThreadId_t tellerHandler)
{
	CUSTOMER_INFO current_customer;
	osStatus_t queue_status;
	uint32_t queue_time;
	uint32_t break_time;
	uint32_t wait_time;
	uint32_t forced_break_start_time;

	// For testing - temporary
	uint8_t buffer[100];
	int data_size;

	for(;;)
	{
		// Waiting for a customer to arrive, this is different than taking a break.
		if(teller_info->waiting_flag == true)
		{
			osDelay(1);
		}

		// Stop when the day ends.
		if(HAL_GetTick() >= (TOTAL_SIM_TIME_MS + SIMULATED_TIME_START))
		{
			osThreadSuspend(tellerHandler);
		}

		// Get next customer from the queue.
		queue_status = osMessageQueueGet(customerQueueHandle, &current_customer, 0, osWaitForever);
		if(queue_status == osOK) // retrieved
		{
			// If this is the first customer after waiting...
			if(teller_info->waiting_flag == true)
			{
				// ...record the time spent waiting. The teller is no longer waiting.
				teller_info->waiting_flag = false;
				wait_time = HAL_GetTick() - teller_info->last_waiting_start_time;
				teller_info->max_wait_time = MAX(teller_info->max_wait_time, wait_time);
				teller_info->total_wait_time += wait_time;
			}

			// Record statistic about customer...
			teller_info->total_customers_serviced++;
			teller_info->total_service_time += current_customer.service_time;
			teller_info->max_transaction_time = MAX(teller_info->max_transaction_time, current_customer.service_time);

			// This is time the customer had to wait in the queue before being serviced
			queue_time = HAL_GetTick() - current_customer.time_entered_queue;


			// Access global variables to update customer queue time statistics
			osMutexAcquire(Mutex01Handle, osWaitForever);
			max_customer_queue_time = MAX(max_customer_queue_time, queue_time);
			total_customer_queue_time += queue_time;
			osMutexRelease(Mutex01Handle);

			// Customer service delay
			osDelay(current_customer.service_time);
		}
		else if(queue_status == osErrorResource) // Empty queue -> teller is waiting for next customer.
		{
			teller_info->waiting_flag = true;

			// Record when the teller started waiting to calculate total time waiting later.
			teller_info->last_waiting_start_time = HAL_GetTick();
		}

		/* --- Check for breaks only after finishing with a customer --- */

		// Forced break - takes priority over natural break
		if(teller_info->is_on_forced_break == true)
		{
			data_size = sprintf((char*)buffer, "A TELLER IS GOING ON A FORCED BREAK!!\r\n");
			HAL_UART_Transmit(&huart2, buffer, data_size, 100U);

			// Grab reference point
			forced_break_start_time = HAL_GetTick();

			// Stay until the forced break is released
			while(teller_info->is_on_forced_break == true)
			{
				osDelay(100); // Decently long delay - can be adjusted
			}

			// Record break time and updated statistics...
			break_time = HAL_GetTick() - forced_break_start_time;
			teller_info->max_break_time = MAX(teller_info->max_break_time, break_time);
			teller_info->total_break_time += break_time;
			teller_info->total_breaks_taken++;

			// ...teller was just on break, so recalculate next available natural break time
			teller_info->next_available_natural_break_time = HAL_GetTick() + rand_range(MIN_TELLER_BREAK_WAIT, MAX_TELLER_BREAK_WAIT);


		} // Natural break
		else if(HAL_GetTick() >= teller_info->next_available_natural_break_time)
		{

			// Generate break time and store statistics
			break_time = rand_range(MIN_TELLER_BREAK_TIME, MAX_TELLER_BREAK_TIME);
			teller_info->max_break_time = MAX(teller_info->max_break_time, break_time);
			teller_info->total_break_time += break_time;
			teller_info->total_breaks_taken++;
			teller_info->next_available_natural_break_time = HAL_GetTick() + rand_range(MIN_TELLER_BREAK_WAIT, MAX_TELLER_BREAK_WAIT);

			// Send message - for testing purposes right now...
			data_size = sprintf((char*)buffer, "A TELLER IS GOING ON A NATURAL BREAK!!\r\n");
			HAL_UART_Transmit(&huart2, buffer, data_size, 100U);

			// Go on break...
			osDelay(break_time);
		}
	}
}
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
	// Update 7-segment display
	set_segment_display(osMessageQueueGetCount(customerQueueHandle));

	// Need short delay for display to stay updated properly
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

  uint32_t delay_until_next_customer_arrival;
  CUSTOMER_INFO customer_template;
  osStatus_t queue_status;

  /* Infinite loop */
  for(;;)
  {
	// Stop when the day ends.
	if(HAL_GetTick() >= (TOTAL_SIM_TIME_MS + SIMULATED_TIME_START))
	{
		osThreadSuspend(genCustomerHandle);
	}

	// New customer arrives every 1-4 minutes
	delay_until_next_customer_arrival = rand_range(MIN_CUSTOMER_ARRIVAL_DELAY, MAX_CUSTOMER_ARRIVAL_DELAY);

	// Generate a new customer with randomized service time
	reset_and_init_customer(&customer_template);

	// Send customer to queue to be picked up by tellers
	queue_status = osMessageQueuePut(customerQueueHandle, &customer_template, 0U, osWaitForever);

	if(queue_status == osOK)
	{

		// Record queue statistics
		max_customer_queue_depth = MAX(max_customer_queue_depth, osMessageQueueGetCount(customerQueueHandle));

		// Simulate time between customers
		osDelay(delay_until_next_customer_arrival);
	}
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

  // All tellers share the same functionality
  teller_functionality(&teller01_info, teller01Handle);

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

  // All tellers share the same functionality
  teller_functionality(&teller02_info, teller02Handle);

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

  // All tellers share the same functionality
  teller_functionality(&teller03_info, teller03Handle);

  /* USER CODE END StartTeller03 */
}

/* USER CODE BEGIN Header_StartSimMonitorInfo */
/**
* @brief Function implementing the simMonitorInfo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSimMonitorInfo */
void StartSimMonitorInfo(void *argument)
{
  /* USER CODE BEGIN StartSimMonitorInfo */
  /* Infinite loop */
  uint8_t buffer[100];
  int data_size;
  uint32_t sim_min;
  uint32_t sim_hours;
  uint32_t current_time_ms;
  for(;;)
  {
	current_time_ms = HAL_GetTick();

	sim_hours = MS_TO_SIM_HOURS(current_time_ms);
	sim_min = MS_TO_SIM_MIN(current_time_ms);

	data_size = sprintf((char*)buffer, "CURRENT TIME: %02ld:%02ld\r\nTELLER01 Status:\r\n", ((sim_hours + 8) % 12) + 1, sim_min % 60);
	HAL_UART_Transmit(&huart2, buffer, data_size, 100U);

	// Stop when the day ends.
	if(HAL_GetTick() >= (TOTAL_SIM_TIME_MS + SIMULATED_TIME_START))
	{
		osThreadSuspend(simMonitorInfoHandle);
	}
    osDelay(100);
  }
  /* USER CODE END StartSimMonitorInfo */
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
