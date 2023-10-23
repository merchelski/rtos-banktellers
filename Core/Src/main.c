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
#define SIM_SEC_TO_MS(sec) (((sec) * 5) / 3UL)

#define MS_TO_SIM_SEC(ms) (((ms) * 3) / 5UL)
#define MS_TO_SIM_MIN(ms) ((ms) / 100UL)
#define MS_TO_SIM_HOURS(ms) ((ms) / (100*60UL))

#define TOTAL_SIM_TIME_MS (SIM_MIN_TO_MS(TOTAL_TIME_IN_DAY_MIN))

#define NUM_SEGMENT_DIGITS (4)

#define MAX(a,b) \
	({ __typeof__ (a) _a = (a); \
	   __typeof__ (b) _b = (b); \
	 _a > _b ? _a : _b; })

#define MIN(a,b) \
	({ __typeof__ (a) _a = (a); \
	   __typeof__ (b) _b = (b); \
	 _a < _b ? _a : _b; })

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for updateSegment */
osThreadId_t updateSegmentHandle;
const osThreadAttr_t updateSegment_attributes = {
  .name = "updateSegment",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for genCustomer */
osThreadId_t genCustomerHandle;
const osThreadAttr_t genCustomer_attributes = {
  .name = "genCustomer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for teller01 */
osThreadId_t teller01Handle;
const osThreadAttr_t teller01_attributes = {
  .name = "teller01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for teller02 */
osThreadId_t teller02Handle;
const osThreadAttr_t teller02_attributes = {
  .name = "teller02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for teller03 */
osThreadId_t teller03Handle;
const osThreadAttr_t teller03_attributes = {
  .name = "teller03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for simMonitorInfo */
osThreadId_t simMonitorInfoHandle;
const osThreadAttr_t simMonitorInfo_attributes = {
  .name = "simMonitorInfo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
bool work_is_done = false;

uint32_t SIMULATED_TIME_START;

// 7-segment display digits.
const uint8_t SEGMENT_DIGIT[4] = {
		0b00001000, // Rightmost
		0b00000100, // Left of rightmost
		0b00000010, // Right of leftmost
		0b00000001	// Leftmost
};

// Display a single digit number.
const uint8_t SEGMENT_NUM[10] = {
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

// Converts teller's status to a string for display purposes.
const char* STATUS_TO_STR[5] = {"working", "waiting", "on break", "done for the day", "didn't show up..."};

// Stores all info related to tellers.
TELLER_INFO teller01_info;
TELLER_INFO teller02_info;
TELLER_INFO teller03_info;

// Used by monitor task.
uint8_t monitor_buffer[500];
int monitor_data_size;


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
void teller_functionality(TELLER_INFO* teller_info, osThreadId_t tellerHandler, GPIO_TypeDef* TELLER_GPIO_PORT, uint16_t TELLER_GPIO_PIN);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  // Initialize tellers
  init_teller(&teller01_info);
  init_teller(&teller02_info);
  init_teller(&teller03_info);

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


  /* ----- SUSPEND THREADS -----*/
//  osThreadSuspend(genCustomerHandle);
//  osThreadSuspend(teller01Handle);
//  osThreadSuspend(teller02Handle);
//  osThreadSuspend(teller03Handle);
//  osThreadSuspend(updateSegmentHandle);
//  osThreadSuspend(simMonitorInfoHandle);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S3_SHLD_BUTTON_Pin */
  GPIO_InitStruct.Pin = S3_SHLD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void teller_functionality(TELLER_INFO* teller_info, osThreadId_t tellerHandler, GPIO_TypeDef* TELLER_GPIO_PORT, uint16_t TELLER_GPIO_PIN)
{
	CUSTOMER_INFO current_customer;
	uint32_t customer_time_spent_in_queue;
	uint32_t break_time;
	uint32_t forced_break_start_time;

	uint32_t pre_wait_reference;
	uint32_t wait_discrepancy;
	for(;;)
	{
		// Teller starts working.
		teller_info->status = status_working;

		// Stop teller when the day ends.
		if((osMessageQueueGetCount(customerQueueHandle) == 0) && (HAL_GetTick() >= (TOTAL_SIM_TIME_MS + SIMULATED_TIME_START)))
		{
			teller_info->status = status_done_for_the_day;
			osThreadSuspend(tellerHandler);
		}

		/* --- First step is to obtain a customer --- */

		// [A: 1/2] We assume the teller will have to wait for a customer...
		teller_info->status = status_waiting;

		// [A: 2/2] ...so we grab a reference snapshot of the current time to calculate total wait later.
		pre_wait_reference = HAL_GetTick();

		// Attempt to grab the next customer from the queue.
		osMessageQueueGet(customerQueueHandle, &current_customer, 0, osWaitForever);

		// If the teller was able to unblock, it means a customer was successfully obtained -> teller is no longer waiting
		wait_discrepancy = HAL_GetTick() - pre_wait_reference;
		teller_info->status = status_working;

		// This is time the customer had to wait in the queue before being serviced.
		customer_time_spent_in_queue = HAL_GetTick() - current_customer.time_entered_queue;

		// [B: 1/2] If the time discrepancy is too high, it probably means the teller had to wait...
		if(wait_discrepancy > TELLER_WAIT_TOLERANCE)
		{
			// [B: 2/2] ...so update the teller's wait statistics.
			teller_info->total_wait_time += (wait_discrepancy - TELLER_WAIT_TOLERANCE);
			teller_info->total_waits_taken++;
			teller_info->max_wait_time = MAX(teller_info->max_wait_time, (wait_discrepancy - TELLER_WAIT_TOLERANCE));
		}

		/* --- Move on to process customer --- */

		// Record statistic about customer.
		teller_info->total_customers_serviced++;
		teller_info->total_service_time += current_customer.service_time;
		teller_info->max_service_time = MAX(teller_info->max_service_time, current_customer.service_time);


		// Access global variables to update customer queue time statistics.
		osMutexAcquire(Mutex01Handle, osWaitForever);
		max_customer_queue_time = MAX(max_customer_queue_time, customer_time_spent_in_queue);
		total_customer_queue_time += customer_time_spent_in_queue;
		osMutexRelease(Mutex01Handle);

		// Simulate time spent servicing customer.
		osDelay(current_customer.service_time);


		/* --- Check for breaks only after finishing with a customer --- */
		for(int i = 0; i < 2; i++) // check twice in case of chaining of natural break -> forced break
		{
			// Forced break -> takes priority over natural break.
			if(HAL_GPIO_ReadPin(TELLER_GPIO_PORT, TELLER_GPIO_PIN) == GPIO_PIN_RESET)
			{
				// Grab reference point
				forced_break_start_time = HAL_GetTick();
				teller_info->status = status_on_break;

				// Stay until the forced break is released
				while(HAL_GPIO_ReadPin(TELLER_GPIO_PORT, TELLER_GPIO_PIN) == GPIO_PIN_RESET)
				{
					osDelay(1);
				}

				// Break ends.
				teller_info->status = status_working;

				// [C: 1/2] Calculate break time and updated statistics...
				break_time = HAL_GetTick() - forced_break_start_time;
				teller_info->max_break_time = MAX(teller_info->max_break_time, break_time);
				teller_info->min_break_time = MIN(teller_info->min_break_time, break_time);
				teller_info->total_break_time += break_time;
				teller_info->total_breaks_taken++;

				// [C: 2/2] ...teller was just on break, so recalculate next available natural break time
				teller_info->next_available_natural_break_time = HAL_GetTick() + rand_range(MIN_TELLER_BREAK_WAIT, MAX_TELLER_BREAK_WAIT);


			} // Natural break.
			else if(HAL_GetTick() >= teller_info->next_available_natural_break_time)
			{

				// Generate a random break duration and update statistics.
				break_time = rand_range(MIN_TELLER_BREAK_TIME, MAX_TELLER_BREAK_TIME);
				teller_info->max_break_time = MAX(teller_info->max_break_time, break_time);
				teller_info->min_break_time = MIN(teller_info->min_break_time, break_time);
				teller_info->total_break_time += break_time;
				teller_info->total_breaks_taken++;

				// Generate next available break.
				teller_info->next_available_natural_break_time = HAL_GetTick() + rand_range(MIN_TELLER_BREAK_WAIT, MAX_TELLER_BREAK_WAIT);

				// Go on break.
				teller_info->status = status_on_break;
				osDelay(break_time);
			}
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
	// Stop generator when the day ends and all work is done.
	bool work_is_done = (osMessageQueueGetCount(customerQueueHandle) == 0) &&
						((eTaskGetState(teller01Handle) == eSuspended) ||
	  					(eTaskGetState(teller02Handle) == eSuspended) ||
						(eTaskGetState(teller03Handle) == eSuspended));

	if(work_is_done && (HAL_GetTick() >= (TOTAL_SIM_TIME_MS + SIMULATED_TIME_START)))
	{
		osThreadSuspend(updateSegmentHandle);
	}

	// [A: 1/2] Constantly update the 7-segment display...
	set_segment_display(osMessageQueueGetCount(customerQueueHandle));

	// [A: 2/2] ... need short delay for display to stay updated properly.
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

  /* Infinite loop */
  for(;;)
  {
	// Stop generator when the day ends.

	if(HAL_GetTick() >= (TOTAL_SIM_TIME_MS + SIMULATED_TIME_START))
	{
		osThreadSuspend(genCustomerHandle);
	}

	// New customer arrives after a random time within a set interval.
	delay_until_next_customer_arrival = rand_range(MIN_CUSTOMER_ARRIVAL_DELAY, MAX_CUSTOMER_ARRIVAL_DELAY);

	// Generate a new customer with randomized service time.
	reset_and_init_customer(&customer_template);

	// Send customer to queue to be picked up by tellers.
	osMessageQueuePut(customerQueueHandle, &customer_template, 0U, osWaitForever);

	// Record queue statistics.
	max_customer_queue_depth = MAX(max_customer_queue_depth, osMessageQueueGetCount(customerQueueHandle));

	// Simulate time between customers.
	osDelay(delay_until_next_customer_arrival);
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

  // All tellers share the same functionality.
  teller_functionality(&teller01_info, teller01Handle, S1_SHLD_BUTTON_GPIO_Port, S1_SHLD_BUTTON_Pin);

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

  // All tellers share the same functionality.
  teller_functionality(&teller02_info, teller02Handle, S2_SHLD_BUTTON_GPIO_Port, S2_SHLD_BUTTON_Pin);

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

  // All tellers share the same functionality.
  teller_functionality(&teller03_info, teller03Handle, S3_SHLD_BUTTON_GPIO_Port, S3_SHLD_BUTTON_Pin);

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
  uint32_t sim_min;
  uint32_t sim_hours;
  uint32_t current_time_ms;

  const char* teller01_info_str;
  const char* teller02_info_str;
  const char* teller03_info_str;

  uint32_t total_customers_served;
  uint32_t total_service_time;
  uint32_t total_wait_time;
  uint32_t total_num_waits;


  for(;;)
  {
	current_time_ms = HAL_GetTick();

	sim_hours = MS_TO_SIM_HOURS(current_time_ms);
	sim_min = MS_TO_SIM_MIN(current_time_ms);

	teller01_info_str = STATUS_TO_STR[teller01_info.status];
	teller02_info_str = STATUS_TO_STR[teller02_info.status];
	teller03_info_str = STATUS_TO_STR[teller03_info.status];

	max_customer_queue_depth = MAX(max_customer_queue_depth, osMessageQueueGetCount(customerQueueHandle));

	// [A: 1/2] Build up the real time data...
	monitor_data_size = sprintf((char*)monitor_buffer, "\r\n\r\nCURRENT TIME: %02ld:%02ld%s\r\nCustomers in queue: %ld\r\nTeller01 status: %s\r\nTeller02 status: %s\r\nTeller03 status: %s\r\n\r\n\r\n",
															(((sim_hours + 8) % 12) + 1),
															(sim_min % 60),
															(((sim_hours + 9) % 24) < 12 ? "am" : "pm"),
															osMessageQueueGetCount(customerQueueHandle),
															teller01_info_str,
															teller02_info_str,
															teller03_info_str);

	// [A: 2/2] ... and print it.
	HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

	// Stop when the day ends and all work is done.
	bool work_is_done = (osMessageQueueGetCount(customerQueueHandle) == 0) &&
							((eTaskGetState(teller01Handle) == eSuspended) ||
		  					(eTaskGetState(teller02Handle) == eSuspended) ||
							(eTaskGetState(teller03Handle) == eSuspended));
	if(work_is_done && (HAL_GetTick() >= (TOTAL_SIM_TIME_MS + SIMULATED_TIME_START)))
	{
		/* --- At the end of the day, gather all statistics --- */

		monitor_data_size = sprintf((char*)monitor_buffer, "---- WORK DAY STATISTICS ----\r\n\r\n");
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The total number of customers served during the day.
		total_customers_served = teller01_info.total_customers_serviced +
								 teller02_info.total_customers_serviced +
								 teller03_info.total_customers_serviced;

		monitor_data_size = sprintf((char*)monitor_buffer, "Total number of customers served during the day: %ld\r\n", total_customers_served);
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The number of customers served by Teller 1, by Teller 2, and by Teller 3.
		monitor_data_size = sprintf((char*)monitor_buffer, "The number of customers served by\r\n\tTeller01: %d\r\n\tTeller02: %d\r\n\tTeller03: %d\r\n",
				teller01_info.total_customers_serviced,
				teller02_info.total_customers_serviced,
				teller03_info.total_customers_serviced);

		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The average time each customer spends waiting in the queue.
		monitor_data_size = sprintf((char*)monitor_buffer, "The average time each customer spends waiting in the queue: %ld seconds\r\n",
				MS_TO_SIM_SEC((total_customer_queue_time / total_customers_served)));

		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The average time each customer spends with the teller.
		total_service_time = teller01_info.total_service_time +
							 teller02_info.total_service_time +
							 teller03_info.total_service_time;

		monitor_data_size = sprintf((char*)monitor_buffer, "The average time each customer spends with the teller: %ld seconds\r\n",
				MS_TO_SIM_SEC(total_service_time / total_customers_served));
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The average time tellers wait for customers.
		total_wait_time = teller01_info.total_wait_time +
						  teller02_info.total_wait_time +
						  teller03_info.total_wait_time;

		total_num_waits = teller01_info.total_waits_taken +
						  teller02_info.total_waits_taken +
						  teller03_info.total_waits_taken;

		monitor_data_size = sprintf((char*)monitor_buffer, "The average time tellers wait for customers: %ld seconds\r\n",
				MS_TO_SIM_SEC(total_wait_time / total_num_waits));
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The maximum customer wait time in the queue.
		monitor_data_size = sprintf((char*)monitor_buffer, "The maximum customer wait time in the queue: %ld seconds\r\n",
				MS_TO_SIM_SEC(max_customer_queue_time));
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The maximum wait time for tellers waiting for customers.
		monitor_data_size = sprintf((char*)monitor_buffer, "The maximum wait time for tellers waiting for customers: %ld seconds\r\n",
				MS_TO_SIM_SEC(MAX(MAX(teller01_info.max_wait_time, teller02_info.max_wait_time), teller03_info.max_wait_time)));
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The maximum transaction time for the tellers.
		monitor_data_size = sprintf((char*)monitor_buffer, "The maximum transaction time for the tellers: %ld seconds\r\n",
				MS_TO_SIM_SEC(MAX(MAX(teller01_info.max_service_time, teller02_info.max_service_time), teller03_info.max_service_time)));
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The maximum depth of the customer queue.
		monitor_data_size = sprintf((char*)monitor_buffer, "The maximum depth of the customer queue: %ld\r\n",
				max_customer_queue_depth);
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// The idle hook count.
		monitor_data_size = sprintf((char*)monitor_buffer, "The idle hook count: %" PRIu64 "\r\n",
				idle_hook_count);
		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// Number of breaks for each of the three tellers.
		monitor_data_size = sprintf((char*)monitor_buffer, "Number of breaks for each of the three tellers\r\n\tTeller01: %d\r\n\tTeller02: %d\r\n\tTeller03: %d\r\n",
				teller01_info.total_breaks_taken,
				teller02_info.total_breaks_taken,
				teller03_info.total_breaks_taken);

		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// Average break time for each of the three tellers.
		monitor_data_size = sprintf((char*)monitor_buffer, "Average break time for each of the three tellers\r\n\tTeller01: %ld seconds\r\n\tTeller02: %ld seconds\r\n\tTeller03: %ld seconds\r\n",
				MS_TO_SIM_SEC(teller01_info.total_break_time / teller01_info.total_breaks_taken),
				MS_TO_SIM_SEC(teller02_info.total_break_time / teller02_info.total_breaks_taken),
				MS_TO_SIM_SEC(teller03_info.total_break_time / teller03_info.total_breaks_taken));

		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// Longest break time for each of the three tellers
		monitor_data_size = sprintf((char*)monitor_buffer, "Longest break time for each of the three tellers\r\n\tTeller01: %ld seconds\r\n\tTeller02: %ld seconds\r\n\tTeller03: %ld seconds\r\n",
				MS_TO_SIM_SEC(teller01_info.max_break_time),
				MS_TO_SIM_SEC(teller02_info.max_break_time),
				MS_TO_SIM_SEC(teller03_info.max_break_time));

		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		// Shortest break time for each of the three tellers
		monitor_data_size = sprintf((char*)monitor_buffer, "Shortest break time for each of the three tellers\r\n\tTeller01: %ld seconds\r\n\tTeller02: %ld seconds\r\n\tTeller03: %ld seconds\r\n",
				MS_TO_SIM_SEC(teller01_info.min_break_time),
				MS_TO_SIM_SEC(teller02_info.min_break_time),
				MS_TO_SIM_SEC(teller03_info.min_break_time));

		HAL_UART_Transmit(&huart2, monitor_buffer, monitor_data_size, 100U);

		osThreadSuspend(simMonitorInfoHandle);
	}

	// Update monitor info every one minute of simulated time.
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
