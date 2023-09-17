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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STOP,
	FOWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	FREE
} motorDirection;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ProcessDistance */
osThreadId_t ProcessDistanceHandle;
const osThreadAttr_t ProcessDistance_attributes = {
  .name = "ProcessDistance",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for updateTicks */
osThreadId_t updateTicksHandle;
const osThreadAttr_t updateTicks_attributes = {
  .name = "updateTicks",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for DirectionContro */
osThreadId_t DirectionControHandle;
const osThreadAttr_t DirectionContro_attributes = {
  .name = "DirectionContro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
int16_t encoder_velocity;
int32_t encoder_position;
uint32_t timerCounter;
int16_t timerC;
uint32_t ticks;
uint32_t timerCounterR;
int16_t timerCR;
uint32_t ticksR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void StartpDistance(void *argument);
void StartTicks(void *argument);
void StartDirection(void *argument);

/* USER CODE BEGIN PFP */
void drive(motorDirection x); // Robot directions (see enum)
void speedRight(int x); //PWM right
void speedLeft(int x); //PWM Left
void updateDistance(void); // Updates Distance Var
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Calls when timer completes
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if(htim == &htim2) {
		timerCounter = __HAL_TIM_GET_COUNTER (htim) ; //gets ticks for motor 1
		timerC = (int16_t)timerCounter; //Convert type to fix overflow
	} else if(htim == &htim4) {
		timerCounterR = __HAL_TIM_GET_COUNTER (htim) ; //gets ticks for motor 2
		timerCR = (int16_t)timerCounterR;
	}

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1); // Start timer for distance sensor sound wave time tracking
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low for distance sensor
	// Sets values to 0 in main (allows live variables for debugging)
	timerCounter = 0;
	timerCounterR = 0;
	ticks = 0;
	//set both motors to 100% duty cycle (full speed)
	speedLeft(1000);
	speedRight(1000);
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL); //Start timer to check ticks for motor 1
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL); //Start timer to check ticks for motor2
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); // Start PWM for motor 1 (PWM controls speed with duty cycle)
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2); // Start PWM for motor 2
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ProcessDistance */
  ProcessDistanceHandle = osThreadNew(StartpDistance, NULL, &ProcessDistance_attributes);

  /* creation of updateTicks */
  updateTicksHandle = osThreadNew(StartTicks, NULL, &updateTicks_attributes);

  /* creation of DirectionContro */
  DirectionControHandle = osThreadNew(StartDirection, NULL, &DirectionContro_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 720-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LED_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R_MOTOR1_Pin|R_MOTOR2_Pin|L_MOTOR2_Pin|L_MOTOR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = LED_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R_MOTOR1_Pin R_MOTOR2_Pin L_MOTOR2_Pin L_MOTOR1_Pin */
  GPIO_InitStruct.Pin = R_MOTOR1_Pin|R_MOTOR2_Pin|L_MOTOR2_Pin|L_MOTOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void drive(motorDirection x) {

	HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR1_Pin, 0);
	HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR2_Pin, 0);
	HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR1_Pin, 0);
	HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR2_Pin, 0);

	if(x==STOP) {
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR1_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR2_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR1_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR2_Pin, 1);
	} else if (x==FREE) {
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR1_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR2_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR1_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR2_Pin, 0);
	}else if (x==FOWARD) {
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR1_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR2_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR1_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR2_Pin, 1);
	} else if (x==BACKWARD) {
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR1_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR2_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR1_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR2_Pin, 0);
	} else if (x==LEFT) {
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR1_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR2_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR1_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR2_Pin, 1);
	} else if (x==RIGHT) {
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR1_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, R_MOTOR2_Pin, 0);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR1_Pin, 1);
		HAL_GPIO_WritePin(R_MOTOR1_GPIO_Port, L_MOTOR2_Pin, 0);
	}
	osDelay(20);
}

void speedRight(int x) {

	if(1000<x) {
		x = 1000;
	}
	TIM5->CCR1 = x;
}
void speedLeft(int x) {
	if(1000<x) {
		x = 1000;
	}
	TIM5->CCR2 = x;
}

void updateDistance(void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	// wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
	Value1 = __HAL_TIM_GET_COUNTER (&htim1);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
	Value2 = __HAL_TIM_GET_COUNTER (&htim1);

	Distance = (Value2-Value1)* 0.034/2;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		osDelay(500);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartpDistance */
/**
 * @brief Function implementing the ProcessDistance thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartpDistance */
void StartpDistance(void *argument)
{
  /* USER CODE BEGIN StartpDistance */
	/* Infinite loop */
	for(;;)
	{
		updateDistance(); //get distance from ultra sonic sensor (updates Distance variable)


		osDelay(20);
	}
  /* USER CODE END StartpDistance */
}

/* USER CODE BEGIN Header_StartTicks */
/**
 * @brief Function implementing the updateTicks thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTicks */
void StartTicks(void *argument)
{
  /* USER CODE BEGIN StartTicks */
	/* Infinite loop */
	for(;;)
	{
		ticks = timerC; //gets ticks for motor 1
		ticksR = timerCR;//gets ticks for motor 2

		timerCounter = 0; // Resets counters for encoders
		timerC = 0;
		timerCounterR = 0;
		timerCR = 0;
		osDelay(10);
	}
  /* USER CODE END StartTicks */
}

/* USER CODE BEGIN Header_StartDirection */
/**
 * @brief Function implementing the DirectionContro thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDirection */
void StartDirection(void *argument)
{
  /* USER CODE BEGIN StartDirection */
	/* Infinite loop */
	for(;;)
	{
		if(Distance < 15&&Distance > 2) { // checks if object is near
			drive(BACKWARD);
			osDelay(900); //delay for 900ms
			drive(LEFT);
			osDelay(2295); // 2295ms delay

		} else {

			drive(FOWARD);
		}
		osDelay(20);
	}
  /* USER CODE END StartDirection */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
