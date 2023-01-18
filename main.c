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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RGB_Off 	0
#define RGB_Red 	1
#define RGB_Green 	2
#define RGB_Blue	3

#define temp_raw_cold 850 // 18ºC
#define temp_raw_hot  950 // 26ºC

#define limite_LDR 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MASK(pos) ( 1 << pos )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RGB */
osThreadId_t RGBHandle;
const osThreadAttr_t RGB_attributes = {
  .name = "RGB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Fire_Task */
osThreadId_t Fire_TaskHandle;
const osThreadAttr_t Fire_Task_attributes = {
  .name = "Fire_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ADC_Read */
osThreadId_t ADC_ReadHandle;
const osThreadAttr_t ADC_Read_attributes = {
  .name = "ADC_Read",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for COM */
osThreadId_t COMHandle;
const osThreadAttr_t COM_attributes = {
  .name = "COM",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Actuadores */
osThreadId_t ActuadoresHandle;
const osThreadAttr_t Actuadores_attributes = {
  .name = "Actuadores",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC_Read_Cola */
osMessageQueueId_t ADC_Read_ColaHandle;
const osMessageQueueAttr_t ADC_Read_Cola_attributes = {
  .name = "ADC_Read_Cola"
};
/* Definitions for Actuadores_Cola */
osMessageQueueId_t Actuadores_ColaHandle;
const osMessageQueueAttr_t Actuadores_Cola_attributes = {
  .name = "Actuadores_Cola"
};
/* Definitions for com_lock */
osMutexId_t com_lockHandle;
const osMutexAttr_t com_lock_attributes = {
  .name = "com_lock"
};
/* Definitions for Fire_Semaphore */
osSemaphoreId_t Fire_SemaphoreHandle;
const osSemaphoreAttr_t Fire_Semaphore_attributes = {
  .name = "Fire_Semaphore"
};
/* Definitions for Sound_Flag */
osEventFlagsId_t Sound_FlagHandle;
const osEventFlagsAttr_t Sound_Flag_attributes = {
  .name = "Sound_Flag"
};
/* Definitions for Fire_Flag */
osEventFlagsId_t Fire_FlagHandle;
const osEventFlagsAttr_t Fire_Flag_attributes = {
  .name = "Fire_Flag"
};
/* Definitions for Sys_Flag */
osEventFlagsId_t Sys_FlagHandle;
const osEventFlagsAttr_t Sys_Flag_attributes = {
  .name = "Sys_Flag"
};
/* USER CODE BEGIN PV */
struct ADC_Values{
	uint32_t temp;
	uint32_t ldr;
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void Start_RGB_Task(void *argument);
void Start_Fire_Task(void *argument);
void Start_ADC_Read(void *argument);
void Start_COM(void *argument);
void StartActuadores_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_rgb (uint8_t red, uint8_t green, uint8_t blue)
{
 htim3.Instance->CCR1 = red;
 htim3.Instance->CCR2 = green;
 htim3.Instance->CCR3 = blue;
}

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin ){
	if(GPIO_Pin == Pulsador_Azul_Pin){
		osEventFlagsSet(Sys_FlagHandle, 0x00000001U);
	}
	else if(GPIO_Pin & MASK(9)){
		osEventFlagsSet(Fire_FlagHandle, 0x00000001U);
	}
	else if((GPIO_Pin & MASK(8)) && !(HAL_GPIO_ReadPin(GPIOB, Fire_Detect_Pin))){
		osEventFlagsSet(Sound_FlagHandle, 0x00000001U);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of com_lock */
  com_lockHandle = osMutexNew(&com_lock_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Fire_Semaphore */
  Fire_SemaphoreHandle = osSemaphoreNew(1, 1, &Fire_Semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ADC_Read_Cola */
  ADC_Read_ColaHandle = osMessageQueueNew (4, sizeof(struct ADC_Values), &ADC_Read_Cola_attributes);

  /* creation of Actuadores_Cola */
  Actuadores_ColaHandle = osMessageQueueNew (4, sizeof(struct ADC_Values), &Actuadores_Cola_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RGB */
  RGBHandle = osThreadNew(Start_RGB_Task, NULL, &RGB_attributes);

  /* creation of Fire_Task */
  Fire_TaskHandle = osThreadNew(Start_Fire_Task, NULL, &Fire_Task_attributes);

  /* creation of ADC_Read */
  ADC_ReadHandle = osThreadNew(Start_ADC_Read, NULL, &ADC_Read_attributes);

  /* creation of COM */
  COMHandle = osThreadNew(Start_COM, NULL, &COM_attributes);

  /* creation of Actuadores */
  ActuadoresHandle = osThreadNew(StartActuadores_task, NULL, &Actuadores_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of Sound_Flag */
  Sound_FlagHandle = osEventFlagsNew(&Sound_Flag_attributes);

  /* creation of Fire_Flag */
  Fire_FlagHandle = osEventFlagsNew(&Fire_Flag_attributes);

  /* creation of Sys_Flag */
  Sys_FlagHandle = osEventFlagsNew(&Sys_Flag_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 692-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Ambiente_Pin|LED_Frio_Pin|LED_Calor_Pin|LED_LDR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|LED_Fire_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Pulsador_Azul_Pin */
  GPIO_InitStruct.Pin = Pulsador_Azul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pulsador_Azul_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Ambiente_Pin LED_Frio_Pin LED_Calor_Pin LED_LDR_Pin */
  GPIO_InitStruct.Pin = LED_Ambiente_Pin|LED_Frio_Pin|LED_Calor_Pin|LED_LDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 LED_Fire_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|LED_Fire_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Sound_Detect_Pin Fire_Detect_Pin */
  GPIO_InitStruct.Pin = Sound_Detect_Pin|Fire_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
	char msg_power_off[8] = "SAPAGADO";
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(Sys_FlagHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);
	  osSemaphoreAcquire(Fire_SemaphoreHandle, osWaitForever);

	  osMutexAcquire(com_lockHandle, osWaitForever);
	  HAL_UART_Transmit(&huart2, msg_power_off, sizeof(msg_power_off), 1000);
	  osMutexRelease(com_lockHandle);

	  // Apagar todas las salidas
	  set_rgb(0,0,0);
	  HAL_GPIO_WritePin(GPIOC, LED_Ambiente_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, LED_Calor_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, LED_Frio_Pin, 0);
	  HAL_GPIO_WritePin(GPIOC, LED_LDR_Pin, 0);

	  osSemaphoreRelease(Fire_SemaphoreHandle);
	  osEventFlagsClear(Sys_FlagHandle, 0x00000001U);
	  exit(0);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_RGB_Task */
/**
* @brief Function implementing the RGB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_RGB_Task */
void Start_RGB_Task(void *argument)
{
  /* USER CODE BEGIN Start_RGB_Task */
  int RGB = RGB_Off;
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_3);
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(Sound_FlagHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);
	switch(RGB){
		case RGB_Off:
			set_rgb(0,0,0);
			RGB = RGB_Red;
			break;
		case RGB_Red:
			set_rgb(255,0,0);
			RGB = RGB_Green;
			break;
		case RGB_Green:
			set_rgb(0,255,0);
			RGB = RGB_Blue;
			break;
		case RGB_Blue:
			set_rgb(0,0,255);
			RGB = RGB_Off;
			break;
	}
	osEventFlagsClear(Sound_FlagHandle, 0x00000001U);
	osDelay(10);
  }
  /* USER CODE END Start_RGB_Task */
}

/* USER CODE BEGIN Header_Start_Fire_Task */
/**
* @brief Function implementing the Fire_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Fire_Task */
void Start_Fire_Task(void *argument)
{
  /* USER CODE BEGIN Start_Fire_Task */
  unsigned char msg_fire[8] = "INCENDIO";
  unsigned char msg_apagado[8] = "IAPAGADO";
  HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_4);
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(Fire_FlagHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);
	osSemaphoreAcquire(Fire_SemaphoreHandle, osWaitForever);

	// Apagar todas las salidas no necesarias
	set_rgb(0,0,0);
	HAL_GPIO_WritePin(GPIOC, LED_Ambiente_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, LED_Calor_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, LED_Frio_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, LED_LDR_Pin, 0);

	htim3.Instance->CCR4 = 250;

	osMutexAcquire(com_lockHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, msg_fire, sizeof(msg_fire), 1000);
	osMutexRelease(com_lockHandle);

	while(HAL_GPIO_ReadPin(GPIOB, Fire_Detect_Pin)){
		HAL_GPIO_TogglePin(GPIOB, LED_Fire_Pin);
		osDelay(500);
	}

	htim3.Instance->CCR4 = 0;
	osDelay(1000);
	HAL_GPIO_WritePin(GPIOB, LED_Fire_Pin, 0);

	osMutexAcquire(com_lockHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, msg_apagado, sizeof(msg_apagado), 1000);
	osMutexRelease(com_lockHandle);

	osEventFlagsClear(Fire_FlagHandle, 0x00000001U);
	osSemaphoreRelease(Fire_SemaphoreHandle);
	osDelay(10);
  }
  /* USER CODE END Start_Fire_Task */
}

/* USER CODE BEGIN Header_Start_ADC_Read */
/**
* @brief Function implementing the ADC_Read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ADC_Read */
void Start_ADC_Read(void *argument)
{
  /* USER CODE BEGIN Start_ADC_Read */
  struct ADC_Values datos;

  HAL_StatusTypeDef status ;
  /* Infinite loop */
  for(;;)
  {
	//osEventFlagsWait(Fire_FlagHandle, 0x00000000U, osFlagWaitAny, osWaitForever);
	if (osSemaphoreGetCount(Fire_SemaphoreHandle)){
		HAL_ADC_Start (&hadc1) ; /* INICIO */
		status = HAL_ADC_PollForConversion (&hadc1, 1);
		if( status == HAL_OK ){
			datos.temp = HAL_ADC_GetValue (&hadc1);
		}
		status = HAL_ADC_PollForConversion (&hadc1, 1);
		if( status == HAL_OK ){
			datos.ldr = HAL_ADC_GetValue (&hadc1);
		}
		osMessageQueuePut(ADC_Read_ColaHandle, &datos, osPriorityNormal, 0U);
		osMessageQueuePut(Actuadores_ColaHandle, &datos, osPriorityNormal, 0U);
		HAL_ADC_Stop (&hadc1) ;
	}
	osDelay(1000);
  }
  /* USER CODE END Start_ADC_Read */
}

/* USER CODE BEGIN Header_Start_COM */
/**
* @brief Function implementing the COM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_COM */
void Start_COM(void *argument)
{
  /* USER CODE BEGIN Start_COM */
  /* Infinite loop */
  for(;;)
  {
	  struct ADC_Values message;
	  unsigned char message_str[8] = {'0','0','0','0','0','0','0','0'};

	  osMessageQueueGet(ADC_Read_ColaHandle, &message, NULL, osWaitForever);
	  sprintf(message_str, "%04ld%04ld", message.temp, message.ldr);
	  osMutexAcquire(com_lockHandle, osWaitForever);
	  HAL_UART_Transmit(&huart2, message_str, sizeof(message_str), 1000);
	  osMutexRelease(com_lockHandle);

  }
  /* USER CODE END Start_COM */
}

/* USER CODE BEGIN Header_StartActuadores_task */
/**
* @brief Function implementing the Actuadores thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartActuadores_task */
void StartActuadores_task(void *argument)
{
  /* USER CODE BEGIN StartActuadores_task */
  struct ADC_Values values;
  uint32_t temp_raw, ldr_raw;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(Actuadores_ColaHandle, &values, NULL, osWaitForever);
	  temp_raw = values.temp;
	  ldr_raw = values.ldr;

	  // Estados Temperatura
	  if(temp_raw < temp_raw_cold) // Frio
	  {
		  HAL_GPIO_WritePin(GPIOC, LED_Ambiente_Pin, 0);
		  HAL_GPIO_WritePin(GPIOC, LED_Calor_Pin, 0);
		  HAL_GPIO_WritePin(GPIOC, LED_Frio_Pin, 1);
		  osDelay(500);
	  }
	  else if (temp_raw > temp_raw_hot) // Calor
	  {
		  HAL_GPIO_WritePin(GPIOC, LED_Ambiente_Pin, 0);
		  HAL_GPIO_WritePin(GPIOC, LED_Calor_Pin, 1);
		  HAL_GPIO_WritePin(GPIOC, LED_Frio_Pin, 0);
	  }
	  else // Templado
	  {
		  HAL_GPIO_WritePin(GPIOC, LED_Ambiente_Pin, 1);
		  HAL_GPIO_WritePin(GPIOC, LED_Calor_Pin, 0);
		  HAL_GPIO_WritePin(GPIOC, LED_Frio_Pin, 0);
	  }

	  if(ldr_raw < limite_LDR){
		  HAL_GPIO_WritePin(GPIOC, LED_LDR_Pin, 0);
	  }
	  else{
		  HAL_GPIO_WritePin(GPIOC, LED_LDR_Pin, 1);
	  }
  }
  /* USER CODE END StartActuadores_task */
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
