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

#include "oled.h"
#include <stdio.h>
#include <stdlib.h>
#include "ICM20948.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTaskHand */
osThreadId_t DisplayTaskHandHandle;
const osThreadAttr_t DisplayTaskHand_attributes = {
  .name = "DisplayTaskHand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroscopeTaskHa */
osThreadId_t GyroscopeTaskHaHandle;
const osThreadAttr_t GyroscopeTaskHa_attributes = {
  .name = "GyroscopeTaskHa",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTaskHandle */
osThreadId_t MotorTaskHandleHandle;
const osThreadAttr_t MotorTaskHandle_attributes = {
  .name = "MotorTaskHandle",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTaskHand */
osThreadId_t EncoderTaskHandHandle;
const osThreadAttr_t EncoderTaskHand_attributes = {
  .name = "EncoderTaskHand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UltrasoundHandl */
osThreadId_t UltrasoundHandlHandle;
const osThreadAttr_t UltrasoundHandl_attributes = {
  .name = "UltrasoundHandl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRSensorHandler */
osThreadId_t IRSensorHandlerHandle;
const osThreadAttr_t IRSensorHandler_attributes = {
  .name = "IRSensorHandler",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void DisplayTask(void *argument);
void GyroTask(void *argument);
void MotorTask(void *argument);
void EncoderTask(void *argument);
void UltrasoundTask(void *argument);
void IRSensorTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART shit
uint8_t Rx_buff[20];
uint8_t Rx_data[20];
int sent = 0;
int Rx_index = 0;
int hasReceivedCommand = 0;
int moveForward = 0; int moveForwardAlways = 0; int scaleWallLeft = 0; int scaleWallRight = 0;
int forwardLeftHalf = 0; int forwardRightHalf = 0;
int forwardLeft90 = 0;
int forwardRight90 = 0;
int moveBackward = 0;
int backwardLeft90 = 0;
int backwardRight90 = 0;
int stopMovement = 0;
int done = 0;
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_UART_Receive_IT(&huart3, (uint8_t *) Rx_buff, 1);
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

  /* creation of DisplayTaskHand */
  DisplayTaskHandHandle = osThreadNew(DisplayTask, NULL, &DisplayTaskHand_attributes);

  /* creation of GyroscopeTaskHa */
  GyroscopeTaskHaHandle = osThreadNew(GyroTask, NULL, &GyroscopeTaskHa_attributes);

  /* creation of MotorTaskHandle */
  MotorTaskHandleHandle = osThreadNew(MotorTask, NULL, &MotorTaskHandle_attributes);

  /* creation of EncoderTaskHand */
  EncoderTaskHandHandle = osThreadNew(EncoderTask, NULL, &EncoderTaskHand_attributes);

  /* creation of UltrasoundHandl */
  UltrasoundHandlHandle = osThreadNew(UltrasoundTask, NULL, &UltrasoundHandl_attributes);

  /* creation of IRSensorHandler */
  IRSensorHandlerHandle = osThreadNew(IRSensorTask, NULL, &IRSensorHandler_attributes);

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
	while (1) {
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_PIN_Pin */
  GPIO_InitStruct.Pin = TRIG_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PIN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Gyro Stuff
ICM20948 imu;
float yawG;
float upperB, lowerB;
int mentallyStable = 0;
int turnMultiplier = 1;
int turnDistance = 0;

// PID stuff
uint16_t pwmVal; uint16_t pwmMax = 1300;
uint32_t countA; int16_t cntA; uint16_t positionA;
uint32_t countB; int16_t cntB; uint16_t positionB;
int distanceA; int distanceB; int targetDistance;
int straightLineDistanceTravelled = 0; int objectTwoLength = 0;
uint16_t pwmValA = 0;
int errA = 0;
int16_t errorA = 0;           // error between target and actual
int32_t error_areaA = 0;  // area under error - to calculate I for PI implementation
int32_t error_oldA = 0, error_changeA = 0;
float error_rateA = 0.0; // to calculate D for PID control
int32_t millisOldA = 0, millisNowA = 0, dtA = 0; // to calculate I and D for PID control
int16_t KpA = 0;
float KdA = 0;
float KiA = 0;

uint16_t pwmValB = 0;
int errB = 0;
int16_t errorB = 0;           // error between target and actual
int32_t error_areaB = 0;  // area under error - to calculate I for PI implementation
int32_t error_oldB = 0, error_changeB = 0;
float error_rateB = 0.0; // to calculate D for PID control
int32_t millisOldB = 0, millisNowB = 0, dtB = 0; // to calculate I and D for PID control
int16_t KpB = 0;
float KdB = 0;
float KiB = 0;

// Ultrasound
int Is_First_Captured = 0;
int32_t IC_Val1 = 0;
int32_t IC_Val2 = 0;
double Difference = 0;
double DistanceUS = 0;

//	IR Sensors
int stopDist = 0;
uint8_t stopDistStr[10];
uint8_t IR_Left;
uint8_t IR_Right;
uint8_t IR_Left_Str[20];
uint8_t IR_Right_Str[20];
int distFromWall = 0;


int Distance_travelled(uint16_t position) {
	int dist = (int) (position * 0.114);
	return dist;
}

void Motor_directionA(uint8_t forward) {
	if (forward) {// move forward
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	}
	else { // reverse
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	}
}

void Motor_directionB(uint8_t forward) {
	if (forward){// move forward
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	  }
	else { // reverse
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	}
}

uint16_t PID_ControlA() {
	if (abs(errorA > 2)) {
		errorA = targetDistance - distanceA;

		if (errorA > 0) Motor_directionA(1);
		else Motor_directionA(0);

		millisNowA = HAL_GetTick();
		dtA = millisNowA - millisOldA;
		millisOldA = millisNowA;
		error_areaA = error_areaA + errorA * dtA;
		error_changeA = errorA - error_oldA;
		error_oldA = errorA;
		error_rateA = error_changeA / dtA;

		pwmValA = (int)(errorA*KpA + error_areaA*KiA + error_rateA*KdA);
		if (pwmValA > pwmMax) pwmValA = pwmMax;
	}
	return pwmValA;
}

uint16_t PID_ControlABack() {
	if (abs(errorA > 2)) {
		errorA = targetDistance - distanceA;

		if (errorA > 0) Motor_directionA(0);
		else Motor_directionA(1);

		millisNowA = HAL_GetTick();
		dtA = millisNowA - millisOldA;
		millisOldA = millisNowA;
		error_areaA = error_areaA + errorA * dtA;
		error_changeA = errorA - error_oldA;
		error_oldA = errorA;
		error_rateA = error_changeA / dtA;

		pwmValA = (int)(errorA*KpA + error_areaA*KiA + error_rateA*KdA);
		if (pwmValA > pwmMax) pwmValA = pwmMax;
	}
	return pwmValA;
}

uint16_t PID_ControlB() {
	if (abs(errorB > 2)) {
		errorB = targetDistance - distanceB;

		if (errorB > 0) Motor_directionB(1);
		else Motor_directionB(0);

		millisNowB = HAL_GetTick();
		dtB = millisNowB - millisOldB;
		millisOldB = millisNowB;
		error_areaB = error_areaB + errorB * dtB;
		error_changeB = errorB - error_oldB;
		error_oldB = errorB;
		error_rateB = error_changeB / dtB;

		pwmValB = (int)(errorB*KpB + error_areaB*KiB + error_rateB*KdB);
		if (pwmValB > pwmMax) pwmValB = pwmMax;
	}
	return pwmValB;
}

uint16_t PID_ControlBBack() {
	if (abs(errorB > 2)) {
		errorB = targetDistance - distanceB;

		if (errorB > 0) Motor_directionB(0);
		else Motor_directionB(1);

		millisNowB = HAL_GetTick();
		dtB = millisNowB - millisOldB;
		millisOldB = millisNowB;
		error_areaB = error_areaB + errorB * dtB;
		error_changeB = errorB - error_oldB;
		error_oldB = errorB;
		error_rateB = error_changeB / dtB;

		pwmValB = (int)(errorB*KpB + error_areaB*KiB + error_rateB*KdB);
		if (pwmValB > pwmMax) pwmValB = pwmMax;
	}
	return pwmValB;
}

void getBoundaries()
{
	upperB = yawG + 1.0;
	lowerB = yawG - 1.0;
}

void delay_us(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER (&htim4) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured == 0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			DistanceUS = Difference * .034 / 2 - 6.0;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}

void Ultrasonic_Read(void){
  // Code for Ultrasonic Sensor
	HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	char unit[4];

	if (huart == &huart3)
	{
		if (Rx_buff[Rx_index] == '\n' || Rx_buff[Rx_index] == '\r')
		{
			Rx_buff[Rx_index] = '\0';
			Rx_index = 0;
			hasReceivedCommand = 1; done = 0; sent = 0;

			if (Rx_data[0] == 'F' && Rx_data[1] == 'S')
			{
				if (Rx_data[2] == 'A')
				{
					unit[0] = Rx_data[3]; unit[1] = Rx_data[4]; unit[2] = Rx_data[5]; unit[3] = '\0';
					targetDistance = atoi(unit);
					moveForwardAlways = 1;
					moveForward = 0;
				}

				else
				{
					unit[0] = Rx_data[2]; unit[1] = Rx_data[3]; unit[2] = Rx_data[4]; unit[3] = '\0';
					targetDistance = atoi(unit);
					getBoundaries();
					moveForward = 1;
					moveForwardAlways = 0;
				}

				moveBackward = forwardLeft90 = forwardLeftHalf = forwardRight90 = forwardRightHalf = backwardLeft90 = backwardRight90 = stopMovement = scaleWallLeft = scaleWallRight = 0;
			}

			else if (Rx_data[0] == 'S' && Rx_data[1] == 'L')
			{
				targetDistance = 200;
				scaleWallLeft = 1;
				distFromWall = IR_Left;
				moveForward = moveForwardAlways = moveBackward = forwardLeft90 = forwardLeftHalf = forwardRight90 = forwardRightHalf = backwardLeft90 = backwardRight90 = stopMovement = scaleWallRight = 0;

			}

			else if (Rx_data[0] == 'S' && Rx_data[1] == 'R')
			{
				targetDistance = 200;
				scaleWallRight = 1;
				distFromWall = IR_Right;
				moveForward = moveForwardAlways = moveBackward = forwardLeft90 = forwardLeftHalf = forwardRight90 = forwardRightHalf = backwardLeft90 = backwardRight90 = stopMovement = scaleWallLeft= 0;

			}

			else if (Rx_data[0] == 'F' && Rx_data[1] == 'L')
			{

				if (Rx_data[2] == 'H')
				{
					forwardLeftHalf = 1;
					forwardLeft90 = 0;
				}

				else
				{
					unit[0] = Rx_data[2]; unit[1] = '\0';
					turnMultiplier = atoi(unit);

					if (turnMultiplier > 4 || turnMultiplier < 1)
						turnMultiplier = 1;

					forwardLeft90 = 1;
					forwardLeftHalf = 0;
				}

				moveForward = moveForwardAlways = moveBackward = forwardRight90 = forwardRightHalf = backwardLeft90 = backwardRight90 = stopMovement = scaleWallLeft= 0;
			}

			else if (Rx_data[0] == 'F' && Rx_data[1] == 'R')
			{
				if (Rx_data[2] == 'H')
				{
					forwardRightHalf = 1;
					forwardRight90 = 0;
				}

				else
				{
					unit[0] = Rx_data[2]; unit[1] = '\0';
					turnMultiplier = atoi(unit);

					if (turnMultiplier > 4 || turnMultiplier < 1)
						turnMultiplier = 1;

					forwardRight90 = 1;
					forwardRightHalf = 0;
				}

				moveBackward = moveForward = forwardLeft90 = backwardLeft90 = backwardRight90 = stopMovement = 0;
			}

			else if (Rx_data[0] == 'B' && Rx_data[1] == 'S')
			{
				unit[0] = Rx_data[2]; unit[1] = Rx_data[3]; unit[2] = Rx_data[4]; unit[3] = '\0';
				targetDistance = atoi(unit);
				getBoundaries();
				moveBackward = 1;
				moveForward = moveForwardAlways = forwardLeftHalf = forwardLeft90 = forwardRightHalf = forwardRight90 = backwardLeft90 = backwardRight90 = stopMovement = 0;
			}

			else if (Rx_data[0] == 'B' && Rx_data[1] == 'L')
			{
				unit[0] = Rx_data[2]; unit[1] = '\0';
				turnMultiplier = atoi(unit);

				if (turnMultiplier > 4 || turnMultiplier < 1)
					turnMultiplier = 1;

				backwardLeft90 = 1;
				moveBackward = moveForward = forwardLeft90 = forwardRight90 = backwardRight90 = stopMovement = 0;
			}

			else if (Rx_data[0] == 'B' && Rx_data[1] == 'R')
			{
				unit[0] = Rx_data[2]; unit[1] = '\0';
				turnMultiplier = atoi(unit);

				if (turnMultiplier > 4 || turnMultiplier < 1)
					turnMultiplier = 1;

				backwardRight90 = 1;
				moveBackward = moveForward = forwardLeft90 = forwardRight90 = backwardLeft90 = stopMovement = 0;
			}

			else if (Rx_data[0] == 'S' && Rx_data[1] == 'T')
			{
				stopMovement = 1;
				backwardRight90 = moveBackward = moveForward = forwardLeft90 = forwardRight90 = backwardLeft90 = 0;
			}
		}

		else
		{
			Rx_index++;
			memset(Rx_data, '\0', sizeof(char) * 20);
			strncpy(Rx_data, Rx_buff, Rx_index);
			if (Rx_index > 20) Rx_index = 0;
		}

		HAL_UART_Receive_IT(&huart3, (uint8_t *)&Rx_buff[Rx_index], 1);
	}

//	HAL_UART_Transmit(&huart3, (uint8_t *)aRxBuffer, 10, 0xFFFF);
}

void resetEverything()
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	countA = 0; cntA = 0; positionA = 0; distanceA = 0;
	countB = 0; cntB = 0; positionB = 0; distanceB = 0;
	targetDistance = 0;
	pwmValA = 0;
	errA = 0;
	errorA = 0;           // error between target and actual
	error_areaA = 0;  // area under error - to calculate I for PI implementation
	error_oldA = 0;
	error_changeA = 0;
	error_rateA = 0.0; // to calculate D for PID control
	millisOldA = 0; millisNowA = 0; dtA = 0; // to calculate I and D for PID control


	pwmValB = 0;
	errB = 0;
	errorB = 0;           // error between target and actual
	error_areaB = 0;  // area under error - to calculate I for PI implementation
	error_oldB = 0;
	error_changeB = 0;
	error_rateB = 0.0; // to calculate D for PID control
	millisOldB = 0;
	millisNowB = 0;
	dtB = 0; // to calculate I and D for PID control

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
	uint8_t sbuf[55] = "\n\r";
	char newline = '\n';
	/* Infinite loop */
	for (;;) {
		if (done == 1 && !sent)
		{
			sprintf(&sbuf[2], "%5dT%5d\n", straightLineDistanceTravelled, objectTwoLength);
			HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf), HAL_MAX_DELAY);
//			HAL_UART_Transmit(&huart3, Rx_data, sizeof(Rx_data), HAL_MAX_DELAY);
//			HAL_UART_Transmit(&huart3, newline, sizeof(char), HAL_MAX_DELAY);
			sent = 1;
		}

		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DisplayTask */
/**
 * @brief Function implementing the DisplayTaskHand thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DisplayTask */
void DisplayTask(void *argument)
{
  /* USER CODE BEGIN DisplayTask */
	/* Infinite loop */
	uint32_t hello_display[20];
	for (;;) {
		sprintf(hello_display, "Dist: %5d", (int) DistanceUS);
		OLED_ShowString(10, 10, hello_display);
		if (!mentallyStable) OLED_ShowString(10, 20, "initializing");
		if (hasReceivedCommand == 1)
		{
			OLED_ShowString(10, 40, Rx_data);
//			hasReceivedCommand = 0;
		}
		OLED_Refresh_Gram();
		osDelay(500);
	}
  /* USER CODE END DisplayTask */
}

/* USER CODE BEGIN Header_GyroTask */
/**
 * @brief Function implementing the GyroscopeTaskHa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GyroTask */
void GyroTask(void *argument)
{
  /* USER CODE BEGIN GyroTask */
	mentallyStable = 0;
//	uint8_t myString[20];
	uint8_t data[20];
	float newTime, oldTime, dT;
//	IMU_Initialise(&imu, &hi2c1, &huart3);
	yawG = 0.0;
	oldTime = 0.0; newTime = 0.0; dT = 0.0;
	/* Infinite loop */

	for (;;) {

		if (!mentallyStable)
		{
			while (yawG < -1 || yawG > 1 || !mentallyStable)
			{
				yawG = 0.0;
				IMU_Initialise(&imu, &hi2c1, &huart3);
				IMU_GyroRead(&imu);

				newTime = HAL_GetTick();
				dT = newTime - oldTime;
				oldTime = newTime;

				yawG = yawG + imu.gyro[2] * dT * 0.001;

				if (yawG > -1.0 && yawG < 1.0)
					mentallyStable = 1;
			}
		}

		IMU_GyroRead(&imu);
		newTime = HAL_GetTick();
		dT = newTime - oldTime;
		oldTime = newTime;

		yawG = yawG + imu.gyro[2] * dT * 0.001;

		sprintf(data, "yawG: %5.2f\0", yawG); // @suppress("Float formatting support")
		OLED_ShowString(10, 20, data);


		if (moveForward)
		{
			if (yawG > upperB) 	htim1.Instance->CCR4 = 166; // off to the left so turn right
			else if (yawG < lowerB) htim1.Instance->CCR4 = 140; // off to the right so turn left
			else 	htim1.Instance->CCR4 = 153; // center
		}

		if (moveBackward)
		{
			if (yawG > upperB) 	htim1.Instance->CCR4 = 140; // off to the left so turn right
			else if (yawG < lowerB) htim1.Instance->CCR4 = 166; // off to the right so turn left
			else 	htim1.Instance->CCR4 = 153; // center
		}
	}
  /* USER CODE END GyroTask */
}

/* USER CODE BEGIN Header_MotorTask */
/**
* @brief Function implementing the MotorTaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask */
void MotorTask(void *argument)
{
  /* USER CODE BEGIN MotorTask */
//	int moveForward = 0;
//	int forwardLeft90 = 0;
//	int forwardRight90 = 0;
//	int moveBackward = 0;
//	int backwardLeft90 = 0;
//	int backwardRight90 = 0;
//	int done = 0;
//	moveForward = 1;
//	forwardLeft90 = 1;
//	forwardRight90 = 1;
//	moveBackward = 1;
//	backwardLeft90 = 1;
//	backwardRight90 = 1;
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 150; // center
	pwmVal = 2000;
	KpA = 40; KpB = 40;       // 10
	KiA = 0.002; KiB = 0.002;  // 0.001
	KdA = 0.001; KdB = 0.001;
  /* Infinite loop */
  for(;;)
  {
	  if (moveForward)
	  {
		  osDelay(500);
		  htim1.Instance->CCR4 = 150;

		  if (mentallyStable && !done)
		  {
			millisNowA = HAL_GetTick();

			errorA = targetDistance - distanceA;
			pwmValA = PID_ControlA();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);

			millisNowB = HAL_GetTick();

			errorB = targetDistance - distanceB;
			pwmValB = PID_ControlB();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);

			if (abs(errorA) < 2) {
				errA++;
				distanceA = Distance_travelled(positionA);
				errorA = targetDistance - distanceA;
			}

			if (abs(errorB) < 2) {
				errB++;
				distanceB = Distance_travelled(positionB);
				errorB = targetDistance - distanceB;
			}

			if (((errA > 5 || distanceA >= targetDistance) && (errB > 5 || distanceB >= targetDistance)) || DistanceUS < 30.0)
			{
				straightLineDistanceTravelled += (distanceA + distanceB) / 2;
				pwmValA = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				pwmValB = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
				moveForward = 0; resetEverything(); done = 1;
			}

		  }
	  }

	  else if (moveForwardAlways)
	  	  {
	  		  osDelay(500);
	  		  htim1.Instance->CCR4 = 150;

	  		  if (mentallyStable && !done)
	  		  {
	  			millisNowA = HAL_GetTick();

	  			errorA = targetDistance - distanceA;
	  			pwmValA = PID_ControlA();
	  			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);

	  			millisNowB = HAL_GetTick();

	  			errorB = targetDistance - distanceB;
	  			pwmValB = PID_ControlB();
	  			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);

	  			if (abs(errorA) < 2) {
	  				errA++;
	  				distanceA = Distance_travelled(positionA);
	  				errorA = targetDistance - distanceA;
	  			}

	  			if (abs(errorB) < 2) {
	  				errB++;
	  				distanceB = Distance_travelled(positionB);
	  				errorB = targetDistance - distanceB;
	  			}

	  			if (((errA > 5 || distanceA >= targetDistance) && (errB > 5 || distanceB >= targetDistance)) || DistanceUS < 15.0)
	  			{
	  				pwmValA = 0;
	  				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
	  				pwmValB = 0;
	  				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
	  				moveForward = 0; resetEverything(); done = 1;
	  			}

	  		  }
	  	  }

	  else if (scaleWallLeft)
	  {
		  osDelay(500);
		  htim1.Instance->CCR4 = 150;

		  if (mentallyStable && !done)
		  {
			millisNowA = HAL_GetTick();

			errorA = targetDistance - distanceA;
			pwmValA = PID_ControlA();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA/2);

			millisNowB = HAL_GetTick();

			errorB = targetDistance - distanceB;
			pwmValB = PID_ControlB();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB/2);

			if (abs(errorA) < 2) {
				errA++;
				distanceA = Distance_travelled(positionA);
				errorA = targetDistance - distanceA;
			}

			if (abs(errorB) < 2) {
				errB++;
				distanceB = Distance_travelled(positionB);
				errorB = targetDistance - distanceB;
			}

			if (((errA > 5 || distanceA >= targetDistance) && (errB > 5 || distanceB >= targetDistance)) || IR_Left > 30)
			{
				objectTwoLength = (distanceA + distanceB) / 4;
				pwmValA = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				pwmValB = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
				scaleWallLeft = 0; resetEverything(); done = 1;
			}

		  }

	  }

	  else if (scaleWallRight)
	  {
		  osDelay(500);
		  htim1.Instance->CCR4 = 150;

		  if (mentallyStable && !done)
		  {
			millisNowA = HAL_GetTick();

			errorA = targetDistance - distanceA;
			pwmValA = PID_ControlA();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA/2);

			millisNowB = HAL_GetTick();

			errorB = targetDistance - distanceB;
			pwmValB = PID_ControlB();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB/2);

			if (abs(errorA) < 2) {
				errA++;
				distanceA = Distance_travelled(positionA);
				errorA = targetDistance - distanceA;
			}

			if (abs(errorB) < 2) {
				errB++;
				distanceB = Distance_travelled(positionB);
				errorB = targetDistance - distanceB;
			}

			if (((errA > 5 || distanceA >= targetDistance) && (errB > 5 || distanceB >= targetDistance)) || IR_Right > 30)
			{
				objectTwoLength = (distanceA + distanceB) / 4;
				pwmValA = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				pwmValB = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
				scaleWallRight = 0; resetEverything(); done = 1;
			}

		  }

	  }

	  else if (forwardLeftHalf)
	  {
		  targetDistance = 21;
		  htim1.Instance->CCR4 = 95;
		  osDelay(500);
		  Motor_directionA(1);
		  Motor_directionB(1);
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000);
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);

		  distanceA = Distance_travelled(positionA);

//		  if (distanceA > targetDistance - 8)
//		  {
//			  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500);
//			  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);
//		  }

		  if (distanceA >= targetDistance)
		  {
			  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
			  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
			  htim1.Instance->CCR4 = 150;
			  forwardLeftHalf = 0; resetEverything(); done = 1;
			  osDelay(1000);

		  }



//		  	  targetDistance = 25;
//			  if (mentallyStable && !done)
//			  {
//
//				htim1.Instance->CCR4 = 95;
//				osDelay(500);
//
//				millisNowA = HAL_GetTick();
//
//				errorA = targetDistance - distanceA;
//				pwmValA = PID_ControlA();
//				Motor_directionB(1);
//				pwmValB = PID_ControlA();
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//
//
//				if (abs(errorA) < 2) {
//					errA++;
//					distanceA = Distance_travelled(positionA);
//					errorA = targetDistance - distanceA;
//				}
//
//				if ((errA > 5 || distanceA >= targetDistance))
//				{
//					pwmValA = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					pwmValB = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					htim1.Instance->CCR4 = 150;
//
//					forwardLeftHalf = 0; resetEverything(); done = 1;
//					osDelay(1000);
//				}
//		  }
	  }

	  else if (forwardLeft90)
	  {
		  targetDistance = 41;
		  if (mentallyStable && !done)
		  {
			htim1.Instance->CCR4 = 100;
			osDelay(500);

			millisNowA = HAL_GetTick();

			errorA = targetDistance - distanceA;
			pwmValA = PID_ControlA();
			Motor_directionB(1);
			pwmValB = PID_ControlA();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);


			if (abs(errorA) < 2) {
				errA++;
				distanceA = Distance_travelled(positionA);
				errorA = targetDistance - distanceA;
			}

			if ((errA > 5 || distanceA >= targetDistance))
			{
				pwmValA = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				pwmValB = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
				htim1.Instance->CCR4 = 150;

				forwardLeft90 = 0; resetEverything(); done = 1;
			}

//			if (distanceA < 42 * turnMultiplier)
//			{
//				Motor_directionA(1);
//				Motor_directionB(1);
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
//			}
//
//
//			else
//			{
//				htim1.Instance->CCR4 = 155;
//				pwmVal = 0;
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
//				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
//
//				forwardLeft90 = 0; resetEverything(); done = 1;
//			}

		  }
	  }

	  else if (forwardRightHalf)
	  {
		  if (mentallyStable && !done)
		  {
			  targetDistance = 17;

			  htim1.Instance->CCR4 = 243;
			  osDelay(500);

				millisNowB = HAL_GetTick();

				errorB = targetDistance - distanceB;
				pwmValB = PID_ControlB();
				Motor_directionA(1);
	//				pwmValA = PID_ControlB();
				pwmValA = PID_ControlB() * 37/22;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);

				if (abs(errorB) < 2) {
					errB++;
					distanceB = Distance_travelled(positionB);
					errorB = targetDistance - distanceB;
				}

				if ((errB > 5 || distanceB >= targetDistance))
				{
					pwmValA = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
					pwmValB = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
					htim1.Instance->CCR4 = 140;
					forwardRightHalf = 0; resetEverything(); done = 1;
					osDelay(1000);
				}


		  }
	  }

	  else if (forwardRight90)
	  {
		  targetDistance = 36;
		  if (mentallyStable && !done)
		  {
			  htim1.Instance->CCR4 = 245;
			  osDelay(500);

				millisNowB = HAL_GetTick();

				errorB = targetDistance - distanceB;
				pwmValB = PID_ControlB();
				Motor_directionA(1);
//				pwmValA = PID_ControlB();
				pwmValA = PID_ControlB() * 37/20;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);

				if (abs(errorB) < 2) {
					errB++;
					distanceB = Distance_travelled(positionB);
					errorB = targetDistance - distanceB;
				}

				if ((errB > 5 || distanceB >= targetDistance))
				{
					pwmValA = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
					pwmValB = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
					htim1.Instance->CCR4 = 140;
					forwardRight90 = 0; resetEverything(); done = 1;
				}
//				htim1.Instance->CCR4 = 240;
//				osDelay(500);
//
//				if (distanceB < 48 * turnMultiplier)
//				{
//					Motor_directionA(1);
//					Motor_directionB(1);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
//				}
//
//				else
//				{
//					htim1.Instance->CCR4 = 140;
//					pwmVal = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
//
//					forwardRight90 = 0; resetEverything(); done = 1;
//				}

		  }
	  }


	  else if (moveBackward)
	  {
		  htim1.Instance->CCR4 = 150;
		  osDelay(500);


		  if (mentallyStable && !done)
		  {
			millisNowA = HAL_GetTick();

			errorA = targetDistance - distanceA;
			pwmValA = PID_ControlABack();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);

			millisNowB = HAL_GetTick();

			errorB = targetDistance - distanceB;
			pwmValB = PID_ControlBBack();
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);

			if (abs(errorA) < 2) {
				errA++;
				distanceA = Distance_travelled(positionA);
				errorA = targetDistance - distanceA;
			}

			if (abs(errorB) < 2) {
				errB++;
				distanceB = Distance_travelled(positionB);
				errorB = targetDistance - distanceB;
			}

			if ((errA > 5 || distanceA >= targetDistance) && (errB > 5 || distanceB >= targetDistance))
			{
				pwmValA = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				pwmValB = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
				moveForward = 0; resetEverything(); done = 1;
			}

		  }

	  }

	  else if (backwardLeft90)
	  	  {
			  targetDistance = 41;
			  if (mentallyStable && !done)
			  {
				htim1.Instance->CCR4 = 100;
				osDelay(500);

				millisNowA = HAL_GetTick();

				errorA = targetDistance - distanceA;
				pwmValA = PID_ControlABack();
				Motor_directionB(0);
				pwmValB = PID_ControlABack();
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValA);


				if (abs(errorA) < 2) {
					errA++;
					distanceA = Distance_travelled(positionA);
					errorA = targetDistance - distanceA;
				}

				if ((errA > 5 || distanceA >= targetDistance))
				{
					pwmValA = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
					pwmValB = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
					htim1.Instance->CCR4 = 150;

					backwardLeft90 = 0; resetEverything(); done = 1;
				}
//	  			htim1.Instance->CCR4 = 100;
//	  			osDelay(500);
//
//				if (distanceA < (31 * turnMultiplier))
//				{
//					Motor_directionA(0);
//					Motor_directionB(0);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
//				}
//
//
//				else
//				{
//					htim1.Instance->CCR4 = 150;
//					pwmVal = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
//		  			backwardLeft90 = 0; resetEverything(); done = 1;
//				}
	  		  }
	  	  }


	  else if (backwardRight90)
	  	  {
		  targetDistance = 38;
		  if (mentallyStable && !done)
		  {
			  htim1.Instance->CCR4 = 245;
			  osDelay(500);

				millisNowB = HAL_GetTick();

				errorB = targetDistance - distanceB;
				pwmValB = PID_ControlBBack();
				Motor_directionA(0);
				pwmValA = PID_ControlBBack();
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValB);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);

				if (abs(errorB) < 2) {
					errB++;
					distanceB = Distance_travelled(positionB);
					errorB = targetDistance - distanceB;
				}

				if ((errB > 5 || distanceB >= targetDistance))
				{
					pwmValA = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
					pwmValB = 0;
					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
					htim1.Instance->CCR4 = 150;
					forwardRight90 = 0; resetEverything(); done = 1;
				}
//	  			htim1.Instance->CCR4 = 235;
//	  			osDelay(500);
//				if (distanceB < 46 * turnMultiplier)
//				{
//					Motor_directionA(0);
//					Motor_directionB(0);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
//				}
//
//
//				else
//				{
//					htim1.Instance->CCR4 = 150;
//					pwmVal = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
//
//					backwardRight90 = 0; resetEverything(); done = 1;
//				}
//
	  		  }
	  	  }

	  else if (stopMovement)
	  {
			htim1.Instance->CCR4 = 150;
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

		  resetEverything(); done = 1; stopMovement = 0;
	  }

	  osDelay(100);
  }
  /* USER CODE END MotorTask */
}

/* USER CODE BEGIN Header_EncoderTask */
/**
* @brief Function implementing the EncoderTaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderTask */
void EncoderTask(void *argument)
{
  /* USER CODE BEGIN EncoderTask */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	uint16_t encoder_buf[20];
	uint32_t tick;
	tick = HAL_GetTick();
	countA = 0; cntA = 0; positionA = 0; distanceA = 0;
	countB = 0; cntB = 0; positionB = 0; distanceB = 0;
  /* Infinite loop */
  for(;;)
  {
	if(HAL_GetTick() - tick > 10)
	{
		countA = __HAL_TIM_GET_COUNTER(&htim2);
		cntA = (int16_t) countA;
		positionA = abs(cntA) / 8;
		distanceA = Distance_travelled(positionA);

		countB = __HAL_TIM_GET_COUNTER(&htim3);
		cntB = (int16_t) countB;
		positionB = abs(cntB) / 8;
		distanceB = Distance_travelled(positionB);

//		sprintf(encoder_buf, "A: %4d B: %4d\0", distanceA, distanceB);
//		OLED_ShowString(10, 30, encoder_buf);
		tick = HAL_GetTick();

//		if (done) resetEverything();
	}

  }
  /* USER CODE END EncoderTask */
}

/* USER CODE BEGIN Header_UltrasoundTask */
/**
* @brief Function implementing the UltrasoundHandl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasoundTask */
void UltrasoundTask(void *argument)
{
  /* USER CODE BEGIN UltrasoundTask */
	uint8_t ultrasound_buf[20];
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
    Ultrasonic_Read();
//	sprintf(ultrasound_buf, "Dist: %5d\0", (int) DistanceUS);
//    sprintf(ultrasound_buf, "DIST HERE");
//	OLED_ShowString(10, 30, ultrasound_buf);
    osDelay(300);
  }
  /* USER CODE END UltrasoundTask */
}

/* USER CODE BEGIN Header_IRSensorTask */
/**
* @brief Function implementing the IRSensorHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IRSensorTask */
void IRSensorTask(void *argument)
{
  /* USER CODE BEGIN IRSensorTask */
	uint32_t adcVal1, adcVal2;
	float voltage1, voltage2;
  /* Infinite loop */
  for(;;)
  {
		//	Left IR Sensor
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		adcVal1 = HAL_ADC_GetValue(&hadc1); // Raw data
		voltage1 = (adcVal1 / pow(2,12)) * 3.3;
		IR_Left = 1 / (0.0140817 * pow(voltage1, 2) + 0.00685361 * voltage1 + 0.012403);
		sprintf(IR_Left_Str, "Left IR: %2d", IR_Left);
		OLED_ShowString(10, 40, IR_Left_Str);

		//	Right IR Sensor
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2,100);
		adcVal2 = HAL_ADC_GetValue(&hadc2); // Raw data
		voltage2 = (adcVal2 / pow(2,12)) * 3.3;
		IR_Right = 1 / (0.0140817 * pow(voltage2, 2) + 0.00685361 * voltage2 + 0.012403);
		sprintf(IR_Right_Str, "Right IR: %2d", IR_Right);
		OLED_ShowString(10, 50, IR_Right_Str);

		if (scaleWallLeft)
		{
			if (IR_Left > distFromWall) htim1.Instance->CCR4 = 140; // moving away from the wall so turn left
			else if (IR_Left < distFromWall) htim1.Instance->CCR4 = 165; // moving toward the wall so turn right
			else 	htim1.Instance->CCR4 = 150; // center
		}

		if (scaleWallRight)
		{
			if (IR_Right > distFromWall) htim1.Instance->CCR4 = 165; // moving away from the wall so turn right
			else if (IR_Right < distFromWall) htim1.Instance->CCR4 = 130; // moving toward the wall so turn left
			else 	htim1.Instance->CCR4 = 150; // center
		}


		osDelay(30);
  }
  /* USER CODE END IRSensorTask */
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
	while (1) {
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
