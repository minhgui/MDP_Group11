/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "time.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SERVOCENTER 146
#define SERVORIGHT 250
#define SERVOLEFT 84

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
/* Definitions for rightEncoderTas */
osThreadId_t rightEncoderTasHandle;
const osThreadAttr_t rightEncoderTas_attributes = {
  .name = "rightEncoderTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for leftEncoderTask */
osThreadId_t leftEncoderTaskHandle;
const osThreadAttr_t leftEncoderTask_attributes = {
  .name = "leftEncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UltrasoundTask */
osThreadId_t UltrasoundTaskHandle;
const osThreadAttr_t UltrasoundTask_attributes = {
  .name = "UltrasoundTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRTask */
osThreadId_t IRTaskHandle;
const osThreadAttr_t IRTask_attributes = {
  .name = "IRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void rightEncoder(void *argument);
void leftEncoder(void *argument);
void StartCommsTask(void *argument);
void StartOLEDTask(void *argument);
void StartMotorTask(void *argument);
void StartGyroTask(void *argument);
void StartUltrasoundTask(void *argument);
void StartIRTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//UART
int receivedInstruction = 0;
uint8_t aRxBuffer[5];
int flagDone = 0;
int amplitude=0;

// movement
uint16_t pwmVal_servo = SERVOCENTER;
uint16_t pwmVal_Right = 0;
uint16_t pwmVal_Left = 0;
int times_acceptable = 0;
int e_brake = 0;
int selfModifyPinsSemaphore = 0;
int uFlag = 0;
int irTaskuFlag = 0;
int gyroResetFlag = 0;

// encoder
int32_t rightEncoderVal = 0, leftEncoderVal = 0;
int32_t rightTarget = 0, leftTarget = 0;
double target_angle = 0;
int32_t O2distanceTravelled = 0;

// gyro
double total_angle = 0;
uint8_t gyroBuffer[20];
uint8_t ICMAddress = 0x68;

//ultrasonic
int u_CapturedTrig1;
int uintPart;
double echo_dist;
double echo_pulse;
int32_t trig1;
int32_t trig2;
int obstacle_detected = 0;

//IR Sensor
uint16_t RawLeftIRVal = 0;
uint16_t RawRightIRVal = 0;
uint16_t LeftIRfiltered = 0;
uint16_t RightIRfiltered = 0;
uint16_t IRDistance_Left = 0;
uint16_t IRDistance_Right = 0;
uint16_t *IRforTask2;



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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /*------------INIT LIBRARIES ----------------*/
 OLED_Init();
 void delay_us(uint16_t time_us);
 //motor_init(&htim8, &htim2, &htim2); 						//initialize motor PWM and encoders. ############################################

 // HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer,5);			//HAL...(HUART3,aRxBuffer, (how many character to input before it receives)

 /*------------END INIT LIBRARIES ----------------*/
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

  /* creation of rightEncoderTas */
  rightEncoderTasHandle = osThreadNew(rightEncoder, NULL, &rightEncoderTas_attributes);

  /* creation of leftEncoderTask */
  leftEncoderTaskHandle = osThreadNew(leftEncoder, NULL, &leftEncoderTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommsTask, NULL, &CommTask_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(StartOLEDTask, NULL, &OLED_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(StartGyroTask, NULL, &GyroTask_attributes);

  /* creation of UltrasoundTask */
  UltrasoundTaskHandle = osThreadNew(StartUltrasoundTask, NULL, &UltrasoundTask_attributes);

  /* creation of IRTask */
  IRTaskHandle = osThreadNew(StartIRTask, NULL, &IRTask_attributes);

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
  while(1)
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
  sConfig.Channel = ADC_CHANNEL_12;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
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

  /*Configure GPIO pin : TRIGGER_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIGGER_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning*/
	 UNUSED(huart);
	 receivedInstruction =1;
}


// movement
void moveCarStraight(double distance) {

	distance = distance * 75;
	  pwmVal_servo = SERVOCENTER;
	  osDelay(300);
	  e_brake = 0;
	  times_acceptable = 0;
	  rightEncoderVal = 75000;
	  leftEncoderVal = 75000;
	  rightTarget = 75000;
	  leftTarget = 75000;
	  rightTarget += distance;
	  leftTarget += distance;

	  while (finishCheck());
}

void moveCarStop() {
	e_brake = 1;
	pwmVal_servo = SERVOCENTER;
	osDelay(200);
}

void moveCarRight(double angle) {
	pwmVal_servo = SERVORIGHT;
	osDelay(450);
	e_brake = 0;
	times_acceptable = 0;
	target_angle -= angle;
	while (finishCheck());
}

void moveCarLeft(double angle) {
	pwmVal_servo = SERVOLEFT;
	osDelay(450);
	e_brake = 0;
	times_acceptable = 0;
	target_angle += angle;
	while (finishCheck());
}



// error correction
int PID_Control(int error, int right) {
	if (right) { //rightMotor
		if (error > 0) { //go forward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel B (RIGHT)- FORWARD
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		} else { //go backward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel B (RIGHT)- BACKWARDS
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		}
	} else { //leftMotor
		if (error > 0) { //go forward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A(LEFT) - FORWARD
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		} else { //go backward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A(LEFT) - BACKWARDS
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		}
	}

	error = abs(error);
	if (error > 2000) {
		return 3000;
	} else if (error > 500)     {
		return 2000;
	 } else if (error > 200)  {
		return 1400;
	  } else if (error > 100)    {
		return 1000;
	   } else if (error > 2)  {
			times_acceptable++;
			return 500;
			} else if (error >= 1) {
		times_acceptable++;
		return 0;
	} else {
		times_acceptable++;
		return 0;
	}
}

int PID_Angle(double errord, int right) {
	int error = (int) (errord * 10);
	if (right) { //rightMotor = WHEEL B
		if (error > 0) { //go forward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // wheel B- forward
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		} else { //go backward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET); // wheel B - reverse
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		}
	} else { //leftMotor = WHEEL A
		if (error < 0) { //go forward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // wheel A - forward
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		} else { //go backward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET); // wheel A - reverse
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		}
	}

	error = abs(error);
	if (error > 300) {
		return 5000;		//3000
	} else if (error > 200) {
		return 3000;		//2000
	} else if (error > 150) {
		return 3000;		//1600
	} else if (error > 100) {
		return 1400;		//1400
	} else if (error > 10) {
		return 1400;		//1000
	} else if (error >= 2) {
		times_acceptable++;
		return 1000;		//600
	} else {
		times_acceptable++;
		return 0;
	}
}

int finishCheck() {
	if (times_acceptable > 5) {		//check to confirm angle/position is reached, then return 0. increase as needed if need more precise measurements
		e_brake = 1;
		pwmVal_Left = pwmVal_Right = 0;
		leftTarget = leftEncoderVal;
		rightTarget = rightEncoderVal;
		times_acceptable = 0;
		osDelay(200);
		return 0;
	}
	return 1;
}


 //gyro
void readByte(uint8_t addr, uint8_t *data) {
	gyroBuffer[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddress << 1, gyroBuffer, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddress << 1, data, 2, 20);
}

void writeByte(uint8_t addr, uint8_t data) {
	gyroBuffer[0] = addr;
	gyroBuffer[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddress << 1, gyroBuffer, 2, 20);
}

void gyroInit() {
	writeByte(0x06, 0x00);
	osDelay(10);
	writeByte(0x03, 0x80);
	osDelay(10);
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x06, 0x01);
	osDelay(10);
	writeByte(0x7F, 0x20);
	osDelay(10);
	writeByte(0x01, 0x2F);
	osDelay(10);
	writeByte(0x0, 0x00);
	osDelay(10);
	writeByte(0x7F, 0x00);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);
}

/************************TASK 2 FUNCTIONS******************/

// ultrasonic
void delay_us(uint16_t time_us) {
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < time_us);
}


void Ultrasonic_Read(void) //Call when u want to get reading from US
{
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){


	if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_1){
		if(u_CapturedTrig1 == 0){	// Ultrasonic Trigger first sent
			trig1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			u_CapturedTrig1 =1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (u_CapturedTrig1 == 1){	//Ultrasonic Trigger 2nd sent
				trig2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				__HAL_TIM_SET_COUNTER(htim,0);

				if (trig2 > trig1){
					echo_pulse = trig2-trig1;		//width of pulse
				} else {
					echo_pulse = (65535-trig1)+trig2;	//if overflow, width of pulse
				}
				echo_dist = echo_pulse * 0.0343/2; //echo_dist in cm

				u_CapturedTrig1 =0;

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}


// ir sensor
void IR_Left_Read() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	RawLeftIRVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	//LPF, Filter coefficient = 0.15
	LeftIRfiltered = (0.15 * RawLeftIRVal) + ((1 - 0.15) * LeftIRfiltered);

	if(LeftIRfiltered <450) 		//cap the IR filter range to be from 0-70cm+-, else it will go haywire
		IRDistance_Left = 101;
	else {
		//Equation from calibration and testing
		IRDistance_Left = 29076.34/(LeftIRfiltered-170.63);
	}
}

void IR_Right_Read() {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 100);
	RawRightIRVal = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

	//LPF, Filter coefficient = 0.15
	RightIRfiltered = (0.15 * RawRightIRVal) + ((1 - 0.15) * RightIRfiltered);

	if(RightIRfiltered <450) 		//cap the IR filter range to be from 0-70cm+-, else it will go haywire
		IRDistance_Right = 101;
	else {
		//Eqn from calibration and testing
		IRDistance_Right = 30181.23/ (RightIRfiltered-133.62);
	//IRDistance_Right = 29076.34/(RightIRfiltered-170.63);
	}
}


void Control_Drift() {		//control drift to go straight
	  //control servo angle to go straight
			  if(total_angle < -2) {	//veering to the right
				  pwmVal_servo = ((-19 * 5) / 5 + SERVOCENTER);
			  }
			  else if(total_angle > 2) {
				  pwmVal_servo = ((19 * 5) / 5 + SERVOCENTER);
			  }
			  else {
				  pwmVal_servo = ((19 * total_angle) / 5 + SERVOCENTER);
			  }
}

void Task2IRmoveStraight() {
	  //**************move parallel along the obstacle till nothing is seen on the side of robot
	selfModifyPinsSemaphore =1 ;
	e_brake = 0;

	//gyro RESET after turning, without whole STM reset
	total_angle =0;
	target_angle = 0;
	pwmVal_servo = SERVOCENTER;
	osDelay(300);

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);		//left wheel fwd direction
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);	//right wheel fwd direction
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

	  while(*IRforTask2 < 50) {

		  pwmVal_Left = pwmVal_Right = 3500;
		  Control_Drift();
	  }
	  e_brake = 1;
	  selfModifyPinsSemaphore = 0;
	  //moveCarStraight(-5);

}

void Obstacle1Ultra() {

	//START task 2. reset gyro
	selfModifyPinsSemaphore =1 ;
	total_angle =0;
	target_angle = 0;
	e_brake = 0;
	pwmVal_servo = SERVOCENTER;

	osDelay(50);

	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);		//left wheel fwd direction
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);	//right wheel fwd direction
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


	  while (echo_dist >17) { 	//actual echo_dist stops at 15cm
		  if(echo_dist >50){
				pwmVal_Left = 6000;
				pwmVal_Right = 6000;
		  }
		  else if(echo_dist >40)
		  {
				pwmVal_Left = 1700;
				pwmVal_Right = 1700;
		  }
		  else if(echo_dist >17) {
			  pwmVal_Left = 1500;
			  pwmVal_Right = 1500;
		  }
		  Control_Drift();
	  }

	  e_brake = 1;
	  selfModifyPinsSemaphore = 0;
}


void Obstacle1_Move() {		//
	//if right arrow
	if(aRxBuffer[4] == 'R') {
		  moveCarRight(58);
		  moveCarLeft(116);
		  moveCarStraight(3);
		  moveCarRight(58);

	}
	  //if left arrow
	if(aRxBuffer[4] == 'L') {
	  moveCarLeft(58);
	  moveCarRight(116);
	  moveCarStraight(3);
	  moveCarLeft(58);
	}

	osDelay(100);
	if(echo_dist < 10)
		moveCarStraight(-25);
	else if(echo_dist < 20)
		moveCarStraight(-15);
	else if(echo_dist < 25)
		  moveCarStraight(-10);

	pwmVal_servo = SERVOCENTER;
	osDelay(300);

}

void Obstacle2Ultra() {
	selfModifyPinsSemaphore =1 ;
	e_brake = 0;
	leftEncoderVal = 0; 	//O2distanceTravelled
	osDelay(25);

	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);		//left wheel fwd direction
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);	//right wheel fwd direction
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


	  while (echo_dist >25) { 	//actual echo_dist stops at 23cm
		  if(echo_dist >50){
				pwmVal_Left = 6000;
				pwmVal_Right = 6000;
		  }
		  else if(echo_dist >40)
		  {
				pwmVal_Left = 1700;
				pwmVal_Right = 1700;
		  }
		  else if(echo_dist >25) {
			  pwmVal_Left = 1500;
			  pwmVal_Right = 1500;
		  }

		  Control_Drift();
	  }
	  e_brake = 1;
	  selfModifyPinsSemaphore = 0;
	  osDelay(100);	//delay to get more accurate reading of O2distanceTravelled
	  O2distanceTravelled = leftEncoderVal;
}

void Obstacle2_Move () {
//if right arrow

	if(aRxBuffer[4] == 'R') {		//use IRDistance_Left
		IRforTask2 = &IRDistance_Left;
		  moveCarRight(90);
	}
	  //if left arrow
	else if(aRxBuffer[4] == 'L') {	//use IRDistance_Right
		IRforTask2 = &IRDistance_Right;
		moveCarLeft(90);
	}
	osDelay(500);
	if(*IRforTask2 >50) { 		//correction to move back if exceeded obstacle length after turning
		moveCarStraight(-25);
	}

	Task2IRmoveStraight();
/*
 * 						<-- 	end goal of the next if statement
 * 		obstacle2			^
 * ********************		|
 * 						-->
 *
 */
	if(aRxBuffer[4] == 'L')		    moveCarRight(180);

	else if(aRxBuffer[4] == 'R') 	moveCarLeft(180);

	moveCarStraight(7);
	Task2IRmoveStraight();

	if(aRxBuffer[4] == 'L') 	    moveCarRight(90);
	else if(aRxBuffer[4] == 'R') 	moveCarLeft(90);


	/****************Start task back to carpark**************************/
	pwmVal_servo = SERVOCENTER;
	osDelay(300);
	total_angle =0;
	target_angle = 0;
	selfModifyPinsSemaphore = 1;
	e_brake = 0;

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);		//left wheel fwd direction
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);	//right wheel fwd direction
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

	  pwmVal_Left = 6000;
	  pwmVal_Right = 6000;
	  leftEncoderVal = 0;

	  //formula to calculate distance travelled between O1 and O2. OFFSET in whole numbers only!!!
	  //OFFSET (in cm) is to account for extra distance to travel to get past O1 when returning back
	  //distance travelled(cm) = (leftEncoderVal / 1500) * 20.4 ; (1500 counts = 1 rev, 20.4cm = circumference of wheel)

	  // formula = ((OFFSET/20.4) * 1500)

	  while(leftEncoderVal < (O2distanceTravelled + 5515) ) {	//OFFSET = 70cm = 5515 (please update these to keep track of our offset)
		  Control_Drift();
	  }
	  e_brake =1;
	  selfModifyPinsSemaphore = 0;

	if(aRxBuffer[4] == 'L')			moveCarRight(90);
	else if(aRxBuffer[4] == 'R') 	moveCarLeft(90);

	pwmVal_servo = SERVOCENTER;
	//osDelay(250);
	moveCarStraight(-15);
	selfModifyPinsSemaphore = 1;
	e_brake = 0;

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);		//left wheel fwd direction
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);	//right wheel fwd direction
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

	total_angle =0;
	target_angle = 0;


	//to detect O1 before turning in to face cp
	while(*IRforTask2 >45) {
		  pwmVal_Left = pwmVal_Right = 3500;
		  Control_Drift();
	}
	e_brake = 1;
	selfModifyPinsSemaphore = 0;

	moveCarStraight(-15);

	if(aRxBuffer[4] == 'L')			moveCarLeft(90);
	else if(aRxBuffer[4] == 'R') 	moveCarRight(90);

	//robot is now facing cp. CHIOOOOOONG TO ENDLINE!!!

	pwmVal_servo = SERVOCENTER;
	selfModifyPinsSemaphore =1 ;
		total_angle =0;
		target_angle = 0;
		e_brake = 0;

		osDelay(300);

		  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);		//left wheel fwd direction
		  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);	//right wheel fwd direction
		  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


		  while (echo_dist >20) { 	//actual echo_dist stops at ~18cm
			  if(echo_dist >50){
					pwmVal_Left = 6000;
					pwmVal_Right = 6000;
			  }
			  else if(echo_dist >35)
			  {
					pwmVal_Left = 2000;
					pwmVal_Right = 2000;
			  }
			  else if(echo_dist >20) {
				  pwmVal_Left = 1500;
				  pwmVal_Right = 1500;
			  }
			  Control_Drift();
		  }

		  e_brake = 1;
		  selfModifyPinsSemaphore = 0;
}

/************** END OF TASK 2 FUNCTIONS *****************/


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
	 HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
     osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_rightEncoder */
/**
* @brief Function implementing the rightEncoderTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rightEncoder */
void rightEncoder(void *argument)
{
  /* USER CODE BEGIN rightEncoder */
  /* Infinite loop */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
		int cnt1;
		int dirR = 1;
		int diff;
		uint32_t tick = HAL_GetTick();
		/* Infinite loop */
		for (;;) {
			if (HAL_GetTick() - tick > 10L) {
				cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
				if (cnt1 > 32000) {
					dirR = 1;
					diff = (65536 - cnt1);
				} else {
					dirR = -1;
					diff = cnt1;
				}
				if (dirR == 1) {
					rightEncoderVal -= diff;
				} else {
					rightEncoderVal += diff;
				}

				__HAL_TIM_SET_COUNTER(&htim3, 0);

				tick = HAL_GetTick();
			}
			osDelay(50);
		}
  /* USER CODE END rightEncoder */
}

/* USER CODE BEGIN Header_leftEncoder */
/**
* @brief Function implementing the leftEncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_leftEncoder */
void leftEncoder(void *argument)
{
  /* USER CODE BEGIN leftEncoder */
  /* Infinite loop */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	int cnt2;
	int dirL = 1;
	int diff;

	uint32_t tick = HAL_GetTick();

	/* Infinite loop */
	for (;;) {
		if (HAL_GetTick() - tick > 10L) {
			cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
			if (cnt2 > 32000) {
				dirL = 1;
				diff = (65536 - cnt2);
			} else {
				dirL = -1;
				diff = cnt2;
			}
			if (dirL == 1) {
				leftEncoderVal += diff;
			} else {
				leftEncoderVal -= diff;
			}

			__HAL_TIM_SET_COUNTER(&htim2, 0);

			tick = HAL_GetTick();
		}

//		sprintf(hello, "SpeedL: %5d\0", diff);
//		  OLED_ShowString(10,30,hello);
//		  sprintf(hello, "DirL: %5d\0", dirL);
//		  OLED_ShowString(10,40,hello);
		  osDelay(50);
	}
  /* USER CODE END leftEncoder */
}

/* USER CODE BEGIN Header_StartCommsTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommsTask */
void StartCommsTask(void *argument)
{
  /* USER CODE BEGIN StartCommsTask */
  /* Infinite loop */

	uint8_t ack[4] = "A\0";        // ACK
	  uint8_t invalid[8] = "I\0";    // "INVALID
	  uint8_t obstacle[2] = "O\0";   // Obstacle detected
	  int skipToStep =0;

	  strcpy((char *)aRxBuffer, "START");

	  /* Infinite loop */
	  for(;;)
	  {
	    // Make sure UART is ready to receive
	    HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 5);

	    if(receivedInstruction == 1)
	    {
	      amplitude = 0;

	      // Check valid command format
	      if (
	    		  (strcmp(aRxBuffer, "RESET") == 0)
				  || (	(aRxBuffer[0] == 'F' || aRxBuffer[0] == 'B')        // move Forward, Backwards
				  && (aRxBuffer[1] == 'S' || aRxBuffer[1] == 'R' || aRxBuffer[1] == 'L')	)   // move straight, right, or left
				  && ((aRxBuffer[2] - '0') >= 0 && (aRxBuffer[2] - '0') <= 9)
				  && ((aRxBuffer[3] - '0') >= 0 && (aRxBuffer[3] - '0') <= 9)
				  && ((aRxBuffer[4] - '0') >= 0 && (aRxBuffer[4] - '0') <= 9)
				   //(aRxBuffer[0] == 'O' && aRxBuffer[1] == 'B' && aRxBuffer[2] == 'S') 	//OBStacle, task 2
				  || (strcmp(aRxBuffer, "OBS1U") == 0) || (strcmp(aRxBuffer, "OBS2U") == 0) 	//ultrasound for obstacles
				  || (strcmp(aRxBuffer, "OBS1L") == 0) || (strcmp(aRxBuffer, "OBS1R") == 0)
				  || (strcmp(aRxBuffer, "OBS2L") == 0) || (strcmp(aRxBuffer, "OBS2R") == 0)
	      	  )
	      {
//	        // Check if we're trying to move forward and there's an obstacle
//	        if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'S' && obstacle_detected) {
//	          // Don't execute the forward command if there's an obstacle
//	          strcpy((char *)aRxBuffer, "OBST!");
//	          HAL_UART_Transmit(&huart3, (uint8_t*) obstacle, 2, 0xFFFF);  // Send obstacle alert
//	          receivedInstruction = 0;
//	        }
//	        else {
	          // Process the command as usual
	          HAL_UART_AbortReceive_IT(&huart3);  // pause receiving from UART

	          if(aRxBuffer[2]!='E'){	//if is normal commands, assign the amplitude value
				  amplitude = ((int) (aRxBuffer[2]) - 48) * 100
							+ ((int) (aRxBuffer[3]) - 48) * 10
							+ ((int) (aRxBuffer[4]) - 48);


				  if (aRxBuffer[0] == 'B') {          // check Direction first
					amplitude *= -1;
				  }
	          }
	          osDelay(10);


	          if(strncmp(aRxBuffer, "OBS", 3) ==0 )
	          {
	        	  //run the switch statements
	        	  switch (aRxBuffer[3]) {
	        	  	  case '1':

							  switch(aRxBuffer[4]) {
								  case 'U':
									  Obstacle1Ultra();
									  strcpy((char *)aRxBuffer, "IMAGE");
									  break;
								  case 'L':
								  case 'R':
									  Obstacle1_Move();
									  break;
							  }
							  //for case 1
							  flagDone = 1;
							  skipToStep = 1;
							  break;

						//break for case 1
	        	  		  break;


	        	  	  case '2':

							  switch(aRxBuffer[4]) {
								  case 'U':
									  Obstacle2Ultra();
									  strcpy((char *)aRxBuffer, "OBST2");
									  break;
								  case 'L':
								  case 'R':
									  Obstacle2_Move();
									  break;
							  }
							  //break for case 2
							  flagDone = 1;
							  skipToStep = 1;
							  break;
	        	  }
	        	  strcpy((char *)aRxBuffer, "DONE!");
	          }


	          if(!skipToStep) {


	          switch (aRxBuffer[1]) {
	          case 'S':
	            moveCarStraight(amplitude);
	            strcpy((char *)aRxBuffer, "DONE!");    // Display to ACK cmd on OLED
	            flagDone = 1;
	            osDelay(10);
	            break;
	          case 'R':
	            moveCarRight(amplitude);
	            strcpy((char *)aRxBuffer, "DONE!");    // Display to ACK cmd on OLED
	            flagDone = 1;
	            osDelay(10);
	            break;
	          case 'L':
	            moveCarLeft(amplitude);
	            strcpy((char *)aRxBuffer, "DONE!");    // Display to ACK cmd on OLED
	            flagDone = 1;
	            osDelay(10);
	            break;
	          case 'E':                   // reset whole system to recalibrate gyro
	            strcpy((char *)aRxBuffer, "RESET");
	            HAL_UART_Transmit(&huart3, (uint8_t*) "R\0", 2, 0xFFFF);  // TRANSMITTING "RESET" TO RPI/PUTTY
	            osDelay(500);
	            NVIC_SystemReset();
	            osDelay(10);
	            break;
	          }
	          }	//for skipToStep
	          skipToStep =0;

	 // for else bracket }
	      }	//end if valid choice


	      else      // INVALID CHOICE
	      {
	        strcpy((char *)aRxBuffer, "INVAL");
	        osDelay(10);
	        HAL_UART_Transmit(&huart3, (uint8_t*) &invalid, 8, 0xFFFF);  // TRANSMITTING "INVAL" TO RPI/PUTTY
	        receivedInstruction = 0;
	      }
	    }

	    if (flagDone == 1) {
	      receivedInstruction = 0;
	      osDelay(10);
	      HAL_UART_Transmit(&huart3, (uint8_t*) &ack, 4, 0xFFFF);   // TRANSMITTING "ACK" TO RPI/PUTTY
	      flagDone = 0;
	    }

	    osDelay(10);
	  }
  /* USER CODE END StartCommsTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void *argument)
{
  /* USER CODE BEGIN StartOLEDTask */
  /* Infinite loop */
// aaa	uint8_t usVal[20] = { 0 };
	uint8_t gyroVal[20] = { 0 };
	uint8_t command[20] = { 0 };
	uint8_t echo[20] = { 0 };
	int32_t IR_Left[20] = { 0 };
	int32_t IR_Right[20] = { 0 };

	for (;;) {
		int decimals = abs((int) ((total_angle - (int) (total_angle)) * 1000));
		sprintf(gyroVal, "Gyro: %d.%d \0", (int) total_angle, decimals);
		OLED_ShowString(0, 0, gyroVal);

		sprintf(command, "C: %c%c%c%c%c \0", aRxBuffer[0], aRxBuffer[1],
				aRxBuffer[2], aRxBuffer[3], aRxBuffer[4]);
		OLED_ShowString(0, 10, command);

		//print ultrasonic readings
		uintPart = (int)echo_dist;
		sprintf(echo, "E = %dcm \0", uintPart);		//ultrasonic readings
		OLED_ShowString(0, 20, echo);

		//IR Readings
		sprintf(IR_Left, "Left:%d D:%d \0", LeftIRfiltered, IRDistance_Left);
		OLED_ShowString(0, 30,IR_Left );
		sprintf(IR_Right, "Right:%d D:%d \0", RightIRfiltered, IRDistance_Right);
		OLED_ShowString(0, 40,IR_Right );

		OLED_Refresh_Gram();
		osDelay(100);
	}
  /* USER CODE END StartOLEDTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
	pwmVal_Right = 0;
	  pwmVal_Left = 0;
	  int straightCorrection = 0;
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	  htim1.Instance->CCR4 = SERVOCENTER; 	//Centre

	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // RIGHT WHEEL FORWARD
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // LEFT WHEEL FORWARD
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	  osDelay(1000);

	  /* Infinite loop */
	  for (;;) {
	    htim1.Instance->CCR4 = pwmVal_servo;

	    if(selfModifyPinsSemaphore ==1){
	    	//skips the MotorTask
	    }

	    else {	//task 1 functions

	    		double error_angle = target_angle - total_angle;	//100 - gyro (with drift) angle

				  // Normal operation - no obstacle or not moving forward
				  if (pwmVal_servo < 127) { //TURN LEFT
					pwmVal_Right = PID_Angle(error_angle, 1) * 1.072;  //RIGHT = MASTER
					pwmVal_Left = pwmVal_Right * (0.59); //LEFT = SLAVE

					if (error_angle > 0) {
					  // LEFT WHEEL GO FORWARD
					  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
					} else {
					  //// LEFT WHEEL GO BACKWARD
					  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
					}
				  }

				  else if (pwmVal_servo > 189) { //TURN RIGHT
					pwmVal_Left = PID_Angle(error_angle, 0);
					pwmVal_Right = pwmVal_Left * (0.59); //RIGHT = SLAVE

					if (error_angle < 0) {
					  //RIGHT (WHEEL B) FORWARD
					  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
					} else {
					  //RIGHT (WHEEL B) BACKWARD
					  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
					}
				  }

				  else {		//mainly for task 1, used in task 2 for accurate turns
					pwmVal_Right = PID_Control(leftTarget - leftEncoderVal, 0) * 1.072;
					if (labs(leftTarget - leftEncoderVal)> labs(rightTarget - rightEncoderVal)) {
					  straightCorrection++;
					} else {
					  straightCorrection--;
					}
					if (labs(leftTarget - leftEncoderVal) < 100) {
					  straightCorrection = 0;
					}
					pwmVal_Left = PID_Control(rightTarget - rightEncoderVal, 1)
							+ straightCorrection;

					if ((leftTarget - leftEncoderVal) < 0) {
					  if (error_angle > 2) { // left +. right -. angle
						pwmVal_servo = ((19 * 5) / 5 + SERVOCENTER);
					  } else if (error_angle < -2) {
						pwmVal_servo = ((-19 * 5) / 5 + SERVOCENTER);
					  } else {
						pwmVal_servo = ((19 * error_angle) / 5 + SERVOCENTER);
					  }
					} else {
					  if (error_angle > 2) { // left +. right -.
						pwmVal_servo = ((-19 * 5) / 5 + SERVOCENTER);
					  } else if (error_angle < -2) {
						pwmVal_servo = ((19 * 5) / 5 + SERVOCENTER);
					  } else {
						pwmVal_servo = ((-19 * error_angle) / 5 + SERVOCENTER);
					  }
					}
					//line correction equation is pwmVal = (19*error)/5 + SERVOCENTER
				  }





	    }//task 1 functions end


	    if (e_brake) {
	      pwmVal_Left = pwmVal_Right = 0;
	      leftTarget = leftEncoderVal;
	      rightTarget = rightEncoderVal;
	    }

	    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal_Left);
	    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal_Right);
	    osDelay(10);

	    if (times_acceptable > 1000) {
	      times_acceptable = 1001;
	    }
	  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
* @brief Function implementing the GyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void *argument)
{
  /* USER CODE BEGIN StartGyroTask */

			gyroInit();
			uint8_t val[2] = { 0, 0 };

			int16_t angularSpeed = 0;

			uint32_t tick = 0;
			double offset = 0;
			double garbage = 0;
			int i = 0;
			total_angle = 0;

			osDelay(50);
			tick = HAL_GetTick();
			while (i < 1000) {					//sample 1000 for calibration
				osDelay(5);
				readByte(0x37, val);
				angularSpeed =  val[1] | (val[0] << 8);

				garbage = garbage + (double) ((double) angularSpeed)
						* ((HAL_GetTick() - tick) / 16400.0);
				offset = offset + angularSpeed;		//CALIBRATION OFFSET
				tick = HAL_GetTick();
				i++;
			}
			offset = offset / i;

			tick = HAL_GetTick();
			/* Infinite loop */
			for (;;) {
				osDelay(5);
				readByte(0x37, val);
				angularSpeed = val[1] | (val[0] << 8);
				total_angle = total_angle + (double) ((double) angularSpeed - offset)
						* ((HAL_GetTick() - tick) / 16400.0);	//calculate angle change by time diff
				i = i- angularSpeed;
				i++;
				tick = HAL_GetTick();
			}

  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartUltrasoundTask */
/**
* @brief Function implementing the UltrasoundTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasoundTask */
void StartUltrasoundTask(void *argument)
{
  /* USER CODE BEGIN StartUltrasoundTask */
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	/* Infinite loop */
  for(;;)
  {
	  Ultrasonic_Read();
	      osDelay(10); // Adjust delay as needed
  }
  /* USER CODE END StartUltrasoundTask */
}

/* USER CODE BEGIN Header_StartIRTask */
/**
* @brief Function implementing the IRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIRTask */
void StartIRTask(void *argument)
{
  /* USER CODE BEGIN StartIRTask */
  /* Infinite loop */
  for(;;)
  {
	  IR_Left_Read();
	  IR_Right_Read();
    osDelay(25);
  }
  /* USER CODE END StartIRTask */
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
