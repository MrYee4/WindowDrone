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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "Accelerometer_Interface.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int Remote_Throttle_Command, Remote_Roll_Command, Remote_Pitch_Command, Remote_Yaw_Command;
int FL_Motor, FR_Motor, BL_Motor, BR_Motor;

float Positional_Roll, Positional_Pitch, Positional_Yaw;
float AccelErrorX, AccelErrorY, AccelErrorZ;

uint8_t Accel_buffer_in[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Get_Pulses(void);
void Send_Pulse(int length);
void MPU_6050_Init(void);
void Get_Pos(void);
void Motor_Adjust_Roll(void);
void Motor_Adjust_Pitch(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char msg[300];
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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Start ADC
  HAL_ADC_Start(&hadc1);

  // Start Timer PWM Generation
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // Initialize and zero out Gyro/Accel chips
  MPU_6050_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Initialize PWM values for motor control
  Remote_Roll_Command = 600;
  Remote_Pitch_Command = 600;
  Remote_Yaw_Command = 600;
  int STM_Throttle_PWM = 80;
  int STM_Roll_PWM = 0;
  int STM_Pitch_PWM = 0;
  int STM_Yaw_PWM = 0;

  while (1)
  {
	/**************************************************/
	/**************** reset Control *******************/
	/**************************************************/

	FL_Motor = 0;
	FR_Motor = 0;
	BL_Motor = 0;
	BR_Motor = 0;

	/**************************************************/
	/**************** Remote_Throttle_Command Control ****************/
	/**************************************************/

	if (Remote_Throttle_Command < 400)
	{
		Remote_Throttle_Command = 400;
	}
	STM_Throttle_PWM = Min_Throttle + ((Remote_Throttle_Command - 400) / 5.7);
	FL_Motor = STM_Throttle_PWM;
	FR_Motor = STM_Throttle_PWM;
	BL_Motor = STM_Throttle_PWM;
	BR_Motor = STM_Throttle_PWM;

	/**************************************************/
	/****************** Remote_Roll_Command Control ******************/
	/**************************************************/

	STM_Roll_PWM = ((Remote_Roll_Command - 400) / 5.7);
	if(STM_Roll_PWM > 40)
	{
		int Temp_Motor_Roll_PWM = STM_Roll_PWM - 40;

		if(STM_Throttle_PWM + Temp_Motor_Roll_PWM < Max_Throttle)
		{
			FL_Motor += (Temp_Motor_Roll_PWM/2);
			BL_Motor += (Temp_Motor_Roll_PWM/2);
		}
		else
		{
			FL_Motor = Max_Throttle;
			BL_Motor = Max_Throttle;
		}
	}
	else if(STM_Roll_PWM < 30)
	{
		int Temp_Motor_Roll_PWM = STM_Roll_PWM - 30;
		Temp_Motor_Roll_PWM *= -1;

		if(STM_Throttle_PWM + Temp_Motor_Roll_PWM < Max_Throttle)
		{
			FR_Motor += (Temp_Motor_Roll_PWM/2);
			BR_Motor += (Temp_Motor_Roll_PWM/2);
		}
		else
		{
			FR_Motor = Max_Throttle;
			BR_Motor = Max_Throttle;
		}


	}

	/**************************************************/
	/****************** Remote_Pitch_Command Control *****************/
	/**************************************************/

	STM_Pitch_PWM = ((Remote_Pitch_Command - 400) / 5.7);
	if(STM_Pitch_PWM > 40)
	{
		int Temp_Motor_Pitch_PWM = STM_Pitch_PWM - 40;

		if(STM_Throttle_PWM + Temp_Motor_Pitch_PWM < Max_Throttle)
		{
			BL_Motor += (Temp_Motor_Pitch_PWM/2);
			BR_Motor += (Temp_Motor_Pitch_PWM/2);
		}
		else
		{
			BL_Motor = Max_Throttle;
			BR_Motor = Max_Throttle;
		}
	}
	else if(STM_Pitch_PWM < 30)
	{
		int Temp_Motor_Pitch_PWM = STM_Pitch_PWM - 30;
		Temp_Motor_Pitch_PWM *= -1;

		if(STM_Throttle_PWM + Temp_Motor_Pitch_PWM < Max_Throttle)
		{
			FL_Motor += (Temp_Motor_Pitch_PWM/2);
			FR_Motor += (Temp_Motor_Pitch_PWM/2);
		}
		else
		{
			FL_Motor = Max_Throttle;
			FR_Motor = Max_Throttle;
		}

	}

	/**************************************************/
	/******************* Remote_Yaw_Command Control ******************/
	/**************************************************/
	STM_Yaw_PWM = ((Remote_Yaw_Command - 400) / 5.7);
	if(STM_Yaw_PWM > 40)
	{
		STM_Yaw_PWM -= 40;
		if(STM_Throttle_PWM + STM_Yaw_PWM < Max_Throttle)
		{
			BL_Motor += STM_Yaw_PWM;
			FR_Motor += STM_Yaw_PWM;
		}
		else
		{
			BL_Motor = Max_Throttle;
			FR_Motor = Max_Throttle;
		}
	}
	else if(STM_Yaw_PWM < 30)
	{
		STM_Yaw_PWM -= 30;
		STM_Yaw_PWM *= -1;
		if(STM_Throttle_PWM + STM_Yaw_PWM < Max_Throttle)
		{
			FL_Motor += STM_Yaw_PWM;
			BR_Motor += STM_Yaw_PWM;
		}
		else
		{
			FL_Motor = Max_Throttle;
			BR_Motor = Max_Throttle;
		}
	}

	/**************************************************/
	/********* Adjust Motor Values per Accel **********/
	/**************************************************/

	if (30 < STM_Roll_PWM && STM_Roll_PWM < 40 && STM_Throttle_PWM > 90)
	{
		Motor_Adjust_Roll();
	}
	if (30 < STM_Pitch_PWM && STM_Pitch_PWM < 40 && STM_Throttle_PWM > 90)
	{
		Motor_Adjust_Pitch();
	}

	/**************************************************/
	/**************** Set Motor Values ****************/
	/**************************************************/

	TIM2->CCR3 = BR_Motor;
	TIM3->CCR4 = FL_Motor;
	TIM4->CCR1 = FR_Motor;
	TIM15->CCR2 = BL_Motor;


	// Convert to string and prints
	Get_Pos();
	//sprintf(msg, "FL: %i FR: %i BL: %i BR: %i Roll: %f.4 Pitch: %f.4\r\n", FL_Motor, FR_Motor, BL_Motor, BR_Motor, Positional_Roll, Positional_Pitch);
	//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_DELAY);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1600-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 60;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1600-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim6.Init.Prescaler = 1000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1600-1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1000-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1600-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 145;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 100-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3200-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pitch_PWM_IN_Pin */
  GPIO_InitStruct.Pin = Pitch_PWM_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pitch_PWM_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Roll_PWM_IN_Pin */
  GPIO_InitStruct.Pin = Roll_PWM_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Roll_PWM_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Yaw_PWM_IN_Pin */
  GPIO_InitStruct.Pin = Yaw_PWM_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Yaw_PWM_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Throttle_PWM_IN_Pin */
  GPIO_InitStruct.Pin = Throttle_PWM_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Throttle_PWM_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0) {
		Get_Pulses();
	} else {
	   __NOP();
	}
}

void MPU_6050_Init(void)
{
	HAL_StatusTypeDef ret = HAL_ERROR;
	uint8_t data = PM1_No_Reset;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, POWER_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, I2C_DELAY);
	if (ret != HAL_OK)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nError Initializing MPU_6050\r\n", strlen("\r\nError Initializing MPU_6050\r\n"), I2C_DELAY);
	}
	else
	{
		data = Gyro_500_Sens;
		ret = HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, GYRO_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, I2C_DELAY);
		if (ret != HAL_OK)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nError Initializing Gyro\r\n", strlen("\r\nError Initializing Gyro\r\n"), I2C_DELAY);
		}
		else
		{
			data = Accel_4g_Sens;
			ret = HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, ACCEL_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, I2C_DELAY);
			if (ret != HAL_OK)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nError Initializing Accel\r\n", strlen("\r\nError Initializing Accel\r\n"), I2C_DELAY);
			}
			else
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nInitialized MPU_6050!\r\n", strlen("\r\nInitialized MPU_6050!\r\n"), I2C_DELAY);
			}
		}
	}

	int x = 0;
	AccelErrorX = 0;
	AccelErrorY = 0;
	AccelErrorZ = 0;
	int16_t dAccX;
	int16_t dAccY;
	int16_t dAccZ;
	while (x < Number_of_Calibrations)
	{
		HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, ACCEL_ADDR, I2C_MEMADD_SIZE_8BIT, Accel_buffer_in, 6, I2C_DELAY);
		dAccX = ((int16_t)Accel_buffer_in[0] << 8) | (Accel_buffer_in[1]);
		dAccY = ((int16_t)Accel_buffer_in[2] << 8) | (Accel_buffer_in[3]);
		dAccZ = ((int16_t)Accel_buffer_in[4] << 8) | (Accel_buffer_in[5]);

		float AccX = dAccX / Accel_4g_LSB_Divide;
		float AccY = dAccY / Accel_4g_LSB_Divide;
		float AccZ = (dAccZ / Accel_4g_LSB_Divide) - 1;
		AccelErrorX += AccX;
		AccelErrorY += AccY;
		AccelErrorZ += AccZ;
		x++;
	}

	AccelErrorX = AccelErrorX / Number_of_Calibrations;
	AccelErrorY = AccelErrorY / Number_of_Calibrations;
	AccelErrorZ = AccelErrorZ / Number_of_Calibrations;
	return;
}

void Get_Pos(void)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, ACCEL_ADDR, I2C_MEMADD_SIZE_8BIT, Accel_buffer_in, 6, I2C_DELAY);
	int16_t dAccX = ((int16_t)Accel_buffer_in[0] << 8) | (Accel_buffer_in[1]);
	int16_t dAccY = ((int16_t)Accel_buffer_in[2] << 8) | (Accel_buffer_in[3]);
	int16_t dAccZ = ((int16_t)Accel_buffer_in[4] << 8) | (Accel_buffer_in[5]);

	float AccX = (dAccX / Accel_4g_LSB_Divide) - AccelErrorX;
	float AccY = (dAccY / Accel_4g_LSB_Divide) - AccelErrorY;
	float AccZ = (dAccZ / Accel_4g_LSB_Divide) - AccelErrorZ;

	Positional_Pitch = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / M_PI);
	Positional_Roll = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / M_PI);

	return;
}

void Motor_Adjust_Roll(void)
{
	/**************************************************/
	/****************** Remote_Roll_Command Control ******************/
	/**************************************************/

	if(Positional_Roll > 2)
	{
		FL_Motor -= (Positional_Roll / Adujst_Value);
		BL_Motor -= (Positional_Roll / Adujst_Value);

		if (FL_Motor < Min_Throttle)
		{
			FL_Motor = Min_Throttle;
		}
		if (BL_Motor < Min_Throttle)
		{
			BL_Motor = Min_Throttle;
		}

	}
	else if (Positional_Roll < -2)
	{
		FR_Motor += (Positional_Roll / Adujst_Value);
		BR_Motor += (Positional_Roll / Adujst_Value);

		if (FR_Motor < Min_Throttle)
		{
			FR_Motor = Min_Throttle;
		}
		if (BR_Motor < Min_Throttle)
		{
			BR_Motor = Min_Throttle;
		}

	}

}

void Motor_Adjust_Pitch(void)
{

	/**************************************************/
	/****************** Remote_Pitch_Command Control *****************/
	/**************************************************/

	// Check if the pitch stick is centered
	if(Positional_Pitch > 2)
	{
		FL_Motor -= (Positional_Pitch / Adujst_Value);
		FR_Motor -= (Positional_Pitch / Adujst_Value);

		// Check if the Front Left Motor is out of the Min throttle ranges
		if (FL_Motor < Min_Throttle)
		{
			FL_Motor = Min_Throttle;
		}
		// Check if the Front Right Motor is out of the Min throttle ranges
		if (FR_Motor < Min_Throttle)
		{
			FR_Motor = Min_Throttle;
		}
	}
	if(Positional_Pitch < -2)
	{
		BL_Motor += (Positional_Pitch / Adujst_Value);
		BR_Motor += (Positional_Pitch / Adujst_Value);

		// Check if the Back Left Motor is out of the Min throttle ranges
		if (BL_Motor < Min_Throttle)
		{
			BL_Motor = Min_Throttle;
		}
		// Check if the Back Right Motor is out of the Min throttle ranges
		if (BR_Motor < Min_Throttle)
		{
			BR_Motor = Min_Throttle;
		}
	}
}

void Get_Pulses(void)
{
	TIM16->CNT = 0;
	uint32_t time_count = TIM16->CNT;
	Remote_Throttle_Command = 0;
	Remote_Roll_Command = 0;
	Remote_Pitch_Command = 0;
	Remote_Yaw_Command = 0;
	while ((TIM16->CNT - time_count) < 2400)
	{
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))
		{
			Remote_Throttle_Command += 1;
		}
		else
		{
			Remote_Throttle_Command = Remote_Throttle_Command;
		}
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
		{
			Remote_Roll_Command += 1;
		}
		else
		{
			Remote_Roll_Command = Remote_Roll_Command;
		}
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
		{
			Remote_Pitch_Command += 1;
		}
		else
		{
			Remote_Pitch_Command = Remote_Pitch_Command;
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
		{
			Remote_Yaw_Command += 1;
		}
		else
		{
			Remote_Yaw_Command = Remote_Yaw_Command;
		}
	}
	//char message[100];
	//sprintf(message, "Read Channel\r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), UART_DELAY);

}

void Send_Pulse(int length)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	HAL_Delay(length);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
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
