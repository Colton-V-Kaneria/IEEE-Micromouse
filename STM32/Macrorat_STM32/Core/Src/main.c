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
#include <math.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	DIST_FL,
	DIST_FR,
	DIST_R,
	DIST_L
} dist_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define diameter 33		// wheel diameter
#define RW 41			// radius from center to wheel
#define max_PWM 1600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t dis_FL;
uint16_t dis_FR;
uint16_t dis_L;
uint16_t dis_R;
int32_t enc_left = 0;
int32_t enc_right = 0;
int32_t d_L = 0;
int32_t d_R = 0;
int32_t d_center = 0;	// center distance
int32_t angle = 0;
uint16_t raw_count_left = 0;
uint16_t raw_count_right = 0;
uint16_t prev_count_left = 0;
uint16_t prev_count_right = 0;
int motorL = 0;
int motorR = 0;

const int base_PWM = 1300;
const int K_rot = 5;
// const int K_fwd = ?;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
static void ADC1_Select_CH4(void);
static void ADC1_Select_CH5(void);
static void ADC1_Select_CH8(void);
static void ADC1_Select_CH9(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t measure_dist(dist_t dist) {
	GPIO_TypeDef* emitter_port;
	uint16_t emitter_pin;
	GPIO_TypeDef* receiver_port;
	uint16_t receiver_pin;

	switch(dist) {
		case DIST_FL:
			emitter_port = EMIT_FL_GPIO_Port;
			emitter_pin = EMIT_FL_Pin;
			receiver_port = RECIV_FL_GPIO_Port;
			receiver_pin = RECIV_FL_Pin;
			ADC1_Select_CH9();
			break;
		case DIST_L:
			emitter_port = EMIT_L_GPIO_Port;
			emitter_pin = EMIT_L_Pin;
			receiver_port = RECIV_L_GPIO_Port;
			receiver_pin = RECIV_L_Pin;
			ADC1_Select_CH8();
			break;
		case DIST_R:
			emitter_port = EMIT_R_GPIO_Port;
			emitter_pin = EMIT_R_Pin;
			receiver_port = RECIV_R_GPIO_Port;
			receiver_pin = RECIV_R_Pin;
			ADC1_Select_CH5();
			break;
		case DIST_FR:
			emitter_port = EMIT_FR_GPIO_Port;
			emitter_pin = EMIT_FR_Pin;
			receiver_port = RECIV_FR_GPIO_Port;
			receiver_pin = RECIV_FR_Pin;
			ADC1_Select_CH4();
			break;
		default:
			break;
	}

	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_SET);
	HAL_Delay(5);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_RESET);

	return adc_val;
}

int calc_distance()
{
	return (d_L + d_R)/2;
}

int calc_angle()
{
	int angle = (int)((d_R - d_L)/(2.0 * RW) * (180.0/M_PI)) % 360;

	// These next statements ensure the result is between -180 and 180
	if (angle > 180)
	{
		angle -= 360;
	}
	else if (angle < -180)
	{
		angle += 360;
	}

	return angle;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// this is the left encoder timer
	if (htim->Instance == TIM3) {
		//enc_left = __HAL_TIM_GET_COUNTER(htim);
		raw_count_left = __HAL_TIM_GET_COUNTER(htim);
		enc_left -= (int16_t)(raw_count_left - prev_count_left);
		d_L = (enc_left / 360.0) * (M_PI * diameter);
		d_center = calc_distance();
		angle = calc_angle();

		prev_count_left = raw_count_left;
	}
	if (htim->Instance == TIM4) {
		//enc_right = __HAL_TIM_GET_COUNTER(htim);
		raw_count_right = __HAL_TIM_GET_COUNTER(htim);
		enc_right -= (int16_t)(raw_count_right - prev_count_right);
		d_R = (enc_right / 360.0) * (M_PI * diameter);
		d_center = calc_distance();
		angle = calc_angle();

		prev_count_right = raw_count_right;
	}
}

void motor_test()
{
	//HAL_TIM_IC_CaptureCallback(&htim2);

	TIM2->CCR4 = 1350; // right motor
	TIM2->CCR3 = 1350; // left motor

	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1);
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);
	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1);
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);
	HAL_Delay(1500);

	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0); // set both to LOW to stop
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);
	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0); // set both to LOW to stop
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);
	HAL_Delay(1500);

	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0);
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 1);
	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0);
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 1);
	HAL_Delay(1500);

	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0); // set both to LOW to stop
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);
	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0); // set both to LOW to stop
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);
	HAL_Delay(1500);
}

void IR_test()
{
	dis_FR = measure_dist(DIST_FR);
	dis_FL = measure_dist(DIST_FL);
	dis_R = measure_dist(DIST_R);
	dis_L = measure_dist(DIST_L);
}

int min(int a, int b)
{
	return (a < b) ? a : b;
}


int max(int a, int b)
{
	return (a > b) ? a : b;
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

//	HAL_Delay(5000); // 5 second delay until motors spin

  TIM2->CCR4 = 1200; // right motor
  TIM2->CCR3 = 1200; // left motor

  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1);
  HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);
  HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1);
  HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
//	  motorL = TIM2->CCR3;
//	  motorR = TIM2->CCR4;
//
//	  angle = calc_angle();
//
//	  TIM2->CCR4 = base_PWM - K_rot * angle;
//	  TIM2->CCR3 = base_PWM + K_rot * angle;
//
//	  // end the while loop if the motor speeds start to become erratic
//	  if (motorL > 2000 || motorR > 2000 || motorL < 500 || motorR < 500)
//	  {
//		  TIM2->CCR4 = 0;
//		  TIM2->CCR3 = 0;
//		  break;
//	  }

	  IR_test();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  //HAL_Delay(500);  /* Insert delay 500 ms */
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2047;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EMIT_R_Pin|EMIT_L_Pin|EMIT_FL_Pin|MR_BWD_Pin
                          |ML_BWD_Pin|MR_FWD_Pin|EMIT_FR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EMIT_R_Pin EMIT_L_Pin EMIT_FL_Pin MR_BWD_Pin
                           ML_BWD_Pin MR_FWD_Pin EMIT_FR_Pin */
  GPIO_InitStruct.Pin = EMIT_R_Pin|EMIT_L_Pin|EMIT_FL_Pin|MR_BWD_Pin
                          |ML_BWD_Pin|MR_FWD_Pin|EMIT_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ML_FWD_Pin */
  GPIO_InitStruct.Pin = ML_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ML_FWD_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void ADC1_Select_CH4(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void ADC1_Select_CH5(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void ADC1_Select_CH8(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void ADC1_Select_CH9(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
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
