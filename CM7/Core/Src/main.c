/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
#define CCRValue_BufferSize     37
#define CCRValue_NSize			72
volatile uint16_t _index = 0;
volatile uint32_t _duty = 0;
volatile uint8_t step = 0;
volatile uint64_t counter_ = 0;
volatile uint32_t arr4_ = 10000;
volatile uint32_t arr1_ = 30000;

ALIGN_32BYTES (uint32_t DiscontinuousSineCCRValue_Buffer[CCRValue_BufferSize]) =
{
  14999, 17603, 20128, 22498, 24640, 26488, 27988, 29093, 29770,
  29998, 29770, 29093, 27988, 26488, 24640, 22498, 20128, 17603,
  14999, 12394, 9869, 7499, 5357, 3509, 2009, 904, 227, 1, 227,
  904, 2009, 3509, 5357, 7499, 9869, 12394, 14999
};

uint32_t CCRValue_30000[CCRValue_BufferSize] =
{
	14999, 17603, 20128, 22498, 24640, 26488, 27988, 29093, 29770,
	29998, 29770, 29093, 27988, 26488, 24640, 22498, 20128, 17603,
	14999, 12394, 9869, 7499, 5357, 3509, 2009, 904, 227, 1, 227,
	904, 2009, 3509, 5357, 7499, 9869, 12394, 14999
};

uint32_t CCRValue_7500[CCRValue_BufferSize] =
{
		3749,4400,5031,5623,6158,6620,6995,7271,7441,7498,7441,7271,6995
		,6620,6158,5623,5031,4400,3749,3097,2466,1874,1339,877,502,226,56,0
		,56,226,502,877,1339,1874,2466,3097,3749
};

uint32_t CCRValue_3750[CCRValue_BufferSize] =
{
		1874,2199,2514,2811,3078,3309,3496,3634,3719,3748,3719,3634,3496
		,3309,3078,2811,2514,2199,1874,1548,1233,937,669,438,251,113
		,28,0,28,113,251,438,669,936,1233,1548,1874
};

uint32_t CCRValue_937_5[CCRValue_BufferSize] =
{
		467,548,627,701,768,826,872,907,928,935,928,907,872,826,768,701
		,627,548,467,386,307,233,167,109,62,28,7,0,7,28,62,109,167,233
		,307,386,467
};

uint32_t CCRValue_468_75[CCRValue_BufferSize] =
{
		233,273,313,350,383,412,435,452,463,466,463,452,435,412,383,350
		,313,273,233,192,153,116,83,54,31,14,3,0,3,14,31,54,83,116,153
		,192,233
};

uint32_t CCRValue_937_5_72[CCRValue_NSize] =
{
		467,508,548,588,627,665,701,736,768,798,826,850,872,891,907,919
		,928,933,935,933,928,919,907,891,872,850,826,798,768,736,701,665
		,627,588,548,508,467,426,386,346,307,270,233,199,167,137,109,84
		,62,43,28,15,7,1,1,7,15,28,43,62,84,109,137,167,199,233,270,307
		,346,386,426,467
};

uint32_t CCRValue_468_75_72[CCRValue_NSize] =
{
		233,253,273,293,313,332,350,367,383,398,412,424,435,444,452,458
		,463,465,466,465,463,458,452,444,435,424,412,398,383,367,350,332
		,313,293,273,253,233,213,192,172,153,134,116,99,83,68,54,42,31,21
		,14,7,3,0,0,3,7,14,21,31,42,54,68,83,99,116,134,153,172,192,213,233
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void SetCommutationStep(uint8_t step, uint16_t duty);
void threeSine(uint16_t degree);
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
	/* Enable I-Cache---------------------------------------------------------*/
	  //SCB_EnableICache();

	  /* Enable D-Cache---------------------------------------------------------*/
	  //SCB_EnableDCache();
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Clean Data Cache to update the content of the SRAM to be used by the DMA */
    //SCB_CleanDCache_by_Addr((uint32_t *) DiscontinuousSineCCRValue_Buffer, CCRValue_BufferSize );
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT( &htim1 );
  HAL_TIM_Base_Start_IT( &htim4 );
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  counter_ = __HAL_TIM_GET_COUNTER(&htim4);
  //HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, DiscontinuousSineCCRValue_Buffer, CCRValue_BufferSize);
  //HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_1, DiscontinuousSineCCRValue_Buffer, CCRValue_BufferSize);
  //HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, DiscontinuousSineCCRValue_Buffer, CCRValue_BufferSize);
  //HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_2, DiscontinuousSineCCRValue_Buffer, CCRValue_BufferSize);
  //HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, DiscontinuousSineCCRValue_Buffer, CCRValue_BufferSize);
  //HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_3, DiscontinuousSineCCRValue_Buffer, CCRValue_BufferSize);
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 30000 - 1;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  htim4.Init.Prescaler = 200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 800-1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//_duty = CCRValue_7500[_index];
	if (step < 6)
	{
		//SetCommutationStep(step,_duty);
		//step++;
	}
	else if (step == 6) {
			//step = 0;
	}
	//++_index;
	counter_ = __HAL_TIM_GET_COUNTER(htim);
	//if(_index == CCRValue_BufferSize) _index = 0;
	__HAL_TIM_SET_COUNTER(htim , 0);
	//counter_ = __HAL_TIM_GET_COUNTER(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//_duty = CCRValue_937_5_72[_index];
	//SetCommutationStep(step,_duty);
	threeSine(_index);
	++_index;
	if(_index == CCRValue_NSize)
	{
		_index = 0;
		/*if (step < 5)
		{
			++step;
		}
		else if (step == 5) {
				step = 0;
		}*/
	}
	__HAL_TIM_SET_AUTORELOAD( &htim4, arr4_);
	__HAL_TIM_SET_AUTORELOAD( &htim1, arr1_);
}

void SetCommutationStep(uint8_t step, uint16_t duty) {
    switch (step) {
        case 0:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
        case 1:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 3:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
        case 2:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, 0 );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 4:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0 );
        	  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
        	  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 5:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, 0 );
        	  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
        	  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );

            break;
        default:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
    }
}

void threeSine(uint16_t degree)
{
	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, CCRValue_937_5_72[degree] );
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );

	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, CCRValue_937_5_72[(degree + 24) % CCRValue_NSize] );
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );

	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, CCRValue_937_5_72[(degree + 48) % CCRValue_NSize] );
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
	HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
}

/*
 void SetCommutationStep(uint8_t step, uint16_t duty) {
    switch (step) {
        case 0:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
        case 1:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 3:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
        	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_1 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
        case 2:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 4:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	__HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, duty );
        	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_1 );
        	  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1 );
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );
            break;
        case 5:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_2, duty );
            HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_2 );
        	  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2 );
            __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_3, duty );
            HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3 );
            HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );

            break;
        default:
        	//__HAL_TIM_SET_AUTORELOAD( &htim1, 101);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
            break;
    }
}
*/

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
