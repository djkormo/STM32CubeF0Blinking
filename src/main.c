/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "resources.h"
#include "algorithm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t counter = 0;
volatile uint8_t counteradc = 0;
volatile uint16_t lutindex = 0;
volatile uint16_t value_dac = 0;
volatile uint16_t ADC_rawold[3];
volatile uint16_t ADC_raw[3];
volatile uint16_t ADCConv1, ADCConv2, ADCConv3;
volatile uint16_t ADCConv1old, ADCConv2old, ADCConv3old;

volatile uint8_t index = 0;

volatile uint16_t counttimer1 = 0;
volatile uint16_t counttimer2=0;

// 1st sine
volatile uint32_t accumulator1 = 0;
volatile uint16_t accumulator1angle = 0;
volatile uint16_t accumulator1step = 0;
volatile uint32_t accumulator1r = 15737418;
volatile double VoltValue1 = 0.0;

//2nd sine
volatile uint32_t accumulator2 = 0;
volatile uint16_t accumulator2angle = 0;
volatile uint16_t accumulator2step = 0;
volatile uint32_t accumulator2r = 25737418;
volatile double VoltValue2 = 0.0;

//3rd sine
volatile uint32_t accumulator3 = 0;
volatile uint16_t accumulator3angle = 0;
volatile uint16_t accumulator3step = 0;
volatile uint32_t accumulator3r = 35737418;
volatile double VoltValue3 = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(int count);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_DAC1_Init();
	MX_TIM6_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler(0);
	}
	/*
	 HAL_TIM_Base_Start_IT(&htim6);
	 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		counter++;

		/* USER CODE BEGIN 3 */

		// DAC


		HAL_Delay(1);
		// Timer for DAC
		counttimer1 = __HAL_TIM_GetCounter(&htim6);

		/*	based on
		 http://amarkham.com/?p=49
		 */

		accumulator1r = (uint32_t) rangeScaleLinear(ADCConv1, 0, 4095, 10,
				5000);

		accumulator2r = (uint32_t) rangeScaleLinear(ADCConv2, 0, 4095, 10,
				5000);

		accumulator3r = (uint32_t) rangeScaleLinear(ADCConv3, 0, 4095, 10,
				5000);

		accumulator1 += accumulator1r;
		//  first 10 (32 -22) bits -> lut table index
		accumulator1angle = (uint16_t) (accumulator1 >> 22);

		accumulator1step = Sine1024_12bit[accumulator1angle];

		accumulator2 += accumulator2r;
		//  first 10 (32 -22) bits -> lut table index
		accumulator2angle = (uint16_t) (accumulator2 >> 22);
		accumulator2step = Sine1024_12bit[accumulator2angle];

		accumulator3 += accumulator3r;
		//  first 10 (32 -22) bits -> lut table index
		accumulator3angle = (uint16_t) (accumulator3 >> 22);
		accumulator3step = Sine1024_12bit[accumulator3angle];

	}
	/* USER CODE END 3 */

}

void ADC_IRQHandler() {
	HAL_ADC_IRQHandler(&hadc);

	//HAL_ADC_IRQHandler(&hadc2); <--- In case of a second ADC
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) {
		//ADC_rawold[index]=ADC_raw[index];
		ADC_raw[index] = HAL_ADC_GetValue(hadc);
		index++;
		counteradc++;
	}

	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS)) {
		index = 0;
		counteradc++;
		ADCConv1 = ADC_raw[0];
		ADCConv2 = ADC_raw[1];
		ADCConv3 = ADC_raw[2];
		/*filtr <- 0.1*nowa_próbka_z_adc + 0.9*filtr; wyjœcie <- filtr;*/


	}

}

void TIM6_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) //check if the interrupt comes from TIM6
	{
		HAL_GPIO_TogglePin(GPIOC, LD4_Pin);
		counttimer2++;

		value_dac = Sine1024_12bit[lutindex];
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value_dac);
				lutindex+=(int)(ADCConv1/50);

				if (lutindex >= 1024) {
					lutindex -= 1024;
				}


	}

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler(0);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler(0);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE; //DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler(0);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	/* Set ADC callback */

	HAL_ADC_ConvCpltCallback(&hadc);

	/* ADC Calibration */

	if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) {
		Error_Handler(0);
	}

	if (HAL_ADC_Start_IT(&hadc) != HAL_OK) {
		Error_Handler(0);
	}

}

/* DAC1 init function */
static void MX_DAC1_Init(void) {

	DAC_ChannelConfTypeDef sConfig;

	/**DAC Initialization
	 */
	hdac1.Instance = DAC;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler(0);
	}

	/**DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE; //DAC_TRIGGER_T2_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE; //DAC_OUTPUTBUFFER_ENABLE
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler(0);
	}

}

/* TIM6 init function */
static void MX_TIM6_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	__TIM6_CLK_ENABLE();
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 500;
	htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	// htim6.Init.RepetitionCounter = 0;
	// htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim6.Init.RepetitionCounter = 0;

	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler(0);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(0);
	}
	// Turn On TIM6 with Interupts
	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
		Error_Handler(0);
	}

	HAL_NVIC_SetPriority(TIM6_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__TIM6_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LD4_Pin | LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LD4_Pin LD3_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// GPIO for DAC Output-> PA4
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(int count) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		for (int i = 0; i <= count; i++) {
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_SET);
		}
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
