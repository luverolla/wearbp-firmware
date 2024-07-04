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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

#include "dsp/statistics_functions.h"
#include "dsp/basic_math_functions.h"
#include "dsp/filtering_functions.h"

#include "signal_utils.h"
#include "model_coefs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#if CFG_MODE == MODE_DEBUG
#define  ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)
#define  ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)
#define  ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)
#endif

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc_buf[CFG_CHUNKLEN] = {0};
float tmp_buf[CFG_SIGLEN] = {0};
float signorm[CFG_SIGLEN];
size_t count = 0;
uint8_t sig_ready = 0;
float samp_raw, samp_rec;
float curr_pred;
float features[13];
float tmp[1000];
sig_fiducials fid;

arm_fir_instance_f32 fir;
	float fir_state[CFG_CHUNKLEN + FILT_FIR_NTAPS - 1];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  memset(fir_state, 0, sizeof(float)*(CFG_CHUNKLEN+FILT_FIR_NTAPS-1));
  	arm_fir_init_f32(&fir, FILT_FIR_NTAPS, FILT_FIR_COEFS, fir_state, CFG_CHUNKLEN);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_DMA(&hadc1, adc_buf, CFG_CHUNKLEN);
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	float* cell = tmp_buf + count*CFG_CHUNKLEN;

	for (size_t i = 0; i < CFG_CHUNKLEN; i++) {
		cell[i] = adc_buf[i] * 1.0f;
		cell[i] = cell[i] / 4095.0f * 3.3f;
	}

	arm_fir_f32(&fir, cell, cell, CFG_CHUNKLEN);
	//HAL_UART_Transmit_DMA(&huart2, cell, 4*CFG_CHUNKLEN);

	if (count == CFG_WINSIZE - 1) {
		count = 0;

		sig_norm(tmp_buf+5*125, CFG_SIGLEN-5*125, signorm, SIG_NORM_RANGE);
		sig_get_fiducials(signorm, CFG_SIGLEN-5*125, &fid);

		sig_featex(tmp_buf+5*125, CFG_SIGLEN-5*125, fid, tmp, features);

		arm_sub_f32(features, SVM_SMIN, features, 13);
		for (size_t i = 0; i < 13; i++) {
			features[i] /= SVM_SDIF[i];
		}

		arm_dot_prod_f32(features, SVM_BETA, 13, &curr_pred);
		curr_pred += SVM_BIAS;

		HAL_UART_Transmit_DMA(&huart2, &curr_pred, 4);

	} else {
		count += 1;
	}
}

/*
 sig_norm(recx, SIGNAL_LENGTH, normed, SIG_NORM_RANGE);
sig_fiducials fid;
sig_get_fiducials(normed, SIGNAL_LENGTH, &fid);

float tmp[SIGNAL_LENGTH];
float feat_mat[19];
float pred;
uint8_t isgood = sig_check(normed, SIGNAL_LENGTH, fid, tmp);
if (isgood) {
	sig_featex(recx, SIGNAL_LENGTH, fid, tmp, feat_mat);

	// predict SVM y = <x,beta>+bias
	arm_dot_prod_f32(feat_mat, SVM_BETA, 19, &pred);
	pred += SVM_BIAS;
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
