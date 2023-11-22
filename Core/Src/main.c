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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

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

/* USER CODE BEGIN PV */
uint8_t rxData[10] = {0};
uint16_t dt;
float omega;
float ref = 0.0f;

float error = 0.0f; // 偏差
float ierror = 0.0f; // 偏差の積分
float output = 0.0f;
float d_hat = 0.0f; // 外乱の推定値

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setMotor(int);
float LPF(float);
float FGinv(float);
float PID(float, float);

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
	setbuf(stdout, NULL);

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
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LL_USART_EnableIT_RXNE(USART2);

  LL_TIM_EnableCounter(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_OC_SetCompareCH1(TIM1, 0);
  LL_TIM_OC_SetCompareCH2(TIM1, 0);

  LL_TIM_EnableCounter(TIM3);

  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);

  LL_TIM_EnableCounter(TIM7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  printf("ref=%f, error=%f, ierror=%f, output=%f, d_hat=%f\r\n", ref, error, ierror, output, d_hat);
	  LL_mDelay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void __io_putchar(uint8_t c) {
	LL_USART_TransmitData8(USART2, c);
	while (LL_USART_IsActiveFlag_TXE(USART2) == 0);
}

/**
 * @brief 制御器
 * @note 制御周期はプリスケーラとカウンタピリオドの値に依存
 */
void controller()
{
	const float T_ctrl = 0.001f;
	const float PPR = 2048.0f;
	const float CPR = PPR * 4;

	int16_t count = -LL_TIM_GetCounter(TIM3);
	LL_TIM_SetCounter(TIM3, 0);
	omega = (float)count / T_ctrl / CPR * 2 * M_PI; // 角速度を計算

	// 上のローラーのゲイン
	//　const float K_p = 0.590558021696119f;
	//　const float K_i = 6.47134066066387f;
	// const float K_d = 0.0f;
	// 下のローラーのゲイン
	const float K_p = 0.570265010581587f;
	const float K_i = 6.00085498191391f;
	const float K_d = 0.0f;

	static float error_pre; // 前回の偏差
	// static float ierror; // 偏差の積分
	static float derror; // 偏差の微分
	static float output_pre;

	// float error; // 偏差
	if (ref == INFINITY) { // リセット信号が来たとき
		error = 0;
	} else {
		error = ref - omega;
	}
	derror = (error - error_pre) / T_ctrl;
	if ((ref == INFINITY) || (fabsf(ref) < 0.001f)) { // リセット信号が来た or 指令値が0
		ierror = 0;
		output_pre = 0;
	} else {
		ierror = ierror + (error + error_pre) * T_ctrl / 2;
	}
	if ((ref == INFINITY) || (fabsf(ref) < 0.001f)) { // リセット信号が来た or 指令値が0
		output = 0;
	} else {
		// float output = K_p * error + K_i * ierror + K_d * derror;
		output = K_p * error + K_i * ierror + K_d * derror;
	}
	d_hat = LPF(output_pre) - FGinv(omega); // 外乱の推定値を計算
	output += d_hat;

	setMotor((int)output);

	error_pre = error;
	output_pre = output;

	/*
	static int step;
	if (step < 1000) {
		setMotor(300);
		printf("%f,%f\r\n", (float)step * 0.001f, omega);
	} else if (step < 2000) {
		setMotor(600);
		printf("%f,%f\r\n", (float)step * 0.001f, omega);
	} else {
		setMotor(300);
	}
	step += 2;
	*/
}

/**
 * @brief ローパスフィルタ
 * @param input 入力
 * @return 出力
 */
float LPF(float input)
{
	const float T_ctrl = 0.001f;
	const float omega_c = 100.0f;
	float a = omega_c * T_ctrl / (1 + omega_c * T_ctrl);

	static float output_pre;

	float output = (1 - a) * output_pre + a * input;

	output_pre = output;

	return output;
}

/**
 * @brief LPF付きモータの逆システム
 * @param input 入力
 * @return 出力
 */
float FGinv(float input)
{
	// 上
	// const float K = 5.192893047396017f;
	// const float T = 0.10130360113627829f;
	// 下
	const float K = 5.240304638093135f;
	const float T = 0.10560630397470239f;

	const float T_ctrl = 0.001f;
	const float omega_c = 100.0f;

	static float input_pre;
	static float output_pre;

	float output;
	if ((ref == INFINITY) || (fabsf(ref) < 0.001f)) { // リセット信号が来た or 指令値が0
		output = 0;
		output_pre = 0;
	} else {
		output = output_pre / (1 + omega_c * T_ctrl) + input * omega_c * (T + T_ctrl) / K / (1 + omega_c * T_ctrl) - input_pre * omega_c * T / K / (1 + omega_c * T_ctrl);
	}

	input_pre = input;
	output_pre = output;

	return output;
}

/**
 * @brief PID制御器
 * @param input 入力値
 * @param target 目標値
 * @return 操作量
 */
float PID(float input, float target)
{
	const float T_ctrl = 0.001f;
    const float K_p = 0.704852646884575f;
    const float K_i = 8.65943789940659f;
    const float K_d = 0.0f;

    static float error_pre;
    static float ierror;
    static float derror;
    static float output;

    float error = target - input;
    derror = (error - error_pre) / T_ctrl;
    ierror = ierror + (error + error_pre) * T_ctrl / 2;
    output = K_p * error + K_i * ierror + K_d * derror;

    error_pre = error;

    return output;
}

/**
 * @brief あ
 * @param duty デューティ比 [‰]
 */
void setMotor(int duty)
{
	if (duty > 950) {
		duty = 950;
	} else if(duty < -950) {
		duty = -950;
	}

	if (duty > 0) { // 正転
		LL_TIM_OC_SetCompareCH1(TIM1, duty);
		LL_TIM_OC_SetCompareCH2(TIM1, 0);
	} else if (duty < 0) { // 逆転
		LL_TIM_OC_SetCompareCH1(TIM1, 0);
		LL_TIM_OC_SetCompareCH2(TIM1, -duty);
	} else { // 停止
		LL_TIM_OC_SetCompareCH1(TIM1, 0);
		LL_TIM_OC_SetCompareCH2(TIM1, 0);
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
