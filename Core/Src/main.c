/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  MODE_PWR_OFF,
  MODE_PWR_ON,
  MODE_1,
  MODE_2,
  MODE_3,
} mode_t;

typedef enum {
	INTENSITY_0,
	INTENSITY_1,
	INTENSITY_2,
	INTENSITY_3,
	INTENSITY_4,
	INTENSITY_5,
} intensity_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ledir_on() HAL_GPIO_WritePin(LEDIR_GPIO_Port, LEDIR_Pin, GPIO_PIN_SET)
#define ledir_off() HAL_GPIO_WritePin(LEDIR_GPIO_Port, LEDIR_Pin, GPIO_PIN_RESET)
#define led1_on() HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define led1_off() HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define led2_on() HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define led2_off() HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define led3_on() HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define led3_off() HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define is_sw1_pushed() (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET)
#define is_sw2_pushed() (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET)
#define is_sw3_pushed() (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_SET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
mode_t mode = MODE_PWR_OFF;
intensity_t intensity = INTENSITY_0;
bool main_pwr = False;

int cnt_pwm_buzzer;

bool flag_dma_finished;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void beep(void)
{
	cnt_pwm_buzzer = 0;
	HAL_TIM_PWM_Start_IT(&htim21, TIM_CHANNEL_1);
}

void led_all_off(void)
{
	ledir_off();
	led1_off();
	led2_off();
	led3_off();
}

void power_on(void)
{
	RTC_TimeTypeDef sTime;

	main_pwr = True;
	HAL_GPIO_WritePin(HV_GPIO_Port, HV_Pin, GPIO_PIN_SET);

	sTime.Seconds = 0;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

//	led1_on();
//	led2_on();
//	led3_on();
//	HAL_Delay(100);
//	led_all_off();
//	HAL_Delay(100);
//	led1_on();
//	led2_on();
//	led3_on();
//	HAL_Delay(100);
//	led_all_off();
//	HAL_Delay(500);
}

void power_off(void)
{
	main_pwr = False;
	HAL_GPIO_WritePin(HV_GPIO_Port, HV_Pin, GPIO_PIN_RESET);

	HAL_Delay(100);
	led_all_off();
	HAL_SuspendTick();
	__HAL_RCC_PWR_CLK_ENABLE();
	//HAL_PWR_EnableSleepOnExit();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	HAL_ResumeTick();
	SystemClock_Config();
	NVIC_SystemReset();
}

void mode1_work(void)
{
//	flag_dma_finished = False;
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t*)sine_dat, SINE_CNT);
//	while (!flag_dma_finished) {};
//	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
//
//	flag_dma_finished = False;
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t*)sine_dat, SINE_CNT);
//	while (!flag_dma_finished) {};
//	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	if (sTime.Minutes == 1) mode = MODE_PWR_OFF;
}

void mode2_work(void)
{/*
	int i = 0;
	uint16_t pulse_dat[64];

	for (; i < 32; i++) {
		pulse_dat[i] = 999;
	}
	for (; i < 64; i++) {
		pulse_dat[i] = 0;
	}
	flag_dma_finished = False;
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t*)pulse_dat, 64);
	while (!flag_dma_finished) {};
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);*/
}

void mode3_work(void)
{

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
  MX_TIM2_Init();
  MX_TIM21_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	if (is_sw2_pushed()) {
		mode = MODE_PWR_ON;
	} else {
		mode = MODE_PWR_OFF;
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(mode) {
		  case MODE_PWR_OFF:
			  power_off();
			  break;
		  case MODE_PWR_ON:
			  beep();
			  power_on();

			  led1_on();
			  mode = MODE_1;
			  break;
		  case MODE_1:
			  mode1_work();
			  break;
		  case MODE_2:
			  mode2_work();
			  break;
		  case MODE_3:
			  mode3_work();
			  break;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
		case SW2_Pin:
			if (mode == MODE_1) {
				led1_off();
				beep();
				led2_on();
				mode = MODE_2;
			} else if (mode == MODE_2) {
				led2_off();
				beep();
				led3_on();
				mode = MODE_3;
			} else if (mode == MODE_3) {
				led3_off();
				beep();
				mode = MODE_PWR_OFF;
			}
			break;
		case SW1_Pin:
			if (main_pwr) {
				if (intensity != INTENSITY_0) {
					beep();
					intensity--;
				}
			}
			break;
		case SW3_Pin:
			if (main_pwr) {
				if (intensity != INTENSITY_5) {
					beep();
					intensity++;
				}
			}
			break;
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		flag_dma_finished = True;
	} else if (htim->Instance == TIM21) {
		if (++cnt_pwm_buzzer >= 400) {
			HAL_TIM_PWM_Stop_IT(&htim21, TIM_CHANNEL_1);
		}
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	mode = MODE_PWR_OFF;
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
