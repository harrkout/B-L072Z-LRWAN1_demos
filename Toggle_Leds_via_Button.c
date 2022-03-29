/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN_DEBOUNE_DELAY    350U // 350ms button debounce period
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int ld_num = 0;
uint8_t btn_lock = 0U;
uint32_t debounce = 0U;
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* Timer frequency: 180MHz
   * Prescaler: 45000
   * Counter Period: 1000
   * Tick Duration = 1 / Timer frequency / Prescaler = 0.25ms
   * Total time to reach counter period = Tick Duration * Counter Period = 250ms
   */
  HAL_TIM_Base_Start_IT(&htim6); // <-- Timer start / interrupt mode

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t EXTI2_3_IRQn)
{
	/* The first if gets the timers' tick and subtracts the debounce from it.
	 * In that was if we hit the button will be read 1 time and will not skip
	 * the next led.
	 */
	if ( HAL_GetTick() - debounce >= BTN_DEBOUNE_DELAY )
	{
		btn_lock = 0U;
	}


	if ( btn_lock == 0U )
	{
		debounce = HAL_GetTick();
		btn_lock = 1;
		ld_num++;
		ld_num %= 4; // <-- MOD 4 means that our range will be 0-3 (4 leds)
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	  {
		if(ld_num == 0)
	    {
		  // Always on LED
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		  // Toggle LED
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	    }
		else if(ld_num == 1)
	    {
		  // Always on previous LED
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		  // Toggle LED
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	    }
		else if(ld_num == 2)
		{
		  // Always on LEDS
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		  // Toggle LED
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
		}
		else
		{
		  // Always on previous LED
	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
		  // Toggle LED
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		}
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
