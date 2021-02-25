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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define	FORWARD			0
#define	REVERSE			1

#define DELAY_BP		3000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
	uint32_t tickcnt;

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  tickcnt = HAL_GetTick();
  while (1)
  {
	  if(!HAL_GPIO_ReadPin(BP_GPIO_Port, BP_Pin))
	  {
		  if(HAL_GetTick()-tickcnt < DELAY_BP)
		  {
			  HAL_Delay(10);
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			  NewStep(FORWARD);
			  HAL_Delay(10);
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			  NewStep(FORWARD);
		  }
		  else
		  {
			  HAL_Delay(1);
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			  NewStep(FORWARD);
			  HAL_Delay(1);
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			  NewStep(FORWARD);
		  }
	  }

	  else if(!HAL_GPIO_ReadPin(BP_GPIO_Port, BP2_Pin))
	  {
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		  NewStep(REVERSE);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  NewStep(REVERSE);
	  }

	  else
	  {
		  tickcnt = HAL_GetTick();
	  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT1_Pin|OUT2_Pin|OUT3_Pin|OUT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT1_Pin OUT2_Pin OUT3_Pin OUT4_Pin */
  GPIO_InitStruct.Pin = OUT1_Pin|OUT2_Pin|OUT3_Pin|OUT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BP_Pin BP2_Pin */
  GPIO_InitStruct.Pin = BP_Pin|BP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void NewStep(uint8_t direction)
{
	static uint8_t cpt=0;

	switch(cpt)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOB, OUT1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, OUT2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT4_Pin, GPIO_PIN_SET);
		break;

	case 1:
		HAL_GPIO_WritePin(GPIOB, OUT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, OUT4_Pin, GPIO_PIN_SET);
		break;

	case 2:
		HAL_GPIO_WritePin(GPIOB, OUT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, OUT3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, OUT4_Pin, GPIO_PIN_RESET);
		break;

	case 3:
		HAL_GPIO_WritePin(GPIOB, OUT1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, OUT2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, OUT3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT4_Pin, GPIO_PIN_RESET);
		break;

	default:
		HAL_GPIO_WritePin(GPIOB, OUT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, OUT4_Pin, GPIO_PIN_RESET);
		break;
	}

	if(direction == FORWARD)
	{
		if(cpt==3)
			cpt=0;
		else
			cpt++;
	}


	if(direction == REVERSE)
	{
		if(cpt==0)
			cpt=3;
		else
			cpt--;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
