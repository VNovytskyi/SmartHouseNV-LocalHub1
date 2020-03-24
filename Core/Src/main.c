/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "math.h"

#include "NRF.h"
#include "NRF_ShiftRegister.h"
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
uint8_t mainHubAddr[] = {1, 1, 1, 1, 1};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void InputMessageHandler(char *message);
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  NRF_SetDefaultSettings();
  SR_SetValue(0x0000);
  /* USER CODE END 2 */
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool sendAnswer = false;
  while (1)
  {
  	if(NRF_IsAvailablePacket())
		{
			NRF_GetPacket(NRF_rxBuff);

			InputMessageHandler(NRF_rxBuff);

			sendAnswer = true;
			sprintf(NRF_txBuff, NRF_rxBuff);
			NRF_ClearRxBuff();
		}

		if(sendAnswer)
		{
			sendAnswer = false;
			uint8_t sendMessage = NRF_SendMessage(mainHubAddr, NRF_txBuff);
			NRF_ClearTxBuff();



			//char buff[128] = {0};
			//sprintf(buff, "STM send message (%d) [%d]: %s",sendMessage, strlen(sendBuff), sendBuff);
			//HAL_UART_Transmit(&huart1, buff, strlen(buff), 1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void InputMessageHandler(char *message)
{
	uint8_t cursorPosition = 0;

	switch(message[cursorPosition])
	{
		case 0x01: HAL_GPIO_WritePin(BuildInLed_GPIO_Port, BuildInLed_Pin, 0); break;
		case 0x02: HAL_GPIO_WritePin(BuildInLed_GPIO_Port, BuildInLed_Pin, 1); break;

		case 0x03: SR_SetPin(0); break;
		case 0x04: SR_ResetPin(0); break;

		case 0x05: SR_SetPin(1); break;
		case 0x06: SR_ResetPin(1); break;

		case 0x07: SR_SetPin(2); break;
		case 0x08: SR_ResetPin(2); break;

		case 0x09: SR_SetPin(3); break;
		case 0x0A: SR_ResetPin(3); break;

		case 0x0B: SR_SetPin(4); break;
		case 0x0C: SR_ResetPin(4); break;

		case 0x0D: SR_SetPin(5); break;
		case 0x0E: SR_ResetPin(5); break;

		case 0x0F: SR_SetPin(6); break;
		case 0x10: SR_ResetPin(6); break;

		case 0x11: SR_SetPin(7); break;
		case 0x12: SR_ResetPin(7); break;

		case 0x13: SR_SetPin(8); break;
		case 0x14: SR_ResetPin(8); break;

		case 0x15: SR_SetPin(9); break;
		case 0x16: SR_ResetPin(9); break;

		case 0x17: SR_SetPin(10); break;
		case 0x18: SR_ResetPin(10); break;

		case 0x19: SR_SetPin(11); break;
		case 0x1A: SR_ResetPin(11); break;

		case 0x1B: SR_SetPin(12); break;
		case 0x1C: SR_ResetPin(12); break;

		case 0x1D: SR_SetPin(13); break;
		case 0x1E: SR_ResetPin(13); break;

		case 0x1F: SR_SetPin(14); break;
		case 0x20: SR_ResetPin(14); break;

		case 0x21: SR_SetPin(15); break;
		case 0x22: SR_ResetPin(15); break;



		/* TIM2_CHANNEL_1 */
		case 0x23:
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			TIM2->CCR1 = ((uint16_t)message[++cursorPosition] << 8) | message[++cursorPosition];
			break;

		case 0x24:
			TIM2->CCR1 = 0;
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			break;

		/* TIM2_CHANNEL_2 */
		case 0x25:
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			TIM2->CCR2 = ((uint16_t)message[++cursorPosition] << 8) | message[++cursorPosition];
			break;

		case 0x26:
			TIM2->CCR2 = 0;
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			break;


		/* TIM4_CHANNEL_4 */
		case 0x27:
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
			TIM4->CCR4 = ((uint16_t)message[++cursorPosition] << 8) | message[++cursorPosition];
			break;

		case 0x28:
			TIM4->CCR4 = 0;
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			break;

		/* TIM4_CHANNEL_3 */
		case 0x29:
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			TIM4->CCR3 = ((uint16_t)message[++cursorPosition] << 8) | message[++cursorPosition];
			break;

		case 0x3A:
			TIM4->CCR3 = 0;
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			break;

		/* TIM4_CHANNEL_2 */
		case 0x3B:
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			TIM4->CCR2 = ((uint16_t)message[++cursorPosition] << 8) | message[++cursorPosition];
			break;

		case 0x3C:
			TIM4->CCR2 = 0;
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
			break;

		/* TIM4_CHANNEL_1 */
		case 0x3D:
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			TIM4->CCR1 = ((uint16_t)message[++cursorPosition] << 8) | message[++cursorPosition];
			break;

		case 0x3E:
			TIM4->CCR1 = 0;
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			break;



		//TODO: Input

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
	while(true)
	{
		HAL_GPIO_TogglePin(BuildInLed_GPIO_Port, BuildInLed_Pin);
		HAL_Delay(100);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
