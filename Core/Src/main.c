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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
  /* USER CODE BEGIN 2 */
  NRF_SetDefaultSettings();
  SR_SetValue(0x0000);

  /*
     * DEBUG
     */
    uint8_t status = NRF_ReadReg(NRF_REG_STATUS);
    uint8_t rxAddrP0[5];
    NRF_ReadMBReg(NRF_REG_RX_ADDR_P0, rxAddrP0, 5);
    uint8_t rxAddrP1[5];
    NRF_ReadMBReg(NRF_REG_RX_ADDR_P1, rxAddrP1, 5);
    //rx addr p2-5
    uint8_t txAddr[5];
    NRF_ReadMBReg(NRF_REG_TX_ADDR, txAddr, 5);
    //rx payload width
    uint8_t enableAA = NRF_ReadReg(NRF_REG_EN_AA);
    uint8_t enableRXAddr = NRF_ReadReg(NRF_REG_EN_RXADDR);
    uint8_t setupAW = NRF_ReadReg(NRF_REG_SETUP_AW);
    uint8_t setuptRETR = NRF_ReadReg(NRF_REG_SETUP_RETR);
    uint8_t channel = NRF_ReadReg(NRF_REG_RF_CH);
    uint8_t rfSetup = NRF_ReadReg(NRF_REG_RF_SETUP);
    uint8_t config = NRF_ReadReg(NRF_REG_CONFIG);
    uint8_t dynpd = NRF_ReadReg(NRF_REG_DYNPD);
    uint8_t feature = NRF_ReadReg(NRF_REG_FEATURE);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool sendAnswer = false;
  while (1)
  {
  	if(NRF_IsAvailablePacket())
  		NRF_AvailablePacket = true;

  	if(NRF_AvailablePacket)
		{
			NRF_AvailablePacket = false;

			uint8_t readData[32] = {0};
			NRF_GetPacket(&readData);

			strcat(NRF_MessageBuff, readData);

			if(strchr(NRF_MessageBuff, '\n') || strlen(NRF_MessageBuff) < 29)
				NRF_AvailableMessage = true;
		}

		if(NRF_AvailableMessage)
		{
			NRF_AvailableMessage = false;
			sendAnswer = true;

			InputMessageHandler(NRF_MessageBuff);

			char buff[512] = {0};
			sprintf(buff, "STM get message [%d]: %s", strlen(NRF_MessageBuff), NRF_MessageBuff);
			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1000);

			NRF_ClearMessageBuff();
		}

		if(sendAnswer)
		{
			sendAnswer = false;
			char buff[128] = {0};
			uint8_t *sendBuff = "Hello";
			uint8_t localHub1[] = {'2', 'N', 'o', 'd', 'e'};

			uint8_t sendMessage = NRF_SendMessage(localHub1, sendBuff);

			sprintf(buff, "STM send message (%d) [%d]: %s",sendMessage, strlen(sendBuff), sendBuff);
			HAL_UART_Transmit(&huart1, buff, strlen(buff), 1000);
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BuildInLed_GPIO_Port, BuildInLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CSN_Pin|SPI1_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : BuildInLed_Pin */
  GPIO_InitStruct.Pin = BuildInLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BuildInLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI1_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CSN_Pin SPI1_CE_Pin */
  GPIO_InitStruct.Pin = SPI1_CSN_Pin|SPI1_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		NRF_CallbackFunc();
}

void InputMessageHandler(char *message)
{
	uint8_t cursorPosition = 0;
	uint8_t messageLen = strlen(message);

	//TODO: Организовать цикл
	char currentValue = message[cursorPosition];

	switch(currentValue)
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


		//default:
			//if(cursorPosition < messageLen)
				//++cursorPosition;
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
