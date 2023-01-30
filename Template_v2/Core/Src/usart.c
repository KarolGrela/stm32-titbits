/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
struct MYLIB_UartInstance MYLIB_uart1;
struct MYLIB_UartInstance MYLIB_uart2;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// initialize MYLIB_UartInstance with default values
void uart_InstanceInit(struct MYLIB_UartInstance* uart)
{
	uart->rxBufferIndex = 0;
	uart->rxMessageRecieved = 0;


	uart->txBufferSize = 0;
}

// check if received character is CR
// check if buffer is not overflowing
void uart_Resend(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart)
{
	if(uart->rxBuffer[uart->rxBufferIndex] == (char)0x0D)
	{
		// CR - entire line has been received
		uart_Console(huart, uart);

		// change 'r' to '\0'
		uart->rxBuffer[uart->rxBufferIndex] = '\0';

		// move line to transmitter
		strncpy(uart->txBuffer, uart->rxBuffer, MYLIB_UART_RX_BUFF_SIZE);
		strncat(uart->txBuffer, "\r\n\0", 3);
		uart->txBufferSize = uart->rxBufferIndex + 2;

		HAL_UART_Transmit_IT(&huart2, MYLIB_uart2.txBuffer, strlen(MYLIB_uart2.txBuffer));

		// clear rx variables
		for(int i = 0; i<uart->rxBufferIndex; i++)
		{
			uart->rxBuffer[i] = '\0';
		}
		uart->rxMessageRecieved = 1;
		uart->rxBufferIndex = 0;
	}
	else if(uart->rxBufferIndex < MYLIB_UART_RX_BUFF_SIZE - 1)
	{
		// move the buffer index
		uart->rxBufferIndex++;
	}
	else
	{
		// protect buffer from overflowing
		uart->rxBufferIndex = 0;
	}
}

void uart_Console(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart)
{
	unsigned char turnOn = 0;
	if(uart->rxBuffer[0] == 'A')
	{
		if(uart->rxBuffer[1] == '+')
		{
			turnOn = 1;
		}
		else if(uart->rxBuffer[1] == '-')
		{
			turnOn = 0;
		}
		else if(uart->rxBuffer[1] == '?')
		{
			/*
			unsigned char diodesState = MYLIB_Gpio_GetDiodes();
			unsigned char diodesLine[NO_OF_DIODES+1] = {0};
			for(int i = 0; i < NO_OF_DIODES; i++)
			{
				if(diodesState | (1<<i) == 1)
				{
					diodesLine[i] = '|';
				}
				else
				{
					diodesLine[i] = '_';
				}

			}
			// move line to transmitter
			strncpy(uart->txBuffer, diodesLine, NO_OF_DIODES);
			strncat(uart->txBuffer, "\r\n\0", 3);
			*/
		}
		else
		{
			return;
		}
		unsigned char diodeNumber = uart->rxBuffer[2] - 48;
		if(diodeNumber < 6)
		{
			if(turnOn == 0)
			{
				gpio_DigOutReset(diodeNumber, diodesTable);
			}
			else
			{
				gpio_DigOutSet(diodeNumber, diodesTable);
			}
		}
		else
		{
			return;
		}
	}
	else
	{
		return;
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart)
{
	// check UART instance
	if(uart->Instance == USART2)
	{
		uart_Resend(&huart2, &MYLIB_uart2); // call a callback function
		HAL_UART_Receive_IT(&huart2, MYLIB_uart2.rxBuffer + MYLIB_uart2.rxBufferIndex, 1); // set UART Receiver
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
	// check UART instance
	if(uart->Instance == USART2)
	{
		for(int i = 0; i < MYLIB_uart2.txBufferSize; i++)
		{
			MYLIB_uart2.txBuffer[i] = '\0';
		}
		MYLIB_uart2.txBufferSize = 0;
	}
}

/* USER CODE END 1 */
