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
struct uart_instance_t uart_instance2;
struct uart_instance_t uart_instance3;

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

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
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART2)
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
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC4     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
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
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC4     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*** INITIALIZERS ***/
// initialize uart_instance_t with default values
void uart_InstanceInit(struct uart_instance_t* uart)
{
	circular_buf_Init(&(uart->rxCircBuffer), uart->rxBuffer, UART_RX_BUFF_SIZE);
	circular_buf_Init(&(uart->txCircBuffer), uart->txBuffer, UART_TX_BUFF_SIZE);
}

/*** READ/WRITE ***/
void uart_WriteTx(struct uart_instance_t* uart, char* input_str)
{
	circular_buf_putStr(&uart->txCircBuffer, input_str);
}

char* uart_ReadRx(struct uart_instance_t* uart, char* output_string)
{
	circular_buf_readBuffer(&uart->rxCircBuffer, output_string);
}

/*** TRANSMISSION ***/
// start transmission - send first byte from txBuffer
void uart_StartTransmission(UART_HandleTypeDef* huart, struct uart_instance_t* uart)
{
	if(circular_buff_isEmpty(uart->txCircBuffer) != 1)
	{
		uart->txTransmissionByte = circular_buf_popChar(&(uart->txCircBuffer));
		HAL_UART_Transmit_IT(huart, &(uart->txTransmissionByte), 1);
	}
}

// continue transmission - send next byte or end transmission
// this function should be called by USART TX ISR callback
void uart_TxCallback(UART_HandleTypeDef* huart, struct uart_instance_t* uart)
{
	if(circular_buff_isEmpty(uart->txCircBuffer) == 1)
	{
		unsigned char lastTransmitedByte = uart->txTransmissionByte;
		if(lastTransmitedByte != '\r' && lastTransmitedByte != '\n')
		{
			circular_buf_putStr(&(uart_instance2.txCircBuffer), "\r");
			uart_StartTransmission(huart, uart);
		}
		else if(lastTransmitedByte == '\r')
		{
			circular_buf_putStr(&(uart_instance2.txCircBuffer), "\n");
			uart_StartTransmission(huart, uart);
		}
		else
		{
			// transmission finished
			// do nothing
		}
	}
	else
	{
		uart_StartTransmission(huart, uart);
	}

}

/*** RECEPTION ***/
// put received data to transfer buffer
// this function should be called by USART RX ISR callback
void uart_RxCallback(UART_HandleTypeDef* huart, struct uart_instance_t* uart)
{
	// if rxReceptionByte == '\r'
	if(uart->rxReceptionByte == (char)0x0D)
	{
		// save RX buffer to local string
		char local[circular_buf_CurrentSize(uart->rxCircBuffer)];
		circular_buf_readBuffer(&(uart->rxCircBuffer), local);

		// put local string into TX buffer
		circular_buf_putStr(&(uart->txCircBuffer), local);
		circular_buf_putStr(&(uart->txCircBuffer), "\r\n");

	}
	else
	{
		// put other characters to RX buffer
		circular_buf_putChar(&(uart->rxCircBuffer), uart->rxReceptionByte);
	}
	HAL_UART_Receive_IT(huart, &(uart->rxReceptionByte), 1); // set UART Receiver
}


/*** ISRs ***/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart)
{

	// check UART instance
	if(uart->Instance == USART2)
	{
		uart_RxCallback(uart, &uart_instance2);
	}
	else if(uart->Instance == USART3)
	{
		//uart_RxCallback(uart, &uart_instance3);

		// if rxReceptionByte == '\r'
		if(uart_instance3.rxReceptionByte == (char)0x0D)
		{
			// save RX buffer to local string
			char local[circular_buf_CurrentSize(uart_instance3.rxCircBuffer)];
			circular_buf_readBuffer(&(uart_instance3.rxCircBuffer), local);

			// put local string into TX buffer
			circular_buf_putStr(&(uart_instance2.txCircBuffer), local);
			circular_buf_putStr(&(uart_instance2.txCircBuffer), "\r\n");

		}
		else
		{
			// put other characters to RX buffer
			circular_buf_putChar(&(uart_instance3.rxCircBuffer), uart_instance3.rxReceptionByte);
		}
		HAL_UART_Receive_IT(uart, &(uart_instance3.rxReceptionByte), 1); // set UART Receiver
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{

	// check UART instance
	if(uart->Instance == USART2)
	{
		uart_TxCallback(uart, &uart_instance2);
	}
	else if(uart->Instance == USART3)
	{
		uart_TxCallback(uart, &uart_instance3);
	}
}

/* USER CODE END 1 */
