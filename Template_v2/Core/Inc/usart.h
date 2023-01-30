/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "gpio.h"
#include <string.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define MYLIB_UART_RX_BUFF_SIZE 32
#define MYLIB_UART_TX_BUFF_SIZE 64

struct MYLIB_UartInstance
{
	// receiver VARIABLES
	volatile unsigned char rxBuffer[MYLIB_UART_RX_BUFF_SIZE];
	volatile unsigned char rxBufferIndex;
	volatile unsigned char rxMessageRecieved;		// flag set to 1 when message is received

	// transmitter
	volatile unsigned char txBuffer[MYLIB_UART_TX_BUFF_SIZE];
	volatile unsigned char txBufferSize;
};

extern struct MYLIB_UartInstance MYLIB_uart1;
extern struct MYLIB_UartInstance MYLIB_uart2;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

// initialize MYLIB_UartInstance with default values
void uart_InstanceInit(struct MYLIB_UartInstance* uart);

// check if received character is CR
// check if buffer is not overflowing
void uart_Resend(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart);

void uart_Console(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

