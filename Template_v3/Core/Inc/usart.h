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
#include "circular_buf_t.h"
#include <string.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define UART_RX_BUFF_SIZE 32
#define UART_TX_BUFF_SIZE 32

struct uart_instance_t
{
	// receiver
	volatile unsigned char rxBuffer[UART_RX_BUFF_SIZE];			// received bytes are stored here
	volatile struct circular_buf_t rxCircBuffer;				// structure implementing circular buffering, referencing rxBuffer
	unsigned char rxReceptionByte;								// byte used by HAL_UART_Receive_IT(&huart, uart_instance2.rxReceptionByte, 1);

	// transmitter
	volatile unsigned char txBuffer[UART_TX_BUFF_SIZE];			// bytes awaiting transmission are stored here
	volatile struct circular_buf_t txCircBuffer;				// pseudo-class implementing circular buffering
	unsigned char txTransmissionByte;							// byte used by HAL_UART_Transmit_IT(&huart, uart_instance2.txTransmissionByte, 1)
};

extern struct uart_instance_t uart_instance2;

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/*** INITIALIZERS ***/
// initialize uart_instance_t with default values
void uart_InstanceInit(struct uart_instance_t* uart);

/*** TRANSMISSION ***/
// start transmission - send first byte from txBuffer
void uart_StartTransmission(UART_HandleTypeDef* huart, struct uart_instance_t* uart);

// continue transmission - send next byte or end transmission
// this function should be called by USART TX ISR callback
void uart_TxCallback(UART_HandleTypeDef* huart, struct uart_instance_t* uart);

/*** RECEPTION ***/
// put received data to transfer buffer
// this function should be called by USART RX ISR callback
void uart_RxCallback(UART_HandleTypeDef* huart, struct uart_instance_t* uart);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

