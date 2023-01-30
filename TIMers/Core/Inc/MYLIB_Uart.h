/*
 * MYLIB_Uart.h
 *
 *  Created on: Jan 11, 2023
 *      Author: kgrela1
 */

#ifndef INC_MYLIB_UART_H_
#define INC_MYLIB_UART_H_

#include "main.h"
#include "MYLIB_Gpio.h"
#include <string.h>

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
	volatile unsigned char txTransferMessage;
};

// initialize MYLIB_UartInstance with default values
void MYLIB_UartInstance_Init(struct MYLIB_UartInstance* uart);

// check if received character is CR
// check if buffer is not overflowing
void MYLIB_Uart_Resend(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart);
void MYLIB_Uart_Console(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart);

void MYLIB_Uart_MoveToTransmitter(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart, const char* line, const unsigned char line_size);

#endif /* INC_MYLIB_UART_H_ */
