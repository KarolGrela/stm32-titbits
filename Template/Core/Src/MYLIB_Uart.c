/*
 * MYLIB_Uart.c
 *
 *  Created on: Jan 11, 2023
 *      Author: kgrela1
 */


#include "MYLIB_Uart.h"

/*** Static Functions ***/


// initialize MYLIB_UartInstance with default values
void MYLIB_UartInstance_Init(struct MYLIB_UartInstance* uart)
{
	uart->rxBufferIndex = 0;
	uart->rxMessageRecieved = 0;


	uart->txBufferSize = 0;
}

// check if received character is CR
// check if buffer is not overflowing
void MYLIB_Uart_Resend(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart)
{
	if(uart->rxBuffer[uart->rxBufferIndex] == (char)0x0D)
	{

		// CR - entire line has been received

		MYLIB_Uart_Console(huart, uart);

		// change 'r' to '\0'
		uart->rxBuffer[uart->rxBufferIndex] = '\0';

		// move line to transmitter
		strncpy(uart->txBuffer, uart->rxBuffer, MYLIB_UART_RX_BUFF_SIZE);
		strncat(uart->txBuffer, "\r\n\0", 3);
		uart->txBufferSize = uart->rxBufferIndex + 2;

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

void MYLIB_Uart_Console(UART_HandleTypeDef* huart, struct MYLIB_UartInstance* uart)
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
				MYLIB_GpioDigOut_Reset(diodeNumber);
			}
			else
			{
				MYLIB_GpioDigOut_Set(diodeNumber);
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



/* Interrupt Callbacks */
void __weak HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart)
{
	// check UART instance
	if(uart->Instance == USART2)
	{
		// code example
		//MYLIB_UartRxCallback(&huart2, &MYLIB_uart2); <- call a callback function
		//HAL_UART_Receive_IT(&huart2, MYLIB_uart2.rxBuffer + MYLIB_uart2.rxBufferIndex, 1); <- set UART Receiver
	}
}

void __weak HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
	// check UART instance
	if(uart->Instance == USART2)
	{
		// code example

	}
}


