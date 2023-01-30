/*
 * MYLIB_Gpio.c
 *
 *  Created on: Jan 11, 2023
 *      Author: kgrela1
 */

#include "MYLIB_Gpio.h"

/// Buttons auxiliary variables
static volatile unsigned char diodeMark = 0;
static volatile unsigned char diodeMarkChange = 0;

struct MYLIB_GpioInstance diodesTable[NO_OF_DIODES] = {
		{.port = L0_GPIO_Port, .pin = L0_Pin},
		{.port = L1_GPIO_Port, .pin = L1_Pin},
		{.port = L2_GPIO_Port, .pin = L2_Pin},
		{.port = L3_GPIO_Port, .pin = L3_Pin},
		{.port = L4_GPIO_Port, .pin = L4_Pin},
		{.port = L5_GPIO_Port, .pin = L5_Pin},
};


void MYLIB_GpioDigOut_ResetAll(void)
{
	for(int i = 0; i < NO_OF_DIODES; i++)
	{
		HAL_GPIO_WritePin(diodesTable[i].port, diodesTable[i].pin, GPIO_PIN_RESET);
	}
}

void MYLIB_GpioDigOut_SetAll(void)
{
	for(int i = 0; i < NO_OF_DIODES; i++)
	{
		HAL_GPIO_WritePin(diodesTable[i].port, diodesTable[i].pin, GPIO_PIN_SET);
	}
}

unsigned char MYLIB_Gpio_GetDiodes(void)
{
	unsigned char retVal = 0;
	for(int i = 0; i < NO_OF_DIODES; i++)
	{
		if(HAL_GPIO_ReadPin(diodesTable[i].port, diodesTable[i].pin) == GPIO_PIN_SET)
		{
			retVal |= (1 << i);
		}
	}
	return retVal;
}

void MYLIB_GpioDigOut_Reset(unsigned char index)
{
	struct MYLIB_GpioInstance instance = diodesTable[index];
	HAL_GPIO_WritePin(instance.port, instance.pin, GPIO_PIN_RESET);
}

void MYLIB_GpioDigOut_Set(unsigned char index)
{
	struct MYLIB_GpioInstance instance = diodesTable[index];
	HAL_GPIO_WritePin(instance.port, instance.pin, GPIO_PIN_SET);
}

volatile unsigned char MYLIB_Gpio_GetDiodeMark(void)
{
	return diodeMark;
}

void MYLIB_Gpio_SetDiodeMark(unsigned char inputVal)
{
	diodeMark = inputVal;
}

volatile unsigned char MYLIB_Gpio_GetDiodeMarkChange(void)
{
	return diodeMarkChange;
}

void MYLIB_Gpio_SetDiodeMarkChange(unsigned char inputVal)
{
	diodeMarkChange = inputVal;
}

