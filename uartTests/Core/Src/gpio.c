/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/// Buttons auxiliary variables
static volatile unsigned char diodeMark = 0;
static volatile unsigned char diodeMarkChange = 0;

// Outputs table
struct gpio_instance_t diodesTable[NO_OF_DIODES] = {
		{.port = L0_GPIO_Port, .pin = L0_Pin},
		{.port = L1_GPIO_Port, .pin = L1_Pin},
		{.port = L2_GPIO_Port, .pin = L2_Pin},
		{.port = L3_GPIO_Port, .pin = L3_Pin},
		{.port = L4_GPIO_Port, .pin = L4_Pin},
		{.port = L5_GPIO_Port, .pin = L5_Pin},
};
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L3_Pin|L4_Pin|L5_Pin|L0_Pin
                          |L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin */
  GPIO_InitStruct.Pin = L3_Pin|L4_Pin|L5_Pin|L0_Pin
                          |L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/*** UNIVERSAL FUNCTIONS ***/

void gpio_DigOutResetAll(struct gpio_instance_t* outputTable, const unsigned char size)
{
	for(int i = 0; i < size; i++)
	{
		HAL_GPIO_WritePin(outputTable[i].port, outputTable[i].pin, GPIO_PIN_RESET);
	}
}

void gpio_DigOutSetAll(struct gpio_instance_t* outputTable, const unsigned char size)
{
	for(int i = 0; i < size; i++)
	{
		HAL_GPIO_WritePin(outputTable[i].port, outputTable[i].pin, GPIO_PIN_SET);
	}
}

void gpio_DigOutReset(unsigned char index, struct gpio_instance_t* outputTable)
{
	HAL_GPIO_WritePin(outputTable[index].port, outputTable[index].pin, GPIO_PIN_RESET);
}

void gpio_DigOutSet(unsigned char index, struct gpio_instance_t* outputTable)
{
	HAL_GPIO_WritePin(outputTable[index].port, outputTable[index].pin, GPIO_PIN_SET);
}

void gpio_DigOutToggle(unsigned char index, struct gpio_instance_t* outputTable)
{
	HAL_GPIO_TogglePin(outputTable[index].port, outputTable[index].pin);
}

/*** SITUATIONAL FUNCTIONS ***/

unsigned char gpio_GetDiodes(struct gpio_instance_t* outputTable, const unsigned char size)
{
	unsigned char retVal = 0;
	for(int i = 0; i < size; i++)
	{
		if(HAL_GPIO_ReadPin(outputTable[i].port, outputTable[i].pin) == GPIO_PIN_SET)
		{
			retVal |= (1 << i);
		}
	}
	return retVal;
}

/// diodeMark getter/setter
volatile unsigned char gpio_GetDiodeMark(void)
{
	return diodeMark;
}
void gpio_SetDiodeMark(unsigned char inputVal)
{
	diodeMark = inputVal;
}

/// diodeMarkChange getter/setter
volatile unsigned char gpio_GetDiodeMarkChange(void)
{
	return diodeMarkChange;
}
void gpio_SetDiodeMarkChange(unsigned char inputVal)
{
	diodeMarkChange = inputVal;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	unsigned char diodeM = gpio_GetDiodeMark();
  if (GPIO_Pin == B1_Pin)
  {
	  // up
	  if(diodeM < NO_OF_DIODES-1)
	  {
		  gpio_SetDiodeMark(diodeM + 1);
	  }
	  else
	  {
		  gpio_SetDiodeMark(0);
	  }

	  gpio_SetDiodeMarkChange(1);
  }
  else if (GPIO_Pin == B0_Pin)
  {
	  // down
	  if(diodeM < 1)
	  {
		  gpio_SetDiodeMark(NO_OF_DIODES-1);
	  }
	  else
	  {
		  gpio_SetDiodeMark(diodeM - 1);
	  }

	  gpio_SetDiodeMarkChange(1);

	  uart_StartTransmission(&huart2, &uart_instance2);
	  uart_StartTransmission(&huart3, &uart_instance3);
  }

}
/* USER CODE END 2 */
