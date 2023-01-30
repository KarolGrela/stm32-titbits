/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "usart.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define NO_OF_DIODES 6

struct gpio_instance_t
{
	GPIO_TypeDef* port;
	uint16_t pin;
};

extern struct gpio_instance_t diodesTable[NO_OF_DIODES];

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/*** UNIVERSAL FUNCTIONS ***/
void gpio_DigOutResetAll(struct gpio_instance_t* outputTable, const unsigned char size);

void gpio_DigOutSetAll(struct gpio_instance_t* outputTable, const unsigned char size);

void gpio_DigOutReset(unsigned char index, struct gpio_instance_t* outputTable);

void gpio_DigOutSet(unsigned char index, struct gpio_instance_t* outputTable);

void gpio_DigOutToggle(unsigned char index, struct gpio_instance_t* outputTable);

/*** SITUATIONAL FUNCTIONS ***/

unsigned char gpio_GetDiodes(struct gpio_instance_t* outputTable, const unsigned char size);

volatile unsigned char gpio_GetDiodeMark(void);
void gpio_SetDiodeMark(unsigned char inputVal);

volatile unsigned char gpio_GetDiodeMarkChange(void);
void gpio_SetDiodeMarkChange(unsigned char inputVal);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

