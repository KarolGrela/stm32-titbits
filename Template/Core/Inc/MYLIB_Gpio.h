/*
 * MYLIB_Gpio.h
 *
 *  Created on: Jan 11, 2023
 *      Author: kgrela1
 */

#ifndef INC_MYLIB_GPIO_H_
#define INC_MYLIB_GPIO_H_

#include "main.h"

/// LEDs
#define NO_OF_DIODES 6
struct MYLIB_GpioInstance
{
	GPIO_TypeDef* port;
	uint16_t pin;
};


/// Buttons
//volatile unsigned char diodeMark = 0;
//volatile unsigned char diodeMarkChange = 0;

void MYLIB_GpioDigOut_ResetAll(void);

void MYLIB_GpioDigOut_SetAll(void);

void MYLIB_GpioDigOut_Reset(unsigned char index);

void MYLIB_GpioDigOut_Set(unsigned char index);

unsigned char MYLIB_Gpio_GetDiodes(void);

volatile unsigned char MYLIB_Gpio_GetDiodeMark(void);
void MYLIB_Gpio_SetDiodeMark(unsigned char inputVal);

volatile unsigned char MYLIB_Gpio_GetDiodeMarkChange(void);
void MYLIB_Gpio_SetDiodeMarkChange(unsigned char inputVal);


#endif /* INC_MYLIB_GPIO_H_ */
