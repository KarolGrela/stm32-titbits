# stm32-titbits
##  Implementations for different microcontroller peripherals on STM32 NUCLEO-L476RG (Cortex M4)


This repo contains some of the projects created by me in first 2-3 months of learning programming on dev board STM32 NUCLEO with STM32 CubeIDE. This readme file focuses on solutions used in Template projects. <br /> 
As for time of writing this file there are three versions of this template project ([1](../Template), [2](../Template_v2) and [3](../Template_v3)).

## Project Template Premises
Template project has been created in order to kickstart work with new ideas on my dev board STM32 NUCLEO-L476RG. Why wasting time on setting up basic peripherals every time if you can start with these things already configured? <br />
[The most recent template project](../Template_v3) contains configuration and functions for:
* GPIO - Digital inputs - two digital inputs, one with pull-up, and one with pull-down resistor (configured with interrupts)
* GPIO - Digital outputs - six digital outputs (push-pull types)
* UART - Tx/Rx with circular buffer (configured with interrupts)

## Configuration


## GPIO (gpio.c)


## UART (usart.c)


## UART - circular buffer (circular_buf_t.c)


## Timers
