/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L3_Pin GPIO_PIN_0
#define L3_GPIO_Port GPIOC
#define L4_Pin GPIO_PIN_2
#define L4_GPIO_Port GPIOC
#define L5_Pin GPIO_PIN_3
#define L5_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_5
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI9_5_IRQn
#define B0_Pin GPIO_PIN_8
#define B0_GPIO_Port GPIOC
#define B0_EXTI_IRQn EXTI9_5_IRQn
#define L0_Pin GPIO_PIN_10
#define L0_GPIO_Port GPIOC
#define L1_Pin GPIO_PIN_12
#define L1_GPIO_Port GPIOC
#define L2_Pin GPIO_PIN_2
#define L2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
