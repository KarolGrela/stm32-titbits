/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct pin_t
{
	GPIO_TypeDef* port;
	uint16_t pin;
};

struct button_t
{
	GPIO_TypeDef* port;
	uint16_t pin;
	unsigned char active_state;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NO_OF_DIODES 6
#define NO_OF_BUTTONS 2
#define UART_BUFFER_LEN 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int __io_putchar(int ch);
unsigned char uart_reveive(char* received_string);

void led_resetAllLeds(void);
void led_setLed(void);
int led_countUp(void);
int led_countDown(void);

static const struct pin_t leds[NO_OF_DIODES] = {
	{.port = LED_0_GPIO_Port, .pin = LED_0_Pin},
	{.port = LED_1_GPIO_Port, .pin = LED_1_Pin},
	{.port = LED_2_GPIO_Port, .pin = LED_2_Pin},
	{.port = LED_3_GPIO_Port, .pin = LED_3_Pin},
	{.port = LED_4_GPIO_Port, .pin = LED_4_Pin},
	{.port = LED_5_GPIO_Port, .pin = LED_5_Pin}
};

static const struct button_t buttons[NO_OF_BUTTONS] = {
		{.port = B_1_GPIO_Port, .pin = B_1_Pin, .active_state = 0},
		{.port = B_2_GPIO_Port, .pin = B_2_Pin, .active_state = 1}
};

static int led_counter = 0;
static unsigned char led_direction = 0;
static char uart_buffer[UART_BUFFER_LEN + 1];
static uint8_t uart_line_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  led_resetAllLeds();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Hello World!\n");
  led_setLed();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uart_reveive(NULL);

	  /*Configure GPIO pin : B_2_Pin *
	  GPIO_InitStruct.Pin = B_2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(B_2_GPIO_Port, &GPIO_InitStruct);*/

	  //button = HAL_GPIO_ReadPin(buttons[0].port, buttons[0].pin);

	  if(HAL_GPIO_ReadPin(buttons[0].port, buttons[0].pin) == buttons[0].active_state)
	  {
		  HAL_Delay(20);
		  while(HAL_GPIO_ReadPin(buttons[0].port, buttons[0].pin) == buttons[0].active_state)
		  {
			  // do nothing
		  }
		  HAL_Delay(20);
		  led_direction ^= (1 >> 0);
	  }

	  if(HAL_GPIO_ReadPin(buttons[1].port, buttons[1].pin) == buttons[1].active_state)
	  {
		  if( (led_direction & (1 >> 0)) == 0)
		  {
			  led_countDown();
		  }
		  else
		  {
			  led_countUp();
		  }
		  printf("Diode %d is turned on!\n", led_counter);

		  led_setLed();

		  while(HAL_GPIO_ReadPin(buttons[1].port, buttons[1].pin) == buttons[1].active_state)
		  {
		  	  // do nothing
		  }
	  }



	  //buttons[1] = HAL_GPIO_ReadPin(B_2_GPIO_Port, B_2_Pin);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  fflush(stdout);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_2_Pin|LED_3_Pin|LED_4_Pin|LED_5_Pin
                          |LED_0_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin LED_4_Pin LED_5_Pin
                           LED_0_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin|LED_4_Pin|LED_5_Pin
                          |LED_0_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B_2_Pin */
  GPIO_InitStruct.Pin = B_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B_1_Pin */
  GPIO_InitStruct.Pin = B_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B_1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

unsigned char uart_reveive(char* received_string)
{
	uint8_t value;
	unsigned char reciver_status = HAL_UART_Receive(&huart2, &value, 1, 100);
	if(reciver_status == HAL_OK)
	{
	  if(value == '\r' || value == '\n')	// end of line received
	  {
		  if(uart_line_len > 0)
		  {
			  uart_buffer[uart_line_len] = '\0';
			  if(strncmp(uart_buffer, "CNT_UP", strlen("CNT_UP")) == 0)
			  {
				  led_countUp();
				  led_setLed();
			  }
			  else if(strncmp(uart_buffer, "CNT_DWN", strlen("CNT_DWN")) == 0)
			  {
				  led_countDown();
				  led_setLed();
			  }
			  else
			  {
				  printf("from uC: %s\n", uart_buffer);
			  }
			  uart_line_len = 0;
		  }
	  }
	  else
	  {
		  if(uart_line_len < UART_BUFFER_LEN)
		  {
			  uart_buffer[uart_line_len] = (char)value;
			  uart_line_len++;
		  }
		  else
		  {
			  printf("buffer overflowed!\n");
			  uart_buffer[uart_line_len] = (char)value;
			  uart_line_len = 1;
		  }
	  }
	}
	else
	{

	}

	return reciver_status;
}

int __io_putchar(int ch)
{
	if((char)ch == '\n')
	{
		uint8_t local = '\r';
		HAL_UART_Transmit(&huart2, &local, 1, HAL_MAX_DELAY);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void led_resetAllLeds(void)
{
	for(int i = 0; i < NO_OF_DIODES; i++)
	{
		HAL_GPIO_WritePin(leds[i].port, leds[i].pin, GPIO_PIN_RESET);
	}
}

void led_setLed(void)
{
	struct pin_t pin = leds[led_counter];
	led_resetAllLeds();
	HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_SET);
}

int led_countUp(void)
{
	if(led_counter < NO_OF_DIODES-1)
	{
		led_counter++;
	}
	else
	{
		led_counter = 0;
	}

	return led_counter;
}

int led_countDown(void)
{
	if(led_counter > 0)
	{
		led_counter--;
	}
	else
	{
		led_counter = NO_OF_DIODES - 1;
	}

	return led_counter;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
