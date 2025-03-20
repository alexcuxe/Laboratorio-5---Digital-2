/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_TypeDef *p1_ports[] = {P1L1_GPIO_Port, P1L2_GPIO_Port, P1L3_GPIO_Port, P1L4_GPIO_Port}; 				// pin port
uint16_t p1_pins[] = {P1L1_Pin, P1L2_Pin, P1L3_Pin, P1L4_Pin}; 												// pin number

GPIO_TypeDef *p2_ports[] = {P2L1_GPIO_Port, P2L2_GPIO_Port, P2L3_GPIO_Port, P2L4_GPIO_Port}; 				// pin port
uint16_t p2_pins[] = {P2L1_Pin, P2L2_Pin, P2L3_Pin, P2L4_Pin}; 												// pin number

uint8_t cont1 = 0;
uint8_t cont2 = 0;
uint8_t start = 0;
uint8_t player1 = 0;
uint8_t player2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void showDisplay(uint8_t number);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(start == 1){
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);									// game has started
		  showDisplay(5); HAL_Delay(1000);													// it displays number 5 on display and waits one second
		  showDisplay(4); HAL_Delay(1000);													// it displays number 4 on display and waits one second
		  showDisplay(3); HAL_Delay(1000);													// it displays number 3 on display and waits one second
		  showDisplay(2); HAL_Delay(1000);													// it displays number 2 on display and waits one second
		  showDisplay(1); HAL_Delay(1000);													// it displays number 1 on display and waits one second
		  showDisplay(0); start = 2;														// it displays number 0 on display and start the game
	  }

	  if(player1 == 1){																		// player1 wins
		  showDisplay(1);																	// it displays number 1 on display
		  HAL_Delay(500); player1 = 2;														// delay for showing 4th LED, it sets value to 2 for doing it once
		  for(uint8_t i = 0; i < 4; i++) HAL_GPIO_WritePin(p1_ports[i], p1_pins[i], SET);	// sets all of its LEDs
		  for(uint8_t j = 0; j < 4; j++) HAL_GPIO_WritePin(p2_ports[j], p2_pins[j], RESET);	// resets all of player2's LEDs
	  }

	  if(player2 == 1){																		// player2 wins
		  showDisplay(2);																	// it displays number 2 on display
		  HAL_Delay(500); player2 = 2;														// delay for showing 4th LED, it sets value to 2 for doing it once
		  for(uint8_t j = 0; j < 4; j++) HAL_GPIO_WritePin(p2_ports[j], p2_pins[j], SET);	// sets all of its LEDs
		  for(uint8_t i = 0; i < 4; i++) HAL_GPIO_WritePin(p1_ports[i], p1_pins[i], RESET);	// resets all of player1's LEDs
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, P2L4_Pin|P2L3_Pin|P2L2_Pin|LD2_Pin
                          |P1L4_Pin|P1L3_Pin|A_Pin|B_Pin
                          |C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, P2L1_Pin|G_Pin|D_Pin|F_Pin
                          |E_Pin|P1L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(P1L1_GPIO_Port, P1L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : P2L4_Pin P2L3_Pin P2L2_Pin LD2_Pin
                           P1L4_Pin P1L3_Pin A_Pin B_Pin
                           C_Pin */
  GPIO_InitStruct.Pin = P2L4_Pin|P2L3_Pin|P2L2_Pin|LD2_Pin
                          |P1L4_Pin|P1L3_Pin|A_Pin|B_Pin
                          |C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : P2L1_Pin G_Pin D_Pin F_Pin
                           E_Pin P1L2_Pin */
  GPIO_InitStruct.Pin = P2L1_Pin|G_Pin|D_Pin|F_Pin
                          |E_Pin|P1L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : P1L1_Pin */
  GPIO_InitStruct.Pin = P1L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(P1L1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){												// interrupt for buttons
	if (GPIO_Pin == BTN1_Pin && player1 == 0 && player2 == 0 && start == 2){				// BTN player1, if nobody wins, yet..., allow players to play when countdown ends
		for(uint8_t i = 0; i < 4; i++) HAL_GPIO_WritePin(p1_ports[i], p1_pins[i], RESET);	// turn off LEDs
		cont1++;																			// increase player1's counter
		if (cont1 >= 5) cont1 = 4;															// it keeps counter if it's bigger than 4
		if (cont1 == 4 && player1 == 0) player1 = 1;										// player1 wins, activate all of its LEDs once
		if (cont1 > 0) HAL_GPIO_WritePin(p1_ports[cont1-1], p1_pins[cont1-1], SET);			// showing the respective pin
	}

	if (GPIO_Pin == BTN2_Pin && player1 == 0 && player2 == 0 && start == 2){				// BTN player2, if nobody wins, yet..., allow players to play when countdown ends
		for(uint8_t j = 0; j < 4; j++) HAL_GPIO_WritePin(p2_ports[j], p2_pins[j], RESET);	// turn off LEDs
		cont2++;																			// increase player2's counter
		if (cont2 >= 5) cont2 = 4;															// it keeps counter if it's bigger than 4
		if (cont2 == 4 && player2 == 0)	player2 = 1;										// player2 wins, activate all of its LEDs once
		if (cont2 > 0) HAL_GPIO_WritePin(p2_ports[cont2-1], p2_pins[cont2-1], SET);			// showing the respective pin
	}

	if (GPIO_Pin == B1_Pin && start == 0) start = 1;										// Integrated BTN, start the game's countdown if game's never been started before
}


void showDisplay(uint8_t number) {	// It updates every display's pin, it compares every pattern bit to set or reset the display pin
	uint8_t displayN[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};							// Numbers for display, hex, 0-F
	GPIO_TypeDef *display_ports[] = {A_GPIO_Port, B_GPIO_Port, C_GPIO_Port, D_GPIO_Port, E_GPIO_Port, F_GPIO_Port, G_GPIO_Port}; 	// pin port, add dot if necessary
	uint16_t display_pins[] = {A_Pin, B_Pin, C_Pin, D_Pin, E_Pin, F_Pin, G_Pin}; 													// pin number, add dot if necessary
	for (int k = 0; k < 7; k++) HAL_GPIO_WritePin(display_ports[k], display_pins[k], (displayN[number]>> k) & 1); 					// Not using dot, if using dot, k<8
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
