/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
//The idea is to OR the WRITE_X command and the DAC_X command together to make one 8 bit packet, then shift it 8 bits to make the first
//16 bits of the data frame.

uint8_t WRITE_CODE_N              = 0b00000000;
uint8_t WRITE_CODE_ALL            = 0b10000000;
uint8_t WRITE_SPAN_N              = 0b01100000;
uint8_t WRITE_SPAN_ALL            = 0b11100000;
uint8_t WRITE_UPDATE_N            = 0b00010000;
uint8_t WRITE_UPDATE_ALL          = 0b10010000;
uint8_t WRITE_CODE_N_UPDATE_N     = 0b00110000;
uint8_t WRITE_CODE_N_UPDATE_ALL   = 0b00100000;
uint8_t WRITE_CODE_ALL_UPDATE_ALL = 0b10100000;
uint8_t WRITE_POWER_N             = 0b01000000;
uint8_t WRITE_POWER_CHIP          = 0b01010000;
uint8_t WRITE_MONITOR_MUX         = 0b10110000;
uint8_t WRITE_TOGGLE_SELECT       = 0b11000000;
uint8_t WRITE_GOLBAL_TOGGLE       = 0b11010000;
uint8_t WRITE_CONFIG_CMD          = 0b01110000;
uint8_t WRITE_NO_OP               = 0b11110000;

uint8_t DAC_0 = 0b00000000;
uint8_t DAC_1 = 0b00000001;
uint8_t DAC_2 = 0b00000010;
uint8_t DAC_3 = 0b00000011;
uint8_t DAC_4 = 0b00000100;
uint8_t DAC_NO_OP = 0b00001111;

uint16_t SPAN_HIZ     = 0b0000000000000000;
uint16_t SPAN_003_125 = 0b0000000000000001;
uint16_t SPAN_006_250 = 0b0000000000000010;
uint16_t SPAN_012_500 = 0b0000000000000011;
uint16_t SPAN_025_000 = 0b0000000000000100;
uint16_t SPAN_050_000 = 0b0000000000000101;
uint16_t SPAN_100_000 = 0b0000000000000110;
uint16_t SPAN_200_000 = 0b0000000000000111;
uint16_t SPAN_300_000 = 0b0000000000001111;
uint16_t SPAN_V_MINUS = 0b0000000000001000;

uint16_t MUX_HIZ       = 0b0000000000000000;
uint16_t MUX_OUT0_CURR = 0b0000000000000001;
uint16_t MUX_OUT1_CURR = 0b0000000000000010;
uint16_t MUX_OUT2_CURR = 0b0000000000000011;
uint16_t MUX_OUT3_CURR = 0b0000000000000100;
uint16_t MUX_OUT4_CURR = 0b0000000000000101;
uint16_t MUX_VCC       = 0b0000000000000110;
uint16_t MUX_VREF      = 0b0000000000001000;
uint16_t MUX_VREFLO    = 0b0000000000001001;
uint16_t MUX_TEMP      = 0b0000000000001010;
uint16_t MUX_VDD0      = 0b0000000000010000;
uint16_t MUX_VDD1      = 0b0000000000010001;
uint16_t MUX_VDD2      = 0b0000000000010010;
uint16_t MUX_VDD3      = 0b0000000000010011;
uint16_t MUX_VDD4      = 0b0000000000010100;
uint16_t MUX_VPLUS     = 0b0000000000010101;
uint16_t MUX_VMINUS    = 0b0000000000010110;
uint16_t MUX_GND       = 0b0000000000010111;
uint16_t MUX_OUT0_VOLT = 0b0000000000011000;
uint16_t MUX_OUT1_VOLT = 0b0000000000011001;
uint16_t MUX_OUT2_VOLT = 0b0000000000011010;
uint16_t MUX_OUT3_VOLT = 0b0000000000011011;
uint16_t MUX_OUT4_VOLT = 0b0000000000011100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint16_t span_code   = 0x0000 | (WRITE_SPAN_ALL | DAC_NO_OP);
	  uint16_t update_code = 0x0000 | (WRITE_CODE_ALL_UPDATE_ALL | DAC_NO_OP);
	  uint16_t no_op_code = 0x0000 | (WRITE_NO_OP | DAC_4);
	  uint16_t mux_code = 0x0000 | (WRITE_MONITOR_MUX | DAC_0);
	  uint16_t on_code_1 = 0x7FFF;
	  uint16_t on_code_2 = 0x6969;
	  uint16_t off_code = 0x0000;

	  uint16_t set_shift_reg_span[20];
	  uint16_t set_shift_reg_value[20];
	  uint16_t set_mux_code[20];
	  for (uint8_t i = 0; i < 20; i = i + 2){
		  set_shift_reg_span[i] = span_code;
		  set_shift_reg_span[i+1] = SPAN_V_MINUS;
	  }

	  for (uint8_t i = 0; i < 20; i = i + 2){
		  set_shift_reg_value[i] = no_op_code;
		  set_shift_reg_value[i+1] = off_code;
	  }

	  for (uint8_t i = 0; i < 20; i = i + 2){
		  set_mux_code[i] = mux_code;
		  set_mux_code[i+1] = MUX_OUT4_CURR;
	  }



	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  for (uint8_t i = 0; i < 10; i++){
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&span_code, 1, 100);
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&SPAN_200_000, 1, 100);
	  }
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  for (uint8_t i = 0; i < 10; i++){
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&update_code, 1, 100);
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&on_code_1, 1, 100);
	  }
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1, (uint8_t*)set_shift_reg_span, 20, 100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1, (uint8_t*)set_shift_reg_value, 20, 100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1, (uint8_t*)set_mux_code, 20, 100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
