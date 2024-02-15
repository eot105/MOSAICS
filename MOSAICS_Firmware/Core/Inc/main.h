/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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
#define MUX_SEL0_Pin GPIO_PIN_0
#define MUX_SEL0_GPIO_Port GPIOA
#define MUX_SEL1_Pin GPIO_PIN_1
#define MUX_SEL1_GPIO_Port GPIOA
#define MUX_SEL2_Pin GPIO_PIN_2
#define MUX_SEL2_GPIO_Port GPIOA
#define MUX_SEL3_Pin GPIO_PIN_3
#define MUX_SEL3_GPIO_Port GPIOA
#define CS_LD_Pin GPIO_PIN_4
#define CS_LD_GPIO_Port GPIOA
#define ADC_CNV_Pin GPIO_PIN_12
#define ADC_CNV_GPIO_Port GPIOB
#define ADC_BUSY_Pin GPIO_PIN_15
#define ADC_BUSY_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

struct MOSAICS{
	double  range;
	double value;
	uint8_t  shift;
	uint8_t  dac;
	uint8_t mux_code;
};

#define MAX_CMD_SIZE 10



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
