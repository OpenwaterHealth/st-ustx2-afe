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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "command_queue.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern UART_HandleTypeDef huart5;
extern I2C_HandleTypeDef hi2c1;
extern CommandQueue commandQueue;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define nINTERRUPT_Pin GPIO_PIN_0
#define nINTERRUPT_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_0
#define FAN_GPIO_Port GPIOB
#define PWR_GOOD_Pin GPIO_PIN_1
#define PWR_GOOD_GPIO_Port GPIOB
#define nHB_LED_Pin GPIO_PIN_12
#define nHB_LED_GPIO_Port GPIOB
#define CS_TXA_Pin GPIO_PIN_11
#define CS_TXA_GPIO_Port GPIOA
#define CS_TXB_Pin GPIO_PIN_12
#define CS_TXB_GPIO_Port GPIOA
#define READY_Pin GPIO_PIN_5
#define READY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// #define RUN_TESTS
#define COMMAND_QUEUE_SIZE 10
// #define DEBUG_COMMS

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
