/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1278.h"
#include "SX1278_hw.h"
#include "fuzzyPID.h"
#include "bsp_lora.h"
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
#define LORA_DIO0_Pin GPIO_PIN_2
#define LORA_DIO0_GPIO_Port GPIOA
#define LORA_DIO0_EXTI_IRQn EXTI2_IRQn
#define LORA_RST_Pin GPIO_PIN_3
#define LORA_RST_GPIO_Port GPIOA
#define MOTOR_IN1_Pin GPIO_PIN_6
#define MOTOR_IN1_GPIO_Port GPIOC
#define MOTOR_IN2_Pin GPIO_PIN_7
#define MOTOR_IN2_GPIO_Port GPIOC
#define MOTOR_IN3_Pin GPIO_PIN_8
#define MOTOR_IN3_GPIO_Port GPIOC
#define MOTOR_IN4_Pin GPIO_PIN_9
#define MOTOR_IN4_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
