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

// 简单的协议帧定义 (用于和 MATLAB 通信)
#pragma pack(push, 1) // 保存当前对齐方式，并设置为 1 字节对齐

typedef struct {
    uint8_t header; 
    float target;   
    float current;  
    uint8_t tail;   
} MatlabRxFrame_t; // 此时 sizeof = 1+4+4+1 = 10

// 接收：[新增] 参数更新数据包
typedef struct {
    uint8_t header;     // 0xB6 (新的帧头，用来区分)
    float   new_kp;     // 新的 Kp
    float   new_ki;     // 新的 Ki
    float   new_kd;     // 新的 Kd
    uint8_t tail;       // 0x5A
} MatlabParamFrame_t;   // 大小 = 1+4+4+4+1 = 14 字节

typedef struct {
    uint8_t header; 
    float output;   
    uint8_t tail;   
} MatlabTxFrame_t; // 此时 sizeof = 1+4+1 = 6

#pragma pack(pop) // 恢复之前的对齐方式，不影响后面的代码

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
