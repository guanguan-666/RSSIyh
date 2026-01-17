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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rtthread.h>
#include "bsp_lora.h" // 包含 LoRa 相关的定义
#include <stdlib.h>
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

/* USER CODE BEGIN PV */
// =============================================================
// [定义] 全局共享变量 (真正的内存分配发生在这里)
// volatile 关键字必不可少，防止编译器优化掉多线程间的变化
// =============================================================
volatile float g_current_temp = 21.0f; // 初始化当前温度
volatile float g_target_temp  = 20.0f; // 初始化目标温度
rt_thread_t tid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// [新增] 声明外部的 LoRa 业务线程函数 (定义在 app_lora_node.c 中)
extern void lora_thread_entry(void *parameter);
extern void PID_UART3_RxCpltCallback(UART_HandleTypeDef *huart);
extern rt_thread_t tid;
extern 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  统一启动用户线程的函数
 * 放在这里管理，避免 main 函数太乱
 */
void User_App_Start(void)
{
    

/* ================== 创建 LoRa 业务主线程 ================== */
    /* * lora_thread_entry: 在 app_lora_node.c 中定义的新业务逻辑
     * 栈大小: 2048 (2KB) - 协议解析和日志打印需要较多栈空间，改大一点更稳健
     * 优先级: 15 - 提高优先级，确保能及时处理网关的 Beacon
     * 时间片: 10 tick
     */
    tid = rt_thread_create("lora_node",
                           lora_thread_entry, RT_NULL,
                           2048, 15, 10); 
    
    if (tid != RT_NULL) {
        rt_thread_startup(tid);
        rt_kprintf("[Main] LoRa Node App Started.\n");
    } else {
        rt_kprintf("[Main] Error: LoRa Node Thread Create Failed!\n");
    }

    /* 2. 如果以后有PID线程或者其他业务线程，都在这里创建 */
    /* tid = rt_thread_create("pid_task", pid_thread_entry, ...);
    if (tid != RT_NULL) rt_thread_startup(tid);
    */
}

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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    rt_kprintf("\n");
    rt_kprintf("==========================================\n");
    rt_kprintf("   Reefer Node System (RT-Thread)         \n");
    rt_kprintf("==========================================\n");

  /* [修改说明] 
   * 移除了这里的 LoRa_Init() 调用。
   * 原因：lora_thread_entry 线程内部会调用 LoRa_Init()。
   * 如果这里也调用，会导致互斥量重复创建，属于资源浪费。
   */
  
  /* 2. 启动用户线程 */
  User_App_Start();
   rt_kprintf("Rx Size: %d (Expect 10)\n", sizeof(MatlabRxFrame_t));
   rt_kprintf("Tx Size: %d (Expect 6)\n", sizeof(MatlabTxFrame_t));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/* 让主线程仅仅作为心跳闪烁，频率低一点，证明系统活着 */
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5); 
      rt_thread_mdelay(1000); // 每秒闪一次，不干扰其他线程
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
/* 引入外部回调函数 */
extern void PID_UART3_RxCpltCallback(UART_HandleTypeDef *huart);

/* HAL库串口接收完成回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 如果是串口3，交给 PID 模块处理 */
    if (huart->Instance == USART3)
    {
        PID_UART3_RxCpltCallback(huart);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

