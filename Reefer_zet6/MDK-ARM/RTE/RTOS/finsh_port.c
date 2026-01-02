/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rthw.h>
#include <rtconfig.h>
#include "usart.h"

#ifndef RT_USING_FINSH
#error Please uncomment the line <#include "finsh_config.h"> in the rtconfig.h 
#endif

#ifdef RT_USING_FINSH

RT_WEAK char rt_hw_console_getchar(void)
{
    int ch = -1;

    /* 1. 检查是否接收到数据 (RXNE标志) */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        /* 直接读取数据寄存器 (DR)，不经过 HAL 库的繁琐检查 */
        ch = huart1.Instance->DR & 0xFF;
    }
    else
    {
        /* 2. 关键：清除溢出错误 (Overrun Error) 
         * 如果数据发太快没来得及读，ORE会置位，导致后续无法接收。
         * 清除方法：读取 SR 然后读取 DR (HAL宏会自动处理)
         */
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
        {
            __HAL_UART_CLEAR_OREFLAG(&huart1);
        }
    }

    return ch;
}

#endif /* RT_USING_FINSH */

