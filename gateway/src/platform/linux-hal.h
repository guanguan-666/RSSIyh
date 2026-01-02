#ifndef __LINUX_HAL_H__
#define __LINUX_HAL_H__

#include <stdint.h>

/* ======================================================================
 * 1. i.MX6ULL GPIO 计算宏 (神器！)
 * ======================================================================
 * 用法: IMX_GPIO_NR(4, 26) 自动算出 122
 * 原理: (Bank - 1) * 32 + Pin
 */
#define IMX_GPIO_NR(bank, pin)  ((((bank) - 1) * 32) + (pin))

/* ======================================================================
 * 2. 通用 GPIO 工具函数声明
 * (需要在 linux-hal.c 中去掉这些函数的 static 关键字才能被外部调用)
 * ====================================================================== */

/**
 * @brief 导出 GPIO 引脚 (相当于 echo 122 > /sys/class/gpio/export)
 * @param pin GPIO 编号
 */
void gpio_export(int pin);

/**
 * @brief 设置 GPIO 方向
 * @param pin GPIO 编号
 * @param dir "in" 或 "out"
 */
void gpio_dir(int pin, const char *dir);

/**
 * @brief 设置 GPIO 输出电平
 * @param pin GPIO 编号
 * @param value 0 或 1
 */
void gpio_set(int pin, int value);

/**
 * @brief 读取 GPIO 输入电平
 * @param pin GPIO 编号
 * @return 0 或 1
 */
int gpio_get(int pin);

#endif // __LINUX_HAL_H__
