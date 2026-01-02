#ifndef __SX1278_HW_H__
#define __SX1278_HW_H__

#include <stdint.h>

/* * Linux 平台硬件结构体定义
 * 在 Linux 下，我们通过文件描述符操作 SPI，通过 sysfs 编号操作 GPIO
 */

typedef struct {
    int spi_fd;       // SPI 设备文件描述符 (对应 /dev/spidev0.0)
    int reset_pin;    // 复位引脚的 GPIO 编号 (例如 115)
    int dio0_pin;     // 中断引脚的 GPIO 编号 (例如 116)
    int nss_pin;      // 片选引脚 (Linux spidev 自动控制，这里仅作预留或手动控制用)
} SX1278_hw_t;

/* -------------------------------------------------------------------------
 * 函数声明 (这些函数在 SX1278_hw_Linux.c 中实现)
 * ------------------------------------------------------------------------- */

/**
 * @brief 初始化硬件层 (打开 SPI 设备，导出 GPIO)
 * @param hw 指向硬件结构体的指针
 */
void SX1278_hw_init(SX1278_hw_t *hw);

/**
 * @brief 复位 LoRa 模块
 * @param hw 指向硬件结构体的指针
 */
void SX1278_hw_Reset(SX1278_hw_t *hw);

/**
 * @brief 设置 NSS 片选信号 (Linux spidev 通常自动控制，此函数可选)
 * @param hw 指向硬件结构体的指针
 * @param value 0:拉低(选中), 1:拉高(释放)
 */
void SX1278_hw_SetNSS(SX1278_hw_t *hw, int value);

/**
 * @brief SPI 核心传输函数 (替换原来的 SPICommand/ReadByte)
 * @param hw 指向硬件结构体的指针
 * @param tx_buf 发送数据缓冲区
 * @param rx_buf 接收数据缓冲区 (不需要接收可传 NULL)
 * @param length 数据长度
 */
void SX1278_hw_SPITransfer(SX1278_hw_t *hw, uint8_t *tx_buf, uint8_t *rx_buf, int len);

/* 兼容原驱动的旧接口包装 (为了不改动 SX1278.c 太多) */
void SX1278_hw_SPICommand(SX1278_hw_t *hw, uint8_t cmd);
uint8_t SX1278_hw_SPIReadByte(SX1278_hw_t *hw);

/**
 * @brief 毫秒级延时
 * @param msec 毫秒数
 */
void SX1278_hw_DelayMs(uint32_t msec);

/**
 * @brief 读取 DIO0 引脚状态 (用于查询中断)
 * @param hw 指向硬件结构体的指针
 * @return 1: 高电平, 0: 低电平
 */
int SX1278_hw_GetDIO0(SX1278_hw_t *hw);

#endif // __SX1278_HW_H__
