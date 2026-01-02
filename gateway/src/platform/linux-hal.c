#include "SX1278_hw.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>
#include "linux-hal.h"

/* ======================================================================
 * 内部 GPIO 辅助函数 (基于 sysfs)
 * ====================================================================== */

void gpio_export(int pin) {
    char buffer[64];
    int len;
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        // 如果无法打开，通常是因为已经导出过了，或者权限不足
        // perror("Warning: Failed to open export for writing"); 
        return; 
    }
    len = snprintf(buffer, sizeof(buffer), "%d", pin);
    if (write(fd, buffer, len) < 0) {
        // 忽略 "Device or resource busy" 错误 (说明已导出)
        if (errno != EBUSY) perror("Failed to export GPIO");
    }
    close(fd);
}

void gpio_dir(int pin, const char *dir) {
    char path[64];
    int fd;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open gpio%d direction\n", pin);
        return;
    }
    if (write(fd, dir, strlen(dir)) < 0) perror("Failed to set direction");
    close(fd);
}

void gpio_set(int pin, int value) {
    char path[64];
    int fd;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) return;
    if (write(fd, value ? "1" : "0", 1) < 0) perror("Failed to set value");
    close(fd);
}

int gpio_get(int pin) {
    char path[64];
    char value_str[3];
    int fd;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    if (read(fd, value_str, 3) < 0) {
        close(fd);
        return 0;
    }
    close(fd);
    return (value_str[0] == '1');
}

/* ======================================================================
 * HAL 接口实现
 * ====================================================================== */

/**
 * @brief 初始化硬件 (打开 SPI，配置 GPIO)
 * @param hw 包含 SPI 文件路径和 GPIO 编号的结构体
 */
void SX1278_hw_init(SX1278_hw_t *hw) {
    // 1. 初始化 GPIO (Reset 和 DIO0)
    // 注意：hw 结构体在使用前必须在 main.c 中赋值好 pin 脚编号
    if (hw->reset_pin > 0) {
        gpio_export(hw->reset_pin);
        gpio_dir(hw->reset_pin, "out");
        gpio_set(hw->reset_pin, 1); // 默认拉高
    }

    if (hw->dio0_pin > 0) {
        gpio_export(hw->dio0_pin);
        gpio_dir(hw->dio0_pin, "in");
    }

    // 2. 打开 SPI 设备
    // 如果还没打开，就尝试打开默认路径，或者你应该在 main 中赋值 spi_fd
    if (hw->spi_fd < 0) {
        hw->spi_fd = open("/dev/spidev0.0", O_RDWR); // 默认使用 spidev0.0
        if (hw->spi_fd < 0) {
            perror("Error: Failed to open /dev/spidev0.0");
            exit(1);
        }
    }

    // 3. 配置 SPI 参数 (Mode 0, 8 bit, 10MHz)
    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = 10000000; // SX1278 支持最高 10MHz

    if (ioctl(hw->spi_fd, SPI_IOC_WR_MODE, &mode) < 0) perror("SPI: set mode");
    if (ioctl(hw->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) perror("SPI: set bits");
    if (ioctl(hw->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) perror("SPI: set speed");
    
    printf("[HAL] SX1278 Hardware Initialized.\n");
}

/**
 * @brief 复位 LoRa 模块
 */
void SX1278_hw_Reset(SX1278_hw_t *hw) {
    if (hw->reset_pin <= 0) return;

    // 拉低 1ms 复位
    gpio_set(hw->reset_pin, 0);
    SX1278_hw_DelayMs(1);
    
    // 拉高并等待稳定
    gpio_set(hw->reset_pin, 1);
    SX1278_hw_DelayMs(6); // SX1278 需要 5ms 启动时间
}

/**
 * @brief 设置 NSS 片选 (Linux spidev 自动控制，这里留空即可)
 */
void SX1278_hw_SetNSS(SX1278_hw_t *hw, int value) {
    // Linux spidev 驱动会在每次 transfer 时自动拉低 CS
    // 除非你自己用 GPIO 模拟 CS，否则这里不需要操作
}

/**
 * @brief SPI 核心传输函数 (全双工)
 */
void SX1278_hw_SPITransfer(SX1278_hw_t *hw, uint8_t *tx_buf, uint8_t *rx_buf, int len) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = len,
        .speed_hz = 10000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
        .cs_change = 0,
    };

    int ret = ioctl(hw->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("SPI: Transfer failed");
    }
}

/* 兼容原驱动的旧接口：发送单个命令 */
/* 注意：为了性能和 CS 时序，建议在 SX1278.c 中改用 SPITransfer 批量发送 */
void SX1278_hw_SPICommand(SX1278_hw_t *hw, uint8_t cmd) {
    uint8_t rx;
    SX1278_hw_SPITransfer(hw, &cmd, &rx, 1);
}

/* 兼容原驱动的旧接口：读取单个字节 */
uint8_t SX1278_hw_SPIReadByte(SX1278_hw_t *hw) {
    uint8_t tx = 0xFF; // 发送空闲字节
    uint8_t rx;
    SX1278_hw_SPITransfer(hw, &tx, &rx, 1);
    return rx;
}

/**
 * @brief 延时函数 (毫秒)
 */
void SX1278_hw_DelayMs(uint32_t msec) {
    usleep(msec * 1000);
}

/**
 * @brief 读取中断引脚 (DIO0)
 */
int SX1278_hw_GetDIO0(SX1278_hw_t *hw) {
    if (hw->dio0_pin <= 0) return 0;
    return gpio_get(hw->dio0_pin);
}
