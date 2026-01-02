#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

/* * 注意：根据你之前的截图，设备节点可能是 /dev/spidev1.0 
 * 如果运行报错，请先在板子上 ls /dev/spidev* 确认名字
 */
static const char *device = "/dev/spidev0.0"; 

int main(int argc, char *argv[])
{
    int fd;
    int ret = 0;
    
    // 发送缓冲区：0x42 是读寄存器地址，0x00 是填充字节(用于推挤出接收数据)
    // SX1278 SPI协议：读操作时地址位最高位为0。RegVersion地址是0x42。
    uint8_t tx[] = {0x42, 0x00}; 
    
    // 接收缓冲区
    uint8_t rx[2] = {0, };

    // SPI 配置参数
    static uint8_t mode = 0;          // Mode 0 (CPOL=0, CPHA=0) 是 SX1278 的标准模式
    static uint8_t bits = 8;          // 8位数据宽
    static uint32_t speed = 1000000;  // 1MHz 速度 (保守测试，最稳)
    static uint16_t delay = 0;

    // 1. 打开设备节点
    // ------------------------------------------------
    if (argc > 1) device = argv[1]; // 允许通过命令行指定设备，例如 ./lora_check /dev/spidev0.0
    
    fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Error: Can't open SPI device");
        printf("Hint: Check if %s exists using 'ls /dev/spidev*'\n", device);
        return -1;
    }

    // 2. 配置 SPI 参数 (Mode, Bits, Speed)
    // ------------------------------------------------
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) { perror("Can't set spi mode"); goto error; }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) { perror("Can't set bits per word"); goto error; }

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) { perror("Can't set max speed hz"); goto error; }

    // 3. 准备传输结构体
    // ------------------------------------------------
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    printf("--- LoRa (SX1278) Hardware Test ---\n");
    printf("Target Device: %s\n", device);
    printf("Sending: Read RegVersion (0x42)\n");

    // 4. 执行全双工传输
    // ------------------------------------------------
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("Can't send spi message");
        goto error;
    }

    // 5. 分析结果
    // ------------------------------------------------
    // rx[0] 是发送地址时 MISO 返回的状态(通常无效或高阻)，rx[1] 才是真正读到的寄存器值
    printf("Response: [0x%02X] [0x%02X]\n", rx[0], rx[1]);

    if (rx[1] == 0x12) {
        printf("\n✅ SUCCESS! Chip found (Version 0x12).\n");
        printf("Hardware connections (MOSI, MISO, SCK, CS) are correct.\n");
    } else {
        printf("\n❌ FAILED. Expected 0x12, got 0x%02X.\n", rx[1]);
        printf("Troubleshooting:\n");
        printf("1. Check wiring: MOSI/MISO swapped? GND connected?\n");
        printf("2. Check CS pin: Is it GPIO4_26 as defined in DTS?\n");
        printf("3. Check Power: Is VCC 3.3V?\n");
    }

    close(fd);
    return 0;

error:
    close(fd);
    return -1;
}
