/* 必须放在第一行 */
#define _DEFAULT_SOURCE 
#define _BSD_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <signal.h> 
#include "linux-hal.h"
#include "SX1278.h"
#include "db_manager.h"         // 引入数据库
#include "app_lora_gateway.h"   // 引入新的组网逻辑头文件

// --- 引脚定义 ---
#define PIN_RESET  119 
#define PIN_DIO0   116 

// --- 全局变量定义 ---
// 注意：必须定义在全局，以便 app_lora_gateway.c 通过 extern 引用
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

volatile int g_running = 1;

// --- 信号处理函数 ---
void sig_handler(int signo) { 
    if (signo == SIGINT) {
        g_running = 0;
        printf("\n[Info] Exiting... Closing DB.\n");
        db_close(); 
        exit(0); // 强制退出
    }
}

int main() {
    setbuf(stdout, NULL);
    printf("Program Starting...\n");
    // 1. 注册信号处理 (Ctrl+C)
    signal(SIGINT, sig_handler);
    
    // 2. 初始化数据库
    // 虽然 Beacon 阶段暂时不用数据库，但先初始化好比较稳健
    if (db_init("sensor_data.db") != 0) {
        printf("[ERR] Database Init Failed!\n");
        return -1;
    }

    printf("\n==================================================\n");
    printf("    LoRa Gateway (H-TDMA Protocol Mode)           \n");
    printf("    Ref: Cold Chain Monitoring System             \n");
    printf("==================================================\n");

    // 3. 配置 LoRa 硬件结构体
    SX1278_hw.spi_fd = -1;
    SX1278_hw.reset_pin = PIN_RESET;
    SX1278_hw.dio0_pin = PIN_DIO0;
    SX1278.hw = &SX1278_hw;

    // 4. 初始化 SX1278 射频芯片
    // 参数说明: 433MHz, 17dBm, SF9, BW125k, CR4/5, CRC开启
    printf("[Init] Configuring SX1278 HW...\n");
    SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, SX1278_LORA_SF_9,
                SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);

    // 5. 检查 SPI 通信是否正常
    // 读取版本寄存器 0x42，SX1278 的固定值为 0x12
    uint8_t version = SX1278_SPIRead(&SX1278, 0x42);
    if (version == 0x12) {
        printf("[Init] SX1278 Connection OK (Ver: 0x%02X)\n", version);
    } else {
        printf("[ERR] SX1278 Connection Failed! Read: 0x%02X, Expected: 0x12\n", version);
        printf("[Tip] Check wiring: SPI, Reset, or Power.\n");
        db_close();
        return -1;
    }

    // 6. 进入 H-TDMA 组网主循环
    // 这个函数定义在 src/app_lora_gateway.c 中，包含了 Beacon 发送逻辑
    Gateway_Task_Loop(); 
    
    // 正常情况下 Gateway_Task_Loop 是死循环，不会执行到这里
    db_close();
    return 0;
}