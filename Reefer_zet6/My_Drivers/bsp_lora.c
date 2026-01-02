#include "SX1278.h"
#include "main.h"   // 获取引脚宏 (LORA_NSS_Pin 等)
#include "rtthread.h"
#include <string.h> // 用于 memcpy, strlen
#include "bsp_lora.h"
/* 引入 SPI 句柄 (CubeMX 生成的在 main.c 里，这里需要 extern) */
extern SPI_HandleTypeDef hspi1; 

/* 定义驱动结构体变量 */
SX1278_hw_t lora_hw;
SX1278_t lora;

/* 如果包含了 rtthread.h 还是报错，可以在这里手动补一个定义（作为兜底） */
#ifndef RT_WAIT_FOREVER
#define RT_WAIT_FOREVER  ((-1))
#endif

/* 定义一个互斥量，保护LoRa资源，防止发送和接收冲突 */
static rt_mutex_t lora_lock = RT_NULL;

/* LoRa 初始化函数 */
int LoRa_Init(void)
{
    rt_kprintf("Starting LoRa Init...\n");

    /* 1. 配置硬件抽象层 (映射引脚) */
    /* 注意：这些宏 (LORA_DIO0_Pin 等) 必须在 main.h 里定义过 */
    /* 如果报错，请检查 CubeMX 里引脚 User Label 是否填对 */
    lora_hw.dio0.port  = LORA_DIO0_GPIO_Port;
    lora_hw.dio0.pin   = LORA_DIO0_Pin;
    
    lora_hw.nss.port   = LORA_NSS_GPIO_Port;
    lora_hw.nss.pin    = LORA_NSS_Pin;
    
    lora_hw.reset.port = LORA_RST_GPIO_Port;
    lora_hw.reset.pin  = LORA_RST_Pin;
    
    lora_hw.spi        = &hspi1; // 你的 SPI 句柄

    /* 2. 将硬件层挂载到逻辑层 */
    lora.hw = &lora_hw;

    /* 3. 初始化模块 */
    /* 参数依据你的论文设计：433MHz, 17dBm, SF9, BW125kHz */
    /*[cite: 1]: 论文中提到 SF9, BW125k, 433MHz */
    SX1278_init(&lora, 433000000, SX1278_POWER_17DBM, SX1278_LORA_SF_9,
                SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 64);

    /* 4. 验证是否成功 (读取版本号) */
    // 0x42 是 Version 寄存器地址，SX1278 的默认值通常是 0x12
    uint8_t version = SX1278_SPIRead(&lora, 0x42);
    
    rt_kprintf("LoRa Check: RegVersion = 0x%02X (Expect 0x12)\n", version);

    if (version == 0x12) {
            /* 创建互斥量 */
            lora_lock = rt_mutex_create("lora_mtx", RT_IPC_FLAG_FIFO);
            
            /* 初始化成功后，进入接收模式准备接收网关指令 */
            //SX1278_LoRaEntryRx(&lora, 64, 2000); // 64是最大接收长度，2000是超时(ms)
            
            return RT_EOK;
        } else {
            rt_kprintf("LoRa Error: Hardware connection failed!\n");
            return -RT_ERROR;
        }
}

/* * 发送数据函数 
 * data: 发送内容的指针
 * len: 发送长度
 */
void LoRa_Send_Data(uint8_t *data, uint8_t len)
{
    if (lora_lock != RT_NULL) rt_mutex_take(lora_lock, RT_WAIT_FOREVER);

    rt_kprintf("LoRa Sending: %s\n", data);

    /* 1. 发送数据 (超时时间 2000ms) */
    /* transmit函数内部会把模式切为TX，发送完后切为Standby */
    int result = SX1278_transmit(&lora, data, len, 2000);
    
    if (result == 1) {
        rt_kprintf("LoRa TX Done.\n");
    } else {
        rt_kprintf("LoRa TX Failed or Timeout.\n");
    }

    /* 2. 发送完成后，必须切回 RX 模式，否则收不到网关的回复 */
    SX1278_LoRaEntryRx(&lora, 64, 2000); 

    if (lora_lock != RT_NULL) rt_mutex_release(lora_lock);
}

void lora_rx_thread_entry(void *parameter)
{
    uint8_t rx_buf[64];
    uint8_t rx_len;

    while (1)
    {
        /* 1. 申请锁：检查是否有数据 */
        if (rt_mutex_take(lora_lock, 200) == RT_EOK) // 等待200ms拿不到锁就放弃本次检查
        {
            /* 检查 DIO0 是否置位 (代表收到包) */
            if (SX1278_available(&lora))
            {
                /* 读取数据 */
                rx_len = SX1278_read(&lora, rx_buf, sizeof(rx_buf)-1);
                rx_buf[rx_len] = '\0'; // 补0做字符串结尾
                
                rt_kprintf("RX: Recv %s, RSSI: %d\n", rx_buf, SX1278_RSSI_LoRa(&lora));
                
                /* 处理你的业务逻辑，比如解析网关指令 */
                // Process_Gateway_Command(rx_buf);
            }
            
            /* 释放锁，给发送函数机会 */
            rt_mutex_release(lora_lock);
        }

        /* 2. 挂起一段时间：不要死循环狂跑，给 CPU 喘息机会 */
        /* LoRa 速率很慢，20ms-50ms 轮询一次完全足够 */
        rt_thread_mdelay(20); 
    }
}


