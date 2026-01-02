#ifndef __BSP_LORA_H__
#define __BSP_LORA_H__

#include <stdint.h>
#include "SX1278.h"
extern SX1278_t lora;
// 建议在 bsp_lora.h 中定义
#pragma pack(1) // 1字节对齐，防止两端编译器由于对齐方式不同导致数据错位
typedef struct {
    uint8_t  node_id;      // 节点ID
    uint8_t  msg_type;     // 消息类型 (0x01: 温湿度数据, 0x02: 报警, 0x03: 心跳)
    uint16_t seq_num;      // 包序号 (用于检测丢包)
    float    temp;         // 温度
    float    humid;        // 湿度
    uint8_t  checksum;     // 简单的校验和
} LoRa_Packet_t;
#pragma pack()


/* 初始化函数 - 在系统启动时调用 */
int LoRa_Init(void);

/* 发送数据函数 - 供其他线程调用发送消息 */
void LoRa_Send_Data(uint8_t *data, uint8_t len);

/* 接收线程入口函数 - 供 main.c 创建线程使用 */
void lora_rx_thread_entry(void *parameter);

uint32_t Get_Random_Delay(uint32_t min_ms, uint32_t max_ms);

uint8_t Is_Channel_Busy(void);

int LoRa_Get_Current_RSSI(void);

#endif /* __BSP_LORA_H__ */
