/*
 * app_lora_protocol.h
 * LoRa H-TDMA 组网协议定义
 *
 * 适用平台:
 * 1. Node: STM32F103 (RT-Thread)
 * 2. Gateway: Linux (i.MX6ULL/RaspberryPi)
 *
 * 说明: 必须确保两端使用完全相同的本文件，并且开启单字节对齐
 */

#ifndef _APP_LORA_PROTOCOL_H_
#define _APP_LORA_PROTOCOL_H_

#include <stdint.h>

// ==========================================
// 1. 网络参数配置 (两端必须一致)
// ==========================================
#define LORA_NET_ID         0xAB      // 网络识别码，过滤非本网络数据
#define LORA_FREQ           433000000 // 工作频率 433MHz

// TDMA 时隙配置
#define TIME_SLOT_MS        1000       // 每个节点的数据传输时隙 (毫秒)
#define MAX_NODES           10        // 系统支持的最大节点数
#define CAP_DURATION_MS     2000      // 竞争接入段 (CAP) 长度，用于新节点入网

// ==========================================
// 2. 帧类型定义 (Frame Types)
// ==========================================
#define TYPE_BEACON         0x01      // [网关 -> 广播] 信标：同步时间，开启超帧
#define TYPE_JOIN_REQ       0x02      // [节点 -> 网关] 入网请求
#define TYPE_JOIN_ACK       0x03      // [网关 -> 节点] 入网允许 (分配时隙)
#define TYPE_DATA_UP        0x04      // [节点 -> 网关] 传感器数据上报
#define TYPE_CMD_DOWN       0x05      // [网关 -> 节点] 控制指令下发

// ==========================================
// 3. 数据结构定义 (强制 1 字节对齐)
// ==========================================
#pragma pack(push, 1)

/**
 * @brief 通用帧头 (所有数据包都包含)
 */
typedef struct {
    uint8_t net_id;      // 网络ID (固定为 LORA_NET_ID)
    uint8_t type;        // 帧类型 (见上文定义)
    uint8_t src_id;      // 源地址 (网关=0, 节点=1~254)
    uint8_t dest_id;     // 目的地址 (广播=0xFF)
    uint8_t seq_num;     // 包序号 (用于检测丢包)
    uint8_t payload_len; // 后续载荷的长度
} lora_header_t;

/**
 * @brief 载荷 1: 信标 (Beacon)
 * 网关发送，用于广播系统时间和超帧结构
 */
typedef struct {
    uint32_t timestamp;      // 网关当前的 Unix 时间戳 (或系统运行tick)
    uint16_t cap_slot_len;   // 告知节点 CAP 阶段有多长
} payload_beacon_t;

/**
 * @brief 载荷 2: 入网响应 (Join ACK)
 * 网关回复给新节点，告知其 ID 和时隙
 */
typedef struct {
    uint8_t assigned_node_id; // 分配给你的新 ID
    uint8_t assigned_slot;    // 分配给你的时隙号 (1, 2, ...)
} payload_join_ack_t;

/**
 * @brief 载荷 3: 节点数据上报 (Data Up)
 * 节点在自己的时隙发送温湿度和电机状态
 */
typedef struct {
    float   current_temp;     // 当前温度 (DS18B20/DHT11)
    float   current_humidity; // 当前湿度
    float   target_temp;      // 当前设定的目标温度
    float   pid_output;       // PID 计算输出值
    uint8_t motor_status;     // 电机运行状态 (0:停止, 1:运行, 2:故障)
} payload_data_up_t;

/**
 * @brief 载荷 4: 控制指令 (Command Down)
 * 网关下发给节点，修改参数
 */
typedef struct {
    uint8_t cmd_type;         // 指令类型 (0x01:改目标温度, 0x02:强制启停)
    float   param_val;        // 参数值 (例如新的目标温度 25.5)
} payload_cmd_down_t;

#pragma pack(pop) // 恢复默认对齐

#endif // _APP_LORA_PROTOCOL_H_
