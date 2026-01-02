#ifndef _APP_LORA_PROTOCOL_H_
#define _APP_LORA_PROTOCOL_H_

#include <stdint.h>

// --- 网络配置 ---
#define LORA_NET_ID     0xAB  // 网络标识符
#define MAX_NODES       10    // 最大节点数
#define CAP_DURATION_MS 2000  // 入网竞争期时长 (ms)
#define TIME_SLOT_MS    1000  // 每个时隙时长 (ms)

// --- 帧类型定义 ---
#define TYPE_BEACON     0x01  // 网关信标
#define TYPE_JOIN_REQ   0x02  // 节点入网请求
#define TYPE_JOIN_ACK   0x03  // 入网确认
#define TYPE_DATA_UP    0x04  // 数据上报
#define TYPE_CMD_DOWN   0x05  // 控制指令

// --- [新增] 服务请求优先级 ---
#define REQ_TYPE_NORMAL  0  // 普通/短包 (占用 1 个时隙)
#define REQ_TYPE_URGENT  1  // 紧急/长包 (占用 2 个连续时隙)

#pragma pack(1) // 强制 1 字节对齐 (非常重要! Linux与STM32通信必须加)

// 1. 通用帧头 (所有包都有)
typedef struct {
    uint8_t net_id;       // 网络ID (防串扰)
    uint8_t type;         // 帧类型
    uint8_t src_id;       // 源地址 (0=网关)
    uint8_t dest_id;      // 目的地址 (0xFF=广播)
    uint8_t seq_num;      // 序列号
    uint8_t payload_len;  // 载荷长度
} lora_header_t;

// 2. Beacon 载荷 (网关广播)
typedef struct {
    uint32_t timestamp;     // Unix 时间戳
    uint16_t cap_slot_len;  // CAP 长度
} payload_beacon_t;

// 3. [修改] Join Request 载荷 (节点申请)
// 之前这里可能很简单，现在增加了 req_type 和 current_err
typedef struct {
    uint32_t device_uid;   // 芯片唯一ID (UID)
    uint8_t  req_type;     // [新增] 请求类型 (0=普通, 1=紧急)
    uint8_t  current_err;  // [新增] 当前误差值 (辅助判断)
} payload_join_req_t;

// 4. [修改] Join ACK 载荷 (网关回复)
// 增加了 assigned_duration
typedef struct {
    uint8_t assigned_node_id; // 分配的短地址
    uint8_t assigned_slot;    // 分配的起始时隙
    uint8_t assigned_duration;// [新增] 分配的时隙长度 (1或2)
} payload_join_ack_t;

// 5. Data Uplink 载荷 (节点上报)
typedef struct {
    float    current_temp;     // 当前温度
    float    target_temp;      // [加回] 目标温度 (用于监测 PID 设定值)
    float    current_humidity; // 当前湿度
    float    pid_output;       // PID 输出值
    uint8_t  motor_status;     // 电机状态
} payload_data_up_t;

// 6. Command Downlink 载荷 (网关控制)
typedef struct {
    uint8_t cmd_type;   // 指令类型
    float   param_val;  // 参数值
} payload_cmd_down_t;

#pragma pack() // 恢复默认对齐

#endif
