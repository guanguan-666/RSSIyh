/*
 * src/app_lora_gateway.c
 * 网关 LoRa 组网核心逻辑 - AE-HTDMA (动态分配 + 自动回收版)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>     // for sleep, usleep
#include <time.h>       // for timestamp
#include "SX1278.h"     // 底层驱动
#include "app_lora_protocol.h" // 协议定义
#include "db_manager.h" // 数据库管理
#include <sys/time.h> // Linux 系统时间头文件

// 外部引用的 LoRa 句柄
extern SX1278_hw_t SX1278_hw;
extern SX1278_t SX1278;

// 包序列号
static uint8_t gateway_seq_num = 0;

// ==========================================
// [AE-HTDMA] 动态时隙管理核心结构
// ==========================================
#define MAX_SLOTS 10 
#define SLOT_TIMEOUT_SEC 30  // 超时时间：30秒 (约3个周期) 没数据就踢人

// 座位表：索引是 Slot-1 (0~9)，值是 NodeID。0表示空闲。
static uint8_t slot_table[MAX_SLOTS] = {0}; 

// 时间表：记录该时隙最后一次收到数据的时间 (用于踢除僵尸节点)
static time_t slot_last_seen[MAX_SLOTS] = {0};

// 获取当前时间戳 (毫秒级)
long long current_timestamp_ms() {
    struct timeval te; 
    gettimeofday(&te, NULL); // 获取当前时间
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
    return milliseconds;
}

/**
 * @brief [新增] 清理僵尸节点 (超时回收)
 */
void Gateway_Clean_Dead_Slots(void) {
    time_t now = time(NULL);
    
    for(int i=0; i<MAX_SLOTS; i++) {
        // 如果这个座位有人 (node_id != 0)
        if (slot_table[i] != 0) {
            // 检查是否超时
            if ((now - slot_last_seen[i]) > SLOT_TIMEOUT_SEC) {
                printf("[AE-HTDMA] Timeout! Node %d removed from Slot %d (Inactive for %ld s)\n", 
                       slot_table[i], i+1, (now - slot_last_seen[i]));
                
                // === 回收座位 ===
                slot_table[i] = 0; 
                slot_last_seen[i] = 0;
            }
        }
    }
}

/**
 * @brief [新增] 寻找一个空闲时隙 (分配逻辑)
 * @return Slot ID (1~10), 返回 -1 表示满了
 */
int Gateway_Find_Free_Slot(uint8_t node_id) {
    time_t now = time(NULL);

    // 1. 先看看这个节点是不是已经有座位了？(防止重复分配)
    for(int i=0; i<MAX_SLOTS; i++) {
        if (slot_table[i] == node_id) {
            printf("[AE-HTDMA] Node %d already has Slot %d. Keeping it.\n", node_id, i+1);
            slot_last_seen[i] = now; // 重新激活时间戳
            return i + 1; 
        }
    }

    // 2. 找一个空的座位
    for(int i=0; i<MAX_SLOTS; i++) {
        if (slot_table[i] == 0) {
            slot_table[i] = node_id; // 占座
            slot_last_seen[i] = now; // 初始化时间戳
            printf("[AE-HTDMA] Found Empty Slot %d for Node %d.\n", i+1, node_id);
            return i + 1; 
        }
    }
    
    return -1; // 满了
}

/**
 * @brief 发送入网响应
 */
void Gateway_Send_Join_ACK(uint8_t node_id, uint8_t slot_id) {
    uint8_t tx_buf[64];
    memset(tx_buf, 0, sizeof(tx_buf));

    lora_header_t *head = (lora_header_t *)tx_buf;
    payload_join_ack_t *ack = (payload_join_ack_t *)(tx_buf + sizeof(lora_header_t));

    head->net_id = LORA_NET_ID;
    head->type   = TYPE_JOIN_ACK;
    head->src_id = 0x00;
    head->dest_id = 0xFF; // 广播
    head->payload_len = sizeof(payload_join_ack_t);
    
    // 关键点：发送分配好的 永久NodeID 和 临时SlotID
    ack->assigned_node_id = node_id;
    ack->assigned_slot    = slot_id; 

    printf("[Gateway] TX Join ACK -> NodeID: %d, Slot: %d\n", node_id, slot_id);

    uint8_t total_len = sizeof(lora_header_t) + sizeof(payload_join_ack_t);
    SX1278_LoRaEntryTx(&SX1278, total_len, 2000);
    SX1278_LoRaTxPacket(&SX1278, tx_buf, total_len, 2000);
}

/**
 * @brief 发送 Beacon
 */
void Gateway_SendBeacon(void) {
    uint8_t buffer[64];
    memset(buffer, 0, sizeof(buffer));

    lora_header_t *pHeader = (lora_header_t *)buffer;
    pHeader->net_id = LORA_NET_ID;
    pHeader->type   = TYPE_BEACON;
    pHeader->src_id = 0x00;
    pHeader->dest_id = 0xFF;
    pHeader->seq_num = gateway_seq_num++;
    pHeader->payload_len = sizeof(payload_beacon_t);

    payload_beacon_t *pBeacon = (payload_beacon_t *)(buffer + sizeof(lora_header_t));
    pBeacon->timestamp = (uint32_t)time(NULL); 
    pBeacon->cap_slot_len = CAP_DURATION_MS;

    uint8_t total_len = sizeof(lora_header_t) + sizeof(payload_beacon_t);

    printf("[Gateway] TX Beacon | Seq: %d | Time: %d | CAP: %dms\n", 
           pHeader->seq_num, pBeacon->timestamp, pBeacon->cap_slot_len);

    SX1278_LoRaEntryTx(&SX1278, total_len, 2000);
    SX1278_LoRaTxPacket(&SX1278, buffer, total_len, 2000);
}

/**
 * @brief 发送控制回复 (Downlink)
 */
void Gateway_Send_Reply(uint8_t dest_id, uint8_t cmd, float param) {
    uint8_t tx_buf[32];
    lora_header_t *head = (lora_header_t *)tx_buf;
    payload_cmd_down_t *load = (payload_cmd_down_t *)(tx_buf + sizeof(lora_header_t));

    head->net_id = LORA_NET_ID;
    head->type   = TYPE_CMD_DOWN;
    head->src_id = 0x00;
    head->dest_id = dest_id;
    head->payload_len = sizeof(payload_cmd_down_t);

    load->cmd_type = cmd;
    load->param_val = param;

    SX1278_LoRaEntryTx(&SX1278, sizeof(lora_header_t) + sizeof(payload_cmd_down_t), 2000);
    SX1278_LoRaTxPacket(&SX1278, tx_buf, sizeof(lora_header_t) + sizeof(payload_cmd_down_t), 2000);
    
    printf("     [CTRL] Sent Reply to Node %d (Cmd:%d, Val:%.1f)\n", dest_id, cmd, param);
}

/**
 * @brief 监听 CAP (入网请求)
 */
void Gateway_Listen_CAP(int duration_ms) {
    printf("[Gateway] Entering CAP (Listening for %d ms)...\n", duration_ms);
    SX1278_LoRaEntryRx(&SX1278, 64, 2000);

    // 1. 记录开始时间 (绝对时间)
    long long start_time = current_timestamp_ms();

    // 2. 循环条件改为：只要当前时间 - 开始时间 < 目标时长，就一直听
    while ((current_timestamp_ms() - start_time) < duration_ms) {
        
        if(SX1278_hw_GetDIO0(SX1278.hw)) {
            uint8_t rx_buf[64];
            int len = SX1278_LoRaRxPacket(&SX1278);
            if(len > 0) {
                SX1278_read(&SX1278, rx_buf, len);
                lora_header_t *head = (lora_header_t *)rx_buf;
                
                if(head->net_id == LORA_NET_ID) {
                    if(head->type == TYPE_JOIN_REQ) {
                        uint32_t node_uid;
                        // 注意：这里保留了您原函数的逻辑，
                        // 如果您已经更新了协议支持 req_type，记得把这里替换为您最新的解析代码
                        memcpy(&node_uid, rx_buf + sizeof(lora_header_t), 4);
                        printf("\n>>> [JOIN] REQ from UID: 0x%08X | RSSI: %d\n", 
                               node_uid, SX1278_RSSI_LoRa(&SX1278));

                        // 1. 获取/创建永久 ID (查库)
                        int assigned_node_id = db_get_or_create_node_id(node_uid);
                        
                        if (assigned_node_id > 0) {
                            // 2. 动态分配时隙 (查表)
                            // 注意：如果您更新了 Gateway_Find_Free_Slot 支持更多参数，请在这里修改调用
                            int assigned_slot = Gateway_Find_Free_Slot(assigned_node_id); 
                            
                            if (assigned_slot > 0) {
                                // 3. 回复
                                // 注意：如果您更新了 Gateway_Send_Join_ACK 支持更多参数，请在这里修改调用
                                Gateway_Send_Join_ACK(assigned_node_id, assigned_slot);
                            } else {
                                printf("[WARN] Network Full! No slot for Node %d\n", assigned_node_id);
                            }
                        }
                        SX1278_LoRaEntryRx(&SX1278, 64, 2000);
                    }
                    else if (head->type == TYPE_DATA_UP) {
                        // 改为绝对时间后，这里出现的概率会大幅降低
                        printf(" [WARN] Received DATA packet during CAP phase! (Too Early)\n");
                    }
                }
            }
            SX1278_LoRaEntryRx(&SX1278, 64, 2000);
        }
        // 这里的 usleep 只是为了防止 CPU 占用率 100%，不再影响计时精度
        usleep(1000); 
    }
}

/**
 * @brief 监听 TDMA 数据时段
 */
void Gateway_Listen_TDMA(int duration_ms) {
    printf("[Gateway] Entering TDMA (Listening for %d ms)...\n", duration_ms);
    SX1278_LoRaEntryRx(&SX1278, 64, 2000);

    // 1. 记录开始时间 (绝对时间)
    long long start_time = current_timestamp_ms();

    // 2. 循环条件改为绝对时间判断
    while ((current_timestamp_ms() - start_time) < duration_ms) {
        
        if(SX1278_hw_GetDIO0(SX1278.hw)) {
            uint8_t rx_buf[64];
            int len = SX1278_LoRaRxPacket(&SX1278);
            if(len > 0) {
                SX1278_read(&SX1278, rx_buf, len);
                lora_header_t *head = (lora_header_t *)rx_buf;
                
                if(head->net_id == LORA_NET_ID) {
                    if(head->type == TYPE_DATA_UP) {
                        payload_data_up_t *pData = (payload_data_up_t *)(rx_buf + sizeof(lora_header_t));
                        
                        printf("  -> [DATA] Node:%d | Temp:%.2f | Humi:%.2f | PID:%.2f | RSSI:%d\n", 
                               head->src_id, pData->current_temp, pData->current_humidity, 
                               pData->pid_output, SX1278_RSSI_LoRa(&SX1278));

                        // === [核心] 收到数据，给该节点占用的时隙“续命” ===
                        for(int k=0; k<MAX_SLOTS; k++) {
                            if(slot_table[k] == head->src_id) {
                                slot_last_seen[k] = time(NULL); // 更新最后活跃时间
                                break;
                            }
                        }

                        // 存库
                        db_insert_data(head->src_id, pData->current_temp, head->seq_num, 0);

                        // 业务控制逻辑
                        float new_target = 20.0f;
                        if (head->src_id == 2) new_target = 18.0f;
                        Gateway_Send_Reply(head->src_id, 0x01, new_target);
                        
                        SX1278_LoRaEntryRx(&SX1278, 64, 2000);
                    }
                    else {
                        printf("  -> [WARN] Unexpected Frame Type: 0x%02X\n", head->type);
                    }
                }
            }
            else {
                // === [新增] 打印 CRC 错误 ===
                printf("  -> [ERROR] CRC Failed! Likely collision or weak signal.\n");
            }
            SX1278_LoRaEntryRx(&SX1278, 64, 2000);
        }
        usleep(1000); 
    }
}

/**
 * @brief 网关主任务循环
 */
void Gateway_Task_Loop(void) {
    printf("Starting LoRa Gateway AE-HTDMA Loop (Database Optimized)...\n");

    // 用于控制清理频率 (不需要每秒都清理)
    int loop_counter = 0;

    while(1) {
        // 1. 发送 Beacon
        Gateway_SendBeacon();

        // 2. 监听入网 (CAP)
        // 入网操作通常是单个的，可以不加事务，或者单独加
        Gateway_Listen_CAP(CAP_DURATION_MS-200); 
        
        // ==========================================
        // [DB优化] 开启事务：准备批量接收 TDMA 数据
        // ==========================================
        db_begin_transaction();

        // 3. 监听数据 (TDMA)
        // 这里的 db_insert_data 现在会非常快，因为都在内存里
        Gateway_Listen_TDMA(MAX_NODES * TIME_SLOT_MS);

        // ==========================================
        // [DB优化] 提交事务：一次性写入 SD 卡
        // ==========================================
        db_commit_transaction();

        // 4. 清理超时节点 (AE-HTDMA 逻辑)
        Gateway_Clean_Dead_Slots();
        
        // 5. 数据库老化维护 (每 60 轮执行一次，约 10分钟)
        loop_counter++;
        if (loop_counter > 60) {
            printf("[DB] Maintenance: Pruning old data...\n");
            db_prune_old_data(7); // 只保留 7 天数据
            loop_counter = 0;
        }

        // 6. 周期结束
        printf("[Gateway] Superframe End.\n\n");
        sleep(2); 
    }
}


