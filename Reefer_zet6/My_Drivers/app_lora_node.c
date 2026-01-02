/*
 * My_Drivers/app_lora_node.c
 * 阶段 2: 节点发送入网请求
 */

#include <rtthread.h>
#include <main.h>
#include "bsp_lora.h"
#include "app_lora_protocol.h" 
#include <stdlib.h> // for rand()



// ============================================
// [新增] 引用外部变量 (Extern Declarations)
// 告诉编译器这些变量在 app_pid_test.c 里
// ============================================
extern volatile float target_temperature;
extern volatile float current_temperature; // 如果你的发送函数里用了它
extern volatile float pid_out;             // 如果你的发送函数里用了它
extern volatile uint8_t motor_state;


extern uint8_t SX1278_SPIRead(SX1278_t * module, uint8_t addr);
// ============================================


#define MAX_JOIN_RETRIES  3    // CAP 期间最多尝试 3 次
#define MIN_BACKOFF_MS    100   // 最小等待 50ms
#define MAX_BACKOFF_MS    800  // 最大等待 300ms (不要超过 CAP 剩余时间)

extern SX1278_t lora;

// --- 宏定义 ---
// 网关周期大约是 11.8秒 (Beacon + CAP + TDMA)
// 我们设置超时为 16000ms，超过这个时间没收到 Beacon 就算丢包
#define BEACON_TIMEOUT_MS   19000  
#define MAX_MISSED_LIMIT    3      // 连续丢失 3 次就重置状态

// 定义一个阈值，温差超过 3度 算紧急
#define URGENT_THRESHOLD  3.0f
// 在文件顶部定义
#define GUARD_TIME_MS  30  // 保护间隔：让出前 30ms，防止撞上状态切换
// --- 状态机定义 (升级版) ---
typedef enum {
    NODE_STATE_SCANNING = 0, // 正在找网关 (静默)
    NODE_STATE_JOINING,      // 正在申请入网
    NODE_STATE_RUNNING,      // 正常工作中
    NODE_STATE_OFFLINE       // [新增] 掉线/异常状态
} node_state_t;

static node_state_t current_state = NODE_STATE_SCANNING;
static uint8_t      rx_buffer[128];
static uint8_t      my_node_id = 0; // 0 表示未分配 (原有的)
static uint8_t      my_slot_id = 0; // [新增] 用于存储网关分配的时隙


// --- 全局变量 ---
static uint32_t last_beacon_tick = 0; // 上一次收到 Beacon 的系统时间
static uint8_t  missed_beacon_cnt = 0; // 连续丢失计数器

// --- 获取 STM32 唯一 ID 的辅助函数 ---
uint32_t Get_MCU_UID(void) {
    // STM32F103 的 UID 地址通常在 0x1FFFF7E8
    return *(uint32_t*)(0x1FFFF7E8); 
}

// 引用全局变量
extern volatile float g_current_temp;
extern volatile float g_target_temp;

/**
 * @brief 发送入网请求
 */
void Send_Join_Request(void) {
    uint8_t tx_buf[64];
    uint8_t retry_count = 0;
    uint8_t send_success = 0;

    // 准备数据包 (代码不变)
    lora_header_t *head = (lora_header_t *)tx_buf;
    payload_join_req_t *req = (payload_join_req_t *)(tx_buf + sizeof(lora_header_t));

    // 计算优先级 (代码不变)
    float error = g_current_temp - g_target_temp;
    if(error < 0) error = -error;
    uint8_t request_type = (error > 3.0f) ? REQ_TYPE_URGENT : REQ_TYPE_NORMAL;
    
    // 填充头部
    head->net_id = LORA_NET_ID;
    head->type   = TYPE_JOIN_REQ;
    head->src_id = my_node_id; 
    head->dest_id = 0x00;      
    head->payload_len = sizeof(payload_join_req_t);
    

    // 填充 Payload
    // === [修改] 读取 STM32 硬件唯一 ID ===
    // HAL_GetUIDw0/w1/w2 分别读取 96位 ID 的前/中/后 32位。
    // 我们通过异或(XOR)运算把 96位 压缩成 32位，虽然理论有碰撞概率，但在局域网内几乎不可能碰撞。
    uint32_t hard_uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
    
    
    // 注意：这里建议用真实的 UID，否则随机种子可能一样
    req->device_uid = hard_uid; // 这里依然建议换成 HAL_GetUIDw0()
    req->req_type   = request_type;
    req->current_err = (uint8_t)error;
    
    rt_kprintf("My Hardware UID is: 0x%08X\n", hard_uid); // 打印出来看看

    uint8_t total_len = sizeof(lora_header_t) + sizeof(payload_join_req_t);
    
    // === [新增] RF 冷却/稳定时间 ===
    // 刚收到 Beacon，让射频电路冷静一下，防止读到 Beacon 的残影
    rt_thread_mdelay(20);

    // === CSMA/CA 核心循环 ===
    while (retry_count < MAX_JOIN_RETRIES) {
        
        // 1. LBT (Listen Before Talk)
        if (Is_Channel_Busy()) {
            rt_kprintf("[CSMA] Channel Busy (RSSI:%d). Backing off...\n", SX1278_RSSI_LoRa(&lora));
            // 信道忙，随机退避
            rt_thread_mdelay(Get_Random_Delay(MIN_BACKOFF_MS, MAX_BACKOFF_MS));
            retry_count++;
            continue; // 重试
        }

        // 2. 发送
        rt_kprintf("[CSMA] Channel Free. Sending JOIN (Try %d)...\n", retry_count + 1);
        SX1278_LoRaEntryTx(&lora, total_len, 2000);
        SX1278_LoRaTxPacket(&lora, tx_buf, total_len, 2000);
        
        // 发完立即切回接收，等待 ACK (这里需要配合主循环的接收逻辑)
        // 在这里我们认为只要发出去了就算这一步成功，等待 ACK 是主循环的事
        SX1278_LoRaEntryRx(&lora, 64, 2000);
        send_success = 1;
        break; 
    }

    if (!send_success) {
        rt_kprintf("[CSMA] Failed to send JOIN after %d retries. Wait next Beacon.\n", MAX_JOIN_RETRIES);
    }
}

/* 添加模拟数据发送函数 */
void Send_Sensor_Data(void) {
    uint8_t tx_buf[64];
    lora_header_t *head = (lora_header_t *)tx_buf;
    payload_data_up_t *load = (payload_data_up_t *)(tx_buf + sizeof(lora_header_t));
    
    // 1. 填充帧头
    head->net_id = LORA_NET_ID;
    head->type   = TYPE_DATA_UP;
    head->src_id = my_node_id;    // 填入我的 ID
    head->dest_id = 0x00;         // 发给网关
    head->seq_num++;              // 序号递增 (利用静态变量或全局变量)
    head->payload_len = sizeof(payload_data_up_t);

    // 2. 填充载荷 (这里先造一些假数据测试)
    // 实际项目中，这里会读取 DS18B20 和 PID 结果
    static float fake_temp = 20.0f;
    fake_temp += 0.5f;            // 每次温度加 0.5，方便观察变化
    if(fake_temp > 40.0) fake_temp = 20.0;

    load->current_temp = fake_temp;
    load->current_humidity = 60.0f;
    load->target_temp = 25.0f;
    load->pid_output = 100.0f;    // 假设电机全速
    load->motor_status = 1;       // 运行中

    // 3. 发送
    uint8_t total_len = sizeof(lora_header_t) + sizeof(payload_data_up_t);
    
    rt_kprintf("[NODE] Sending Data in Slot %d... Temp: %d.%d C\n", 
               my_slot_id, 
               (int)fake_temp, 
               (int)((fake_temp - (int)fake_temp) * 10)); // 打印一位小数
    
    SX1278_LoRaEntryTx(&lora, total_len, 2000);
    SX1278_LoRaTxPacket(&lora, tx_buf, total_len, 2000);
    rt_kprintf("[NODE] Data Sent. Waiting for ACK/CMD...\n");
    

// === [新增] 发送后的“接收窗口” (RX Window) ===
    // 切换到接收模式，等待 500ms
    SX1278_LoRaEntryRx(&lora, 64, 2000);
    
    // 我们在这里死等一小会儿 (或者用循环轮询)
    // 因为这是在 TDMA 自己的时隙内，占用 CPU 也是安全的
    uint32_t rx_start = rt_tick_get();
    while (rt_tick_get() - rx_start < 500) { // 等待 500ms
        
        if (SX1278_available(&lora)) {
            uint8_t ack_buf[64];
            int len = SX1278_read(&lora, ack_buf, sizeof(ack_buf));
            if (len > 0) {
                lora_header_t *ack_head = (lora_header_t *)ack_buf;
                
                // 检查是不是给我的控制指令
                if (ack_head->net_id == LORA_NET_ID && 
                    ack_head->type == TYPE_CMD_DOWN &&
                    ack_head->dest_id == my_node_id) 
                {
                    payload_cmd_down_t *cmd = (payload_cmd_down_t *)(ack_buf + sizeof(lora_header_t));
                    
                    rt_kprintf(">>> [CTRL] Recv Command! Type:%d, Val:%.1f\n", cmd->cmd_type, cmd->param_val);
                    
                    // --- 执行控制逻辑 ---
                    if (cmd->cmd_type == 0x01) {
                        target_temperature = cmd->param_val;
                        rt_kprintf("    -> Update Target Temp to %.1f\n", target_temperature);
                    }
                    
                    // 收到指令，闪烁 LED 提示
                    // rt_pin_write(LED_PIN, 0); rt_thread_mdelay(50); rt_pin_write(LED_PIN, 1);
                    break; // 收到就退出等待
                }
            }
            // 收到错误的包，继续接收
            SX1278_LoRaEntryRx(&lora, 64, 2000);
        }
        rt_thread_mdelay(5);
    }
    
    rt_kprintf("[NODE] RX Window Closed.\n");               
    // 发完切回接收，准备接下一个 Beacon
    SX1278_LoRaEntryRx(&lora, 64, 2000);
}




void lora_thread_entry(void *parameter)
{
    int len;
    
    if (LoRa_Init() != RT_EOK) return;

    rt_kprintf("[LoRa] Node Started. State: SCANNING\n");
    SX1278_LoRaEntryRx(&lora, 64, 2000);
    
    // 初始化时间戳
    last_beacon_tick = rt_tick_get();

    while (1)
    {
        if (SX1278_available(&lora))
        {
            len = SX1278_read(&lora, rx_buffer, sizeof(rx_buffer));
            if (len > 0)
            {
                lora_header_t *pHead = (lora_header_t *)rx_buffer;

                if (pHead->net_id == LORA_NET_ID && pHead->type == TYPE_BEACON)
                {
                    payload_beacon_t *pBeacon = (payload_beacon_t *)(rx_buffer + sizeof(lora_header_t));
                    
                    rt_kprintf("[SYNC] Beacon Recv. Time:%u\n", pBeacon->timestamp);
                    
                    // 1. 喂狗：收到信标，重置计数器和时间戳
                    last_beacon_tick = rt_tick_get();
                    missed_beacon_cnt = 0;
                    
                    // 如果我是 SCANNING (刚上电) -> 切到 JOINING
                    if (current_state == NODE_STATE_SCANNING) {
                        rt_kprintf("[Status] Found Network! Switching to JOINING...\n");
                        current_state = NODE_STATE_JOINING;
                    }
                    // 如果之前掉线了，现在找回了，尝试重新申请或恢复
                    else if (current_state == NODE_STATE_OFFLINE) {
                        rt_kprintf("[Status] Network Recovered! Re-Joining...\n");
                        my_node_id = 0; // 清除旧 ID，重新申请
                        current_state = NODE_STATE_JOINING;
                    }
                    
                    // 如果我是 JOINING (正在申请) -> 发送请求
                    if (current_state == NODE_STATE_JOINING && my_node_id == 0) {
                        // === [新增] 初始随机抖动 ===
                        // 防止所有节点都在 Beacon 结束后的第 1ms 同时发起 LBT
                        // 随机延时 0 ~ 100ms
                        uint32_t start_delay = rand() % 100; 
                        rt_thread_mdelay(start_delay);
                        Send_Join_Request();
                    }
                    
                    // === [修改] RUNNING 状态下的 TDMA 逻辑 ===
                    if (current_state == NODE_STATE_RUNNING) 
                    {
                        // 1. 计算基础等待时间：CAP时长
                        uint32_t wait_ms = CAP_DURATION_MS; 
                        
                        // 2. 加上前面的时隙时间 (Slot 1 不加, Slot 2 加 1000ms...)
                        if (my_slot_id > 0) {
                            wait_ms += (my_slot_id - 1) * TIME_SLOT_MS;
                        }

                        // 3. === [关键修改] 加上 30ms 保护间隔 ===
                        // 替换掉了原先的 +500，确保在时隙刚开始稍后一点发送
                        wait_ms += GUARD_TIME_MS; 

                        rt_kprintf("[SYNC] Waiting %d ms for Slot %d (Guard: %dms)...\n", 
                                   wait_ms, my_slot_id, GUARD_TIME_MS);
                        
                        // 4. 线程精准延时
                        rt_thread_mdelay(wait_ms);
                        
                        // 5. 醒来，发送数据
                        Send_Sensor_Data();
                    }
                }
                // 2. 处理 Join ACK (身份证)
                else if (pHead->net_id == LORA_NET_ID && pHead->type == TYPE_JOIN_ACK)
                {
                    // 只有在申请状态下才处理 ACK
                    if (current_state == NODE_STATE_JOINING) 
                    {
                        payload_join_ack_t *pAck = (payload_join_ack_t *)(rx_buffer + sizeof(lora_header_t));
                        
                        my_node_id = pAck->assigned_node_id;
                        my_slot_id = pAck->assigned_slot;
                        
                        current_state = NODE_STATE_RUNNING; // 切换状态
                        
                        rt_kprintf("\n");
                        rt_kprintf("######################################\n");
                        rt_kprintf("# JOIN SUCCESS! I am Node %d (Slot %d) #\n", my_node_id, my_slot_id);
                        rt_kprintf("######################################\n");
                    }
                }
            }
            // 恢复接收
            SX1278_LoRaEntryRx(&lora, 64, 2000);
        }
        // ============================================================
        //  第二部分：异常检测看门狗 (Watchdog) - 保持不变
        // ============================================================
        
        // 检查当前时间与上一次收到 Beacon 的时间差
        if ((rt_tick_get() - last_beacon_tick) > BEACON_TIMEOUT_MS)
        {
            // 如果超时了 (说明错过了一个周期)
            missed_beacon_cnt++;
            rt_kprintf("[WARN] Missed Beacon! Count: %d/%d\n", missed_beacon_cnt, MAX_MISSED_LIMIT);
            
            // 为了防止 print 刷屏，把 last_beacon_tick 往后推一个周期
            last_beacon_tick = rt_tick_get(); 

            // 如果连续丢失超过限制 -> 判定为掉线
            if (missed_beacon_cnt >= MAX_MISSED_LIMIT)
            {
                if (current_state == NODE_STATE_RUNNING) 
                {
                    rt_kprintf("!!! CONNECTION LOST !!! Switching to OFFLINE/SCANNING.\n");
                    current_state = NODE_STATE_OFFLINE; 
                }
            }
        }
        
        rt_thread_mdelay(10);
    }
}

// 简单的随机延时函数 (利用系统 tick 或未初始化的内存做种子)
uint32_t Get_Random_Delay(uint32_t min_ms, uint32_t max_ms) {
    // 如果没有 srand，每次上电序列可能一样。
    // 简单的办法是用 UID 或 ADC 悬空引脚的值做种子。
    // 这里简单处理，利用 HAL_GetTick() 的低位差异
    uint32_t random = rand() + rt_tick_get(); 
    return min_ms + (random % (max_ms - min_ms));
}

// LBT: 检测信道是否忙碌
uint8_t Is_Channel_Busy(void) {
    // 1. 获取当前瞬时 RSSI
    int current_rssi = LoRa_Get_Current_RSSI();
    
    // 2. 设定阈值
    // 您实测底噪在 64~68 之间。
    // 我们留出安全余量 (Margin)，设为 75。
    // 只有当信号强度超过 75 时，才认为真的有人在发包。
    int threshold = 75; 

    if (current_rssi > threshold) { 
        rt_kprintf("[CSMA] Busy! CurrRSSI: %d > Threshold: %d\n", current_rssi, threshold);
        return 1; // 忙
    }
    
    // 调试时可以打印空闲时的值，稳定后注释掉
    // rt_kprintf("[CSMA] Free. CurrRSSI: %d\n", current_rssi); 
    return 0; // 闲
}

// 获取当前的瞬时底噪
int LoRa_Get_Current_RSSI(void) {
    // 0x1B 是 RegRssiValue (Current RSSI)
    // 0x1A 是 RegPktRssiValue (Last Packet RSSI) <- 您之前读的是这个
    uint8_t raw_value = SX1278_SPIRead(&lora, 0x1B);
    
    // LoRa RSSI 计算公式通常是: -164 + raw_value (低频) 或 -157 + raw_value (高频)
    // 但根据您的打印 (57)，您的驱动可能没有做减法，直接返回了 raw_value。
    // 我们这里直接返回原始值用于比较，或者您可以根据实际情况 -164。
    // 为了配合您的打印习惯，我们暂时返回 raw_value (正数)。
    return (int)raw_value;
}


