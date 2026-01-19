#include "fuzzyPID.h"
#include "rtthread.h"
#include "bsp_lora.h"
#include <string.h>
#include "main.h"
#include "usart.h"
#include <stdio.h>
#define DOF 6

volatile float target_temperature = 25.0f; 
volatile float current_temperature = 0.0f;
volatile float pid_out = 0.0f;
volatile uint8_t motor_state = 0;

#pragma pack(push, 1)
typedef struct {
    uint8_t  head;
    uint8_t  dev_id;
    float    temp;
    uint32_t seq;
} LoraPacket_t;
#pragma pack(pop)

static int rule_base[][qf_default] = {
        {PB, PB, PM, PM, PS, ZO, ZO},
        {PB, PB, PM, PS, PS, ZO, NS},
        {PM, PM, PM, PS, ZO, NS, NS},
        {PM, PM, PS, ZO, NS, NM, NM},
        {PS, PS, ZO, NS, NS, NM, NM},
        {PS, ZO, NS, NM, NM, NM, NB},
        {ZO, ZO, NM, NM, NM, NB, NB},
        {NB, NB, NM, NM, NS, ZO, ZO},
        {NB, NB, NM, NS, NS, ZO, ZO},
        {NB, NM, NS, NS, ZO, PS, PS},
        {NM, NM, NS, ZO, PS, PM, PM},
        {NM, NS, ZO, PS, PS, PM, PB},
        {ZO, ZO, PS, PS, PM, PB, PB},
        {ZO, ZO, PS, PM, PM, PB, PB},
        {PS, NS, NB, NB, NB, NM, PS},
        {PS, NS, NB, NM, NM, NS, ZO},
        {ZO, NS, NM, NM, NS, NS, ZO},
        {ZO, NS, NS, NS, NS, NS, ZO},
        {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
        {PB, PS, PS, PS, PS, PS, PB},
        {PB, PM, PM, PM, PS, PS, PB}
};

static int mf_params[4 * qf_default] = {
    -3, -3, -2, 0,
    -3, -2, -1, 0,
    -2, -1,  0, 0,
    -1,  0,  1, 0,
     0,  1,  2, 0,
     1,  2,  3, 0,
     2,  3,  3, 0
};

static float fuzzy_pid_params[DOF][pid_params_count] = {
    {0.65f,  0,     0,    0, 0, 0, 1},
    {-0.34f, 0,     0,    0, 0, 0, 1},
    {-1.1f,  0,     0,    0, 0, 0, 1},
    {-2.4f,  0,     0,    0, 0, 0, 1},
    {1.2f,   0,     0,    0, 0, 0, 1},
    {1.2f,   0.05f, 0.1f, 0, 0, 0, 1}
};

void print_float(const char* tag, float val)
{
    int integer = (int)val;
    int decimal = (int)((val - integer) * 1000);
    if (decimal < 0) decimal = -decimal;
    rt_kprintf("%s: %d.%03d\n", tag, integer, decimal);
}

void pid_sim(void) 
{
    rt_kprintf("Start Fuzzy PID Simulation.. .\n");
    struct PID **pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);
    
    if (pid_vector == NULL) {
        rt_kprintf("Error:  Malloc failed!\n");
        return;
    }

    int control_id = 5; 
    float real = 0;     
    float idea = max_error * 0.9f; 
    bool direct[DOF] = {true, false, false, false, true, true};

    for (int j = 0; j < 1000; ++j) {
        int out = fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
        real += (float)(out - middle_pwm_output) / (float)middle_pwm_output * (float)max_error * 0.1f;
        
        int r_int = (int)real;
        int r_dec = (int)((real - r_int) * 1000);
        if(r_dec < 0) r_dec = -r_dec;

        rt_kprintf("Step %d: Out=%d, Real=%d. %03d\n", j, out, r_int, r_dec);
        rt_thread_mdelay(20); 
    }

    delete_pid_vector(pid_vector, DOF);
    rt_kprintf("Simulation Finished.\n");
}
MSH_CMD_EXPORT(pid_sim, Run Fuzzy PID Simulation);

void pid_test(void)
{
    rt_kprintf("Starting PID + LoRa Manual Packing Test...\n");
    struct PID **pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);
    if (! pid_vector) return;

    int control_id = 5; 
    float real = 20.0f;
    float idea = max_error * 0.9f; 
    bool direct[DOF] = {true, false, false, false, true, true};
    static uint32_t global_seq = 0; 
    uint8_t tx_buf[10]; 

    for (int j = 0; j < 2000; ++j) {
        int out = fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
        real += 0.01f; 
        if (real > 30.0f) real = 20.0f;

        if (j % 50 == 0) {
            tx_buf[0] = 0xAA;
            tx_buf[1] = 0x01;
            memcpy(&tx_buf[2], &real, 4);
            tx_buf[6] = (uint8_t)(global_seq & 0xFF);
            tx_buf[7] = (uint8_t)((global_seq >> 8) & 0xFF);
            tx_buf[8] = (uint8_t)((global_seq >> 16) & 0xFF);
            tx_buf[9] = (uint8_t)((global_seq >> 24) & 0xFF);

            int r_int = (int)real;
            int r_dec = (int)((real - r_int) * 100);
            if(r_dec<0) r_dec = -r_dec;
            rt_kprintf("[TX] Seq:%d | Temp:%d.%02d\n", global_seq, r_int, r_dec);

            LoRa_Send_Data(tx_buf, 10);
            global_seq++;
        }
        rt_thread_mdelay(20); 
    }

    delete_pid_vector(pid_vector, DOF);
    rt_kprintf("Test Finished.\n");
}
MSH_CMD_EXPORT(pid_test, Run PID Manual Pack);

// 定义接收缓冲区 (如果你的 .h 里没声明，这里补上)
volatile uint8_t is_matlab_mode = 1;       // 默认关闭 MATLAB 模式
uint8_t rx3_buf[20];  // 接收缓冲区

/* 包含必要的头文件 */

#include "app_lora_protocol.h"
#include "usart.h"
#include <math.h>
#include "stm32f1xx_hal.h"
// ==========================================
// ?? 1. 在这里填入你的仿真最优参数
// ==========================================
// 根据你之前的仿真结果 (例如: Kp=0.386, Ki=0.005, Kd=15.0)
// 如果你有新的参数，请修改这三个值
// 给它们赋一个安全的初始值（就是你现在的参数）
volatile float g_pid_kp = 0.386f;
volatile float g_pid_ki = 0.001795f;
volatile float g_pid_kd = 7.4f; 

// 限制参数 (保护执行器)
#define INTEGRAL_MAX  4000.0f   // 积分限幅 (防止长期误差导致积分爆炸)
#define OUTPUT_MAX    2000.0f  // 输出最大值 (对应 PWM 满占空比)

/*
 * 串口接收完成回调函数 (PID 计算核心)
 * 每当 MATLAB 发送来一组数据 (Target, Current) 时触发
 */
// 声明 PID 内部的静态变量，方便我们在更新参数时清零积分项
// (这些变量原来是在函数内部的 static，建议移到函数外面变成全局 static，或者提供一个 Reset 函数)
// 静态变量保持状态
static float last_error = 0.0f;
static float integral_sum = 0.0f;

//void PID_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    // 1. 安全检查: 如果没开启 HIL 模式，仅维持接收但不处理
//    // (这里也必须接收 14 字节，防止数据错位堆积)
//    if (!is_matlab_mode) {
//        HAL_UART_Receive_IT(&huart3, rx3_buf, 14); 
//        return;
//    }

//    // 2. 准备校验索引
//    // MatlabRxFrame_t (控制包) 通常为 10 字节，帧尾在 index 9
//    // MatlabParamFrame_t (参数包) 通常为 14 字节，帧尾在 index 13
//    uint8_t head = rx3_buf[0];
//    uint8_t tail_pos_control = sizeof(MatlabRxFrame_t) - 1;   
//    uint8_t tail_pos_param   = sizeof(MatlabParamFrame_t) - 1; 

//    // ==========================================================
//    // 情况 A: 收到 PID 控制指令 (0xA5)
//    // ==========================================================
//    // 校验：头是 0xA5 且 第10个字节(index 9) 是 0x5A
//    if (head == 0xA5 && rx3_buf[tail_pos_control] == 0x5A) 
//    {
//        MatlabRxFrame_t *rx_frame = (MatlabRxFrame_t*)rx3_buf;
//        
//        float target = rx_frame->target;
//        float current = rx_frame->current;

//        // --- PID 核心算法 ---
//        float error = target - current;

//        integral_sum += error;
//        // 积分抗饱和
//        if (integral_sum > INTEGRAL_MAX) integral_sum = INTEGRAL_MAX;
//        else if (integral_sum < -INTEGRAL_MAX) integral_sum = -INTEGRAL_MAX;

//        // 使用全局变量 g_pid_... 计算
//        float p_out = g_pid_kp * error;
//        float i_out = g_pid_ki * integral_sum;
//        float d_out = g_pid_kd * (error - last_error);

//        float total_out = p_out + i_out + d_out;
//        last_error = error;

//        // 输出限幅
//        if (total_out > OUTPUT_MAX) total_out = OUTPUT_MAX;
//        else if (total_out < -OUTPUT_MAX) total_out = -OUTPUT_MAX;

//        // 发送回 Matlab (发送浮点数)
//        MatlabTxFrame_t tx_frame;
//        tx_frame.header = 0xA5;
//        tx_frame.output = total_out; 
//        tx_frame.tail   = 0x5A;
//        
//        // 发送 6 字节 (4字节float + 头尾)
//        HAL_UART_Transmit(&huart3, (uint8_t*)&tx_frame, sizeof(MatlabTxFrame_t), 10);
//    }
//    // ==========================================================
//    // 情况 B: 收到 参数更新指令 (0xB6)
//    // ==========================================================
//    // 校验：头是 0xB6 且 第14个字节(index 13) 是 0x5A
//    else if (head == 0xB6 && rx3_buf[tail_pos_param] == 0x5A)
//    {
//        MatlabParamFrame_t *param_frame = (MatlabParamFrame_t*)rx3_buf;

//        // 1. 更新全局 PID 参数
//        g_pid_kp = param_frame->new_kp;
//        g_pid_ki = param_frame->new_ki;
//        g_pid_kd = param_frame->new_kd;

//        // 2. 清零历史状态 (防止参数突变导致系统震荡)
//        integral_sum = 0.0f;
//        last_error = 0.0f;

//        // (调试用) 可在此处翻转 LED 指示参数更新成功
//    }
//    
//    // ==========================================================
//    // 3. [关键修正] 重新开启中断
//    // ==========================================================
//    // 必须强制设为 14，因为 Matlab 脚本现在固定发送 14 字节
//    // 即使是控制包(10字节有效)，Matlab 也会补 4 个字节的 0
//    HAL_UART_Receive_IT(&huart3, rx3_buf, 14);
//}

void PID_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // =================================================================
    // ?? 调试第一步：不管三七二十一，先打印收到的原始数据
    // =================================================================
    rt_kprintf("\n[RX Raw]: ");
    for (int i = 0; i < 14; i++) {
        rt_kprintf("%02X ", rx3_buf[i]);
    }
    rt_kprintf("\n");
    // =================================================================

    // 1. 安全检查 (为了调试，暂时注释掉拦截，强制运行！)
    // if (!is_matlab_mode) {
    //     __HAL_UART_CLEAR_OREFLAG(&huart3);
    //     HAL_UART_Receive_IT(&huart3, rx3_buf, 14);
    //     return;
    // }

    uint8_t head = rx3_buf[0];

    // =================================================================
    // 情况 A: 收到 PID 控制指令 (0xA5)
    // =================================================================
    // 校验：头是 A5，且第 10 个字节 (index 9) 是 5A
    if (head == 0xA5 && rx3_buf[9] == 0x5A) 
    {
        MatlabRxFrame_t *rx_frame = (MatlabRxFrame_t*)rx3_buf;
        float target = rx_frame->target;
        float current = rx_frame->current;
        
        // 打印解析结果，看看单片机读到的数对不对
        rt_kprintf(" -> [CMD: Control] Tgt: %d, Cur: %d\n", (int)target, (int)current);

        // --- PID 计算 ---
        float error = target - current;
        integral_sum += error;
        
        // 积分抗饱和
        if (integral_sum > 1000.0f) integral_sum = 1000.0f;
        else if (integral_sum < -1000.0f) integral_sum = -1000.0f;

        float p_out = g_pid_kp * error;
        float i_out = g_pid_ki * integral_sum;
        float d_out = g_pid_kd * (error - last_error);
        float total_out = p_out + i_out + d_out;
        last_error = error;

        // 输出限幅
        if (total_out > 100.0f) total_out = 100.0f;
        else if (total_out < -100.0f) total_out = -100.0f;

        // --- 回复 Matlab ---
        MatlabTxFrame_t tx_frame;
        tx_frame.header = 0xA5;
        tx_frame.output = total_out; 
        tx_frame.tail   = 0x5A;
        
        HAL_UART_Transmit(&huart3, (uint8_t*)&tx_frame, sizeof(MatlabTxFrame_t), 50);
        
        rt_kprintf(" -> Sent Output: %d (x100)\n", (int)(total_out * 100));
    }
    // =================================================================
    // 情况 B: 收到 参数更新指令 (0xB6)
    // =================================================================
    // 校验：头是 B6，且第 14 个字节 (index 13) 是 5A
    else if (head == 0xB6 && rx3_buf[13] == 0x5A)
    {
        MatlabParamFrame_t *param_frame = (MatlabParamFrame_t*)rx3_buf;
        
        g_pid_kp = param_frame->new_kp;
        g_pid_ki = param_frame->new_ki;
        g_pid_kd = param_frame->new_kd;
        
        // 换参数时清零状态
        integral_sum = 0.0f;
        last_error = 0.0f;
        
        rt_kprintf(" -> [CMD: Param Update] OK! New Kp: %d (x1000)\n", (int)(g_pid_kp * 1000));
    }
    // =================================================================
    // 情况 C: 数据错位或无效
    // =================================================================
    else
    {
        rt_kprintf(" -> [ERROR] Invalid Frame! Head=%02X, Tail9=%02X, Tail13=%02X\n", 
                   head, rx3_buf[9], rx3_buf[13]);
    }

    // 3. 容错与重启接收 (关键！)
    __HAL_UART_CLEAR_OREFLAG(&huart3);         // 清除溢出标志
    HAL_UART_Receive_IT(&huart3, rx3_buf, 14); // 重新开启 14 字节接收
}

extern volatile uint8_t is_lora_enable;

void pid_matlab(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage:  pid_matlab [0/1]\n");
        rt_kprintf("  pid_matlab 1  - Start MATLAB HIL mode\n");
        rt_kprintf("  pid_matlab 0  - Stop MATLAB HIL mode\n");
        return;
    }

    int enable = atoi(argv[1]);

    if (enable) {
        // ==================== 启动模式 ====================
        
        // 1. 暂停 LoRa 任务
        is_lora_enable = 0;
        rt_thread_mdelay(50);  // 等待 LoRa 任务停止
        rt_kprintf("[System] LoRa PAUSED.\n");
        
        // 2. 完整清理 UART3 状态（关键！）
        HAL_UART_AbortReceive(&huart3);  // 终止之前的接收
        
        // 清除硬件 FIFO 残留数据
        __HAL_UART_FLUSH_DRREGISTER(&huart3);
        
        // 清除所有标志位
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE | UART_FLAG_ORE | 
                                       UART_FLAG_IDLE | UART_FLAG_TC);
        
        // 清空软件缓冲区
        memset(rx3_buf, 0, sizeof(rx3_buf));
        
        // 3. 启动接收中断（14 字节固定长度）
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart3, rx3_buf, 14);
        
        if (status == HAL_OK) {
            // 4. 标记进入 MATLAB 模式（放在最后，避免中断提前触发）
            is_matlab_mode = 1;
            
            rt_kprintf("\n╔════════════════════════════════════╗\n");
            rt_kprintf("║   MATLAB HIL Mode STARTED         ║\n");
            rt_kprintf("╚════════════════════════════════════╝\n");
            rt_kprintf("  UART3: PB10(TX) / PB11(RX)\n");
            rt_kprintf("  Baud:   115200\n");
            rt_kprintf("  Frame: 14 bytes (0xA5/0xB6 + data + checksum)\n");
            rt_kprintf("  Waiting for MATLAB connection...\n\n");
        } else {
            rt_kprintf("[ERROR] UART3 Init Failed (code %d)\n", status);
            is_lora_enable = 1;  // 失败则恢复 LoRa
        }
        
    } else {
        // ==================== 退出模式 ====================
        
        // 1. 停止接收中断
        HAL_UART_AbortReceive_IT(&huart3);
        
        // 2. 等待当前数据处理完毕
        rt_thread_mdelay(10);
        
        // 3. 清理状态
        memset(rx3_buf, 0, sizeof(rx3_buf));
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE | UART_FLAG_ORE);
        
        // 4. 更新模式标志
        is_matlab_mode = 0;
        is_lora_enable = 1;
        
        rt_kprintf("\n[System] MATLAB HIL Mode STOPPED.\n");
        rt_kprintf("[System] LoRa Resumed.\n");
    }
}
MSH_CMD_EXPORT(pid_matlab, Start/Stop MATLAB HIL Mode);

void test_uart3(void)
{
    const char* msg = "Hello MATLAB!\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    rt_kprintf("Test message sent to UART3\n");
}
MSH_CMD_EXPORT(test_uart3, Test UART3 TX);
