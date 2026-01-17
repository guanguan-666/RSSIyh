#include "fuzzyPID.h"
#include "rtthread.h"
#include "bsp_lora.h" // [新增] 引入LoRa驱动头文件
#include <string.h>
#include "main.h"
#define DOF 6

// ============================================
// [新增] 全局共享变量定义 (Global Definitions)
// 这些变量会被 LoRa 线程读取或修改
// ============================================

// 目标温度 (默认 25.0 度)
// 使用 volatile 关键字，防止编译器过度优化，因为它是被多线程共享的
volatile float target_temperature = 25.0f; 

// 当前温度 (由传感器更新)
volatile float current_temperature = 0.0f;

// PID 计算输出值 (用于 LoRa 上报)
volatile float pid_out = 0.0f;

// 电机状态 (0:停止, 1:运行)
volatile uint8_t motor_state = 0;

// ============================================


/* ====== 1. 修改协议结构体 ====== */
/* 使用 __packed 关键字 (Keil/ARMCC 专用) 或 #pragma pack */
/* 为了保险，我们用最通用的写法 */

#pragma pack(push, 1) // 压栈并强制1字节对齐
typedef struct {
    uint8_t  head;      // 0: 帧头
    uint8_t  dev_id;    // 1: ID
    float    temp;      // 2-5: 温度 (无填充)
    uint32_t seq;       // 6-9: 序号
} LoraPacket_t;
#pragma pack(pop)     // 恢复默认对齐

/* 1. 【关键修改】将大数组移到函数外部，并加上 static 关键字 */
/* 这样它们就不占线程栈空间了，而是占全局内存 */
static int rule_base[][qf_default] = {
        //delta kp rule base
        {PB, PB, PM, PM, PS, ZO, ZO},
        {PB, PB, PM, PS, PS, ZO, NS},
        {PM, PM, PM, PS, ZO, NS, NS},
        {PM, PM, PS, ZO, NS, NM, NM},
        {PS, PS, ZO, NS, NS, NM, NM},
        {PS, ZO, NS, NM, NM, NM, NB},
        {ZO, ZO, NM, NM, NM, NB, NB},
        //delta ki rule base
        {NB, NB, NM, NM, NS, ZO, ZO},
        {NB, NB, NM, NS, NS, ZO, ZO},
        {NB, NM, NS, NS, ZO, PS, PS},
        {NM, NM, NS, ZO, PS, PM, PM},
        {NM, NS, ZO, PS, PS, PM, PB},
        {ZO, ZO, PS, PS, PM, PB, PB},
        {ZO, ZO, PS, PM, PM, PB, PB},
        //delta kd rule base
        {PS, NS, NB, NB, NB, NM, PS},
        {PS, NS, NB, NM, NM, NS, ZO},
        {ZO, NS, NM, NM, NS, NS, ZO},
        {ZO, NS, NS, NS, NS, NS, ZO},
        {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
        {PB, PS, PS, PS, PS, PS, PB},
        {PB, PM, PM, PM, PS, PS, PB}};

static int mf_params[4 * qf_default] = {-3, -3, -2, 0,
                                 -3, -2, -1, 0,
                                 -2, -1,  0, 0,
                                 -1,  0,  1, 0,
                                  0,  1,  2, 0,
                                  1,  2,  3, 0,
                                  2,  3,  3, 0};

static float fuzzy_pid_params[DOF][pid_params_count] = {{0.65f,  0,     0,    0, 0, 0, 1},
                                                 {-0.34f, 0,     0,    0, 0, 0, 1},
                                                 {-1.1f,  0,     0,    0, 0, 0, 1},
                                                 {-2.4f,  0,     0,    0, 0, 0, 1},
                                                 {1.2f,   0,     0,    0, 0, 0, 1},
                                                 {1.2f,   0.05f, 0.1f, 0, 0, 0, 1}};

/* 辅助函数：专门用来打印 float */
void print_float(const char* tag, float val)
{
    // 将浮点数拆分为整数部分和小数部分
    int integer = (int)val;
    int decimal = (int)((val - integer) * 1000); // 保留3位小数
    if (decimal < 0) decimal = -decimal; // 处理负数的小数部分
    
    // 打印格式： Tag: 12.345
    rt_kprintf("%s: %d.%03d\n", tag, integer, decimal);
}

void pid_sim(void) 
{
    rt_kprintf("Start Fuzzy PID Simulation...\n");

    // 2. 使用全局数组初始化
    struct PID **pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);

    if (pid_vector == NULL)
    {
        rt_kprintf("Error: Malloc failed! Check Heap Size.\n");
        return;
    }

    rt_kprintf("PID Init Success.\n");

    int control_id = 5; 
    float real = 0;     
    float idea = max_error * 0.9f; 
    
    // 使用辅助函数打印目标值
    print_float("Target", idea);
    
    bool direct[DOF] = {true, false, false, false, true, true};

    for (int j = 0; j < 1000; ++j) {
        int out = fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
        
        real += (float) (out - middle_pwm_output) / (float) middle_pwm_output * (float) max_error * 0.1f;
        
        // 3. 手动格式化打印：Step 0: Out=600, Real=12.345
        int r_int = (int)real;
        int r_dec = (int)((real - r_int) * 1000);
        if(r_dec < 0) r_dec = -r_dec;

        rt_kprintf("Step %d: Out=%d, Real=%d.%03d\n", j, out, r_int, r_dec);
        
        rt_thread_mdelay(20); 
    }

    delete_pid_vector(pid_vector, DOF);
    rt_kprintf("Simulation Finished.\n");
}
MSH_CMD_EXPORT(pid_sim, Run Fuzzy PID Simulation);


/* ====== 修复后的 pid_test 函数 (手动打包模式) ====== */
void pid_test(void)
{
    rt_kprintf("Starting PID + LoRa Manual Packing Test...\n");

    // 1. 初始化 PID (保持原样)
    struct PID **pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);
    if (!pid_vector) return;

    int control_id = 5; 
    float real = 20.0f; // 初始温度
    float idea = max_error * 0.9f; 
    bool direct[DOF] = {true, false, false, false, true, true};

    // 2. 定义关键变量
    // 【关键】必须是 static，否则每次函数退出或栈刷新会导致序号归零
    static uint32_t global_seq = 0; 
    
    // 发送缓冲区：固定 10 字节
    // [0]: Head, [1]: ID, [2-5]: Temp, [6-9]: Seq
    uint8_t tx_buf[10]; 

    for (int j = 0; j < 2000; ++j) { // 运行 40秒
        // PID 计算
        int out = fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
        
        // 模拟温度波动：让温度稍微变动一下，方便观察
        real += 0.01f; 
        if (real > 30.0f) real = 20.0f;

        // 3. 每 1秒 (50次 * 20ms) 发送一次
        if (j % 50 == 0) {
            // --- 手动打包 (Manual Packing) ---
            tx_buf[0] = 0xAA;       // 帧头
            tx_buf[1] = 0x01;       // 设备ID
            
            // 拷贝温度 float (4字节) 到 buf[2]~buf[5]
            memcpy(&tx_buf[2], &real, 4);
            
            // 拷贝序号 uint32 (4字节) 到 buf[6]~buf[9] (强制小端序 Little Endian)
            tx_buf[6] = (uint8_t)(global_seq & 0xFF);
            tx_buf[7] = (uint8_t)((global_seq >> 8) & 0xFF);
            tx_buf[8] = (uint8_t)((global_seq >> 16) & 0xFF);
            tx_buf[9] = (uint8_t)((global_seq >> 24) & 0xFF);

            // 打印日志方便对比
            int r_int = (int)real;
            int r_dec = (int)((real - r_int) * 100);
            if(r_dec<0) r_dec = -r_dec;
            rt_kprintf("[TX] Seq:%d | Temp:%d.%02d\n", global_seq, r_int, r_dec);

            // 发送
            LoRa_Send_Data(tx_buf, 10);
            
            // 序号递增
            global_seq++;
        }
        rt_thread_mdelay(20); 
    }

    delete_pid_vector(pid_vector, DOF);
    rt_kprintf("Test Finished.\n");
}
MSH_CMD_EXPORT(pid_test, Run PID Manual Pack);

/* ================================================================= */
/* 以下为新增的 MATLAB HIL 模式代码                   */
/* (Append this to the end of app_pid_test.c)              */
/* ================================================================= */

#include "usart.h" // 引入 huart3



// 变量
static uint8_t is_matlab_mode = 0;
static uint8_t rx3_buf[sizeof(MatlabRxFrame_t)]; // 接收缓冲区
static struct PID **global_pid_vector = NULL;    // 复用之前的 PID 指针
/* 引入必要的头文件，确保能用 memcpy 或 abs 等函数 */
#include <string.h>
#include <math.h>

/* 定义 PID 参数 (你可以直接在这里改，不用去其他文件找) */
#define PID_KP  15.0f   // 比例系数：负责响应速度
#define PID_KI  0.5f    // 积分系数：负责消除静态误差
#define PID_KD  2.5f    // 微分系数：负责抑制震荡

/* 定义限制参数 */
#define OUT_MAX 1000.0f // PWM 输出最大值 (例如 1000)
#define OUT_MIN -1000.0f
#define INT_MAX 500.0f  // 积分限幅 (防止积分饱和过大)
// --------------------------------------------------------
// 串口3 接收中断回调 (由 main.c 调用)
// --------------------------------------------------------
void PID_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 0. 安全检查
    if (!is_matlab_mode) {
        // 如果没开启模式，继续监听但不处理
        HAL_UART_Receive_IT(&huart3, rx3_buf, sizeof(MatlabRxFrame_t));
        return;
    }

    // 1. 简单的帧校验
    MatlabRxFrame_t *rx_frame = (MatlabRxFrame_t*)rx3_buf;
    
    if (rx_frame->header == 0xA5 && rx_frame->tail == 0x5A)
    {
        // 2. 提取数据 (MATLAB 发来的)
        float target = rx_frame->target;   // 目标值 (Idea)
        float current = rx_frame->current; // 当前值 (Real)

        // ==========================================================
        // 3. 【核心】手写 PID 算法实现 (替代外部函数)
        // ==========================================================
        
        // 使用 static 变量来保存上一次的状态 (因为中断结束后普通变量会消失)
        static float last_error = 0.0f; // 上一次的误差
        static float integral = 0.0f;   // 积分累加值

        // A. 计算当前误差
        float error = target - current;

        // B. 积分项计算 (带抗饱和限制)
        integral += error;
        if (integral > INT_MAX) integral = INT_MAX;
        else if (integral < -INT_MAX) integral = -INT_MAX;

        // C. PID 公式: P项 + I项 + D项
        // P = Kp * error
        // I = Ki * integral
        // D = Kd * (error - last_error) -> 简单的微分近似
        float p_out = PID_KP * error;
        float i_out = PID_KI * integral;
        float d_out = PID_KD * (error - last_error);

        float total_out = p_out + i_out + d_out;

        // D. 更新历史误差 (为下一次计算做准备)
        last_error = error;

        // E. 输出限幅 (Clamping)
        if (total_out > OUT_MAX) total_out = OUT_MAX;
        else if (total_out < OUT_MIN) total_out = OUT_MIN;

        // 最终转换为整数 (PWM 占空比)
        int out_int = (int)total_out;

        // ==========================================================
        // 4. 发送回 MATLAB
        // ==========================================================
        MatlabTxFrame_t tx_frame;
        tx_frame.header = 0xA5;
        tx_frame.output = (float)out_int; // 发送计算好的 PWM 值
        tx_frame.tail = 0x5A;
        
        // 发送 6 个字节
        HAL_UART_Transmit(&huart3, (uint8_t*)&tx_frame, sizeof(MatlabTxFrame_t), 10);
        
        // 【可选】调试打印：在串口调试助手看数据 (如果不接MATLAB的话)
        // rt_kprintf("T:%.1f C:%.1f E:%.1f Out:%d\n", target, current, error, out_int);
    }

    // 5. 重新开启接收中断 (接收下一帧)
    HAL_UART_Receive_IT(&huart3, rx3_buf, sizeof(MatlabRxFrame_t));
}
// --------------------------------------------------------
// 新增 FinSH 命令: pid_matlab
// 用法: pid_matlab 1 (开启) / pid_matlab 0 (关闭)
// --------------------------------------------------------
extern volatile uint8_t is_lora_enable;
void pid_matlab(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: pid_matlab [0/1]\n");
        return;
    }

    int enable = atoi(argv[1]);

    if (enable) {
        // 1. 【修改】关掉 LoRa 软开关
        // LoRa 线程检测到这个变量为 0 后，会自动休眠，不进行接收和打印
        is_lora_enable = 0;
        rt_kprintf("[System] LoRa Logic PAUSED (Soft Switch).\n");
        
        // 2. 初始化 PID (如果还没初始化)
        if (global_pid_vector == NULL) {
            global_pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);
            if (global_pid_vector == NULL) {
                rt_kprintf("Memory Error!\n");
                return;
            }
        }
        
        // 3. 开启接收中断
        is_matlab_mode = 1;
        // 建议清除一下 RXNE 标志位，防止有残留数据触发误动作
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);
        HAL_UART_Receive_IT(&huart3, rx3_buf, sizeof(MatlabRxFrame_t));
        
        rt_kprintf("\n=== MATLAB HIL Mode STARTED ===\n");
        rt_kprintf("UART3 (PB10/PB11) is listening.\n");
        
    } else {
        // 关闭 MATLAB 模式
        is_matlab_mode = 0;
        
        // 4. 【修改】恢复 LoRa 软开关
        is_lora_enable = 1;
        
        rt_kprintf("MATLAB HIL Mode STOPPED.\n");
        rt_kprintf("[System] LoRa Logic RESUMED.\n");
    }
}
MSH_CMD_EXPORT(pid_matlab, Start/Stop Matlab HIL Mode);

