#include "fuzzyPID.h"
#include "rtthread.h"
#include "bsp_lora.h" // [新增] 引入LoRa驱动头文件
#include <string.h>

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


