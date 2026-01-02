/*
 * File: stepper.c
 * 功能: 28BYJ-48 步进电机控制 (输入圈数和秒数)
 */

#include "rtthread.h"
#include "main.h"
#include <stdlib.h> // 包含 atof, abs

/* 如果包含了 rtthread.h 还是报错，可以在这里手动补一个定义（作为兜底） */
#ifndef RT_WAIT_FOREVER
#define RT_WAIT_FOREVER  ((-1))
#endif

/* ================= 硬件配置 ================= */
// 28BYJ-48 减速比 1:64，半步模式(8拍)下一圈约 4096 步
#define STEPS_PER_REV  4096 

// 如果 CubeMX 里没设置 User Label，这里手动定义
#ifndef MOTOR_IN1_Pin
#define MOTOR_IN1_GPIO_Port GPIOF
#define MOTOR_IN1_Pin       GPIO_PIN_0
#define MOTOR_IN2_GPIO_Port GPIOF
#define MOTOR_IN2_Pin       GPIO_PIN_1
#define MOTOR_IN3_GPIO_Port GPIOF
#define MOTOR_IN3_Pin       GPIO_PIN_2
#define MOTOR_IN4_GPIO_Port GPIOF
#define MOTOR_IN4_Pin       GPIO_PIN_3
#endif

/* ================= 全局变量 ================= */
static rt_thread_t stepper_tid = RT_NULL;
static rt_sem_t    stepper_sem = RT_NULL;

// 核心控制变量
static volatile int32_t  g_steps_target = 0; // 剩余步数
static volatile uint32_t g_step_delay   = 2; // 每步延时(ms)

/* ================= 8拍时序表 ================= */
const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

/* 硬件层：设置相位 */
static void stepper_set_phase(uint8_t phase_idx)
{
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, step_sequence[phase_idx][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, step_sequence[phase_idx][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, step_sequence[phase_idx][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, step_sequence[phase_idx][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 硬件层：断电 */
static void stepper_stop_hw(void)
{
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
}

/* ================= 线程入口 ================= */
static void stepper_thread_entry(void *parameter)
{
    static int phase_ptr = 0;
    
    while (1)
    {
        if (g_steps_target == 0)
        {
            stepper_stop_hw();
            rt_sem_take(stepper_sem, RT_WAIT_FOREVER); // 挂起等待新任务
        }

        if (g_steps_target != 0)
        {
            int direction = (g_steps_target > 0) ? 1 : -1;

            phase_ptr += direction;
            if (phase_ptr > 7) phase_ptr = 0;
            if (phase_ptr < 0) phase_ptr = 7;

            stepper_set_phase(phase_ptr);

            if (g_steps_target > 0) g_steps_target--;
            else g_steps_target++;

            rt_thread_mdelay(g_step_delay);
        }
    }
}

/* ================= 系统初始化 ================= */
static int stepper_init(void)
{
    stepper_sem = rt_sem_create("st_sem", 0, RT_IPC_FLAG_FIFO);
    stepper_tid = rt_thread_create("stepper", stepper_thread_entry, RT_NULL, 1024, 20, 10);
    if (stepper_tid != RT_NULL) rt_thread_startup(stepper_tid);
    return 0;
}
INIT_APP_EXPORT(stepper_init);

/* ================= FinSH 命令函数 ================= */
/* * 命令: motor_rev <圈数> <时间秒>
 * 示例: motor_rev 1 5    (正转1圈，耗时5秒)
 * 示例: motor_rev -2 10  (反转2圈，耗时10秒)
 * 示例: motor_rev 0 0    (停止)
 */
void motor_rev(int argc, char** argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: motor_rev <revolutions> <seconds>\n");
        rt_kprintf("Ex: motor_rev 1 5   (1 circle in 5s)\n");
        rt_kprintf("Ex: motor_rev 0.5 2 (0.5 circle in 2s)\n");
        rt_kprintf("Ex: motor_rev 0 0   (Stop)\n");
        return;
    }

    // 1. 获取参数 (支持小数)
    float revs = atof(argv[1]);
    float duration_sec = atof(argv[2]);

    // 2. 特殊处理：停止
    if (revs == 0.0f)
    {
        g_steps_target = 0;
        rt_kprintf("Motor Stopping...\n");
        return;
    }

    // 3. 计算步数 (总步数 = 圈数 * 4096)
    int32_t total_steps = (int32_t)(revs * STEPS_PER_REV);
    
    // 4. 计算延时 (总时间ms / 步数绝对值)
    // 防止除以0
    if (total_steps == 0) return; 
    
    uint32_t delay_ms = (uint32_t)((duration_sec * 1000.0f) / abs(total_steps));

    // 5. 物理速度限制保护
    // 28BYJ-48 极限大概在 1ms 左右，再快就只响不转了
    if (delay_ms < 1) 
    {
        delay_ms = 1;
        rt_kprintf("[WARNING] Too fast! Cap speed to max (1ms delay).\n");
    }

    // 6. 应用参数并唤醒线程
    g_step_delay = delay_ms;
    g_steps_target = total_steps;
    rt_sem_release(stepper_sem);

    // 7. 打印信息 (注意: rt_kprintf 默认可能不支持 float 打印，这里转成整数部分打印显示)
    rt_kprintf("CMD: %d steps, Delay: %d ms\n", total_steps, delay_ms);
}
// 导出命令
MSH_CMD_EXPORT(motor_rev, Control motor by revolutions and seconds);
