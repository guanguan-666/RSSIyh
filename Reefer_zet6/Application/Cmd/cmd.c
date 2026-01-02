#include "rtthread.h"
#include "main.h"   // 引入CubeMX生成的头文件，以便识别 GPIOE/GPIOB 等宏
#include <string.h> // 用于 strcmp 比较字符串
#include "bsp_lora.h"
extern  SX1278_t lora;
/* ====== 开关灯 测试命令 ====== */
/* * 命令格式: led [color] [action]
 * 示例: 
 * led r on   (打开红灯)
 * led g off  (关闭绿灯)
 */
void led_ctrl(int argc, char** argv)
{
    /* 1. 参数检查：必须输入 3 个部分 (命令本身 + 颜色 + 动作) */
    if (argc < 3)
    {
        rt_kprintf("Usage: led [r/g] [on/off]\n");
        rt_kprintf("Example: led r on  (Turn on Red LED)\n");
        return;
    }

    /* 2. 解析颜色 (argv[1]) */
    GPIO_TypeDef* port = RT_NULL;
    uint16_t pin = 0;

    if (strcmp(argv[1], "r") == 0) //如果是 r (Red)
    {
        port = GPIOB;       // 你的红灯是 PE5
        pin = GPIO_PIN_5;
    }
    else if (strcmp(argv[1], "g") == 0) // 如果是 g (Green)
    {
        port = GPIOE;       // 你的绿灯是 PB5
        pin = GPIO_PIN_5;
    }
    else
    {
        rt_kprintf("Error: Unknown color! Use 'r' or 'g'.\n");
        return;
    }

    /* 3. 解析动作 (argv[2]) 并执行 */
    /* 注意：通常开发板 LED 是低电平点亮 (Active Low) */
    /* 如果你的灯是高电平亮，请反过来写 */
    
    if (strcmp(argv[2], "on") == 0)
    {
        // 低电平点亮 (RESET)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); 
        rt_kprintf("LED %s is ON\n", argv[1]);
    }
    else if (strcmp(argv[2], "off") == 0)
    {
        // 高电平熄灭 (SET)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); 
        rt_kprintf("LED %s is OFF\n", argv[1]);
    }
    else if (strcmp(argv[2], "toggle") == 0)
    {
        // 翻转状态
        HAL_GPIO_TogglePin(port, pin);
        rt_kprintf("LED %s Toggled\n", argv[1]);
    }
    else
    {
        rt_kprintf("Error: Unknown action! Use 'on', 'off' or 'toggle'.\n");
    }
}

/* 4. 导出到 FinSH 命令列表 */
/* 参数1: C函数名, 参数2: 命令别名(在终端里输的), 参数3: 描述 */
MSH_CMD_EXPORT_ALIAS(led_ctrl, led, Control LED: led [r/g] [on/off/toggle]);



/* ====== FinSH 测试命令 ====== */
void lora_test(void)
{
    // 手动读取版本号测试
    uint8_t ver = SX1278_SPIRead(&lora, 0x42);
    rt_kprintf("Manual Read: 0x%02X\n", ver);
}
MSH_CMD_EXPORT(lora_test, check lora connection);


/* * 用于测试发送 
 * 在终端输入: lora_send hello 即可发送
 */
void lora_send(int argc, char** argv)
{
    if (argc > 1) {
        LoRa_Send_Data((uint8_t*)argv[1], strlen(argv[1]));
    } else {
        rt_kprintf("Usage: lora_send <message>\n");
    }
}
MSH_CMD_EXPORT(lora_send, Send LoRa message);
