#include "fuzzyPID.h"
#include "rtthread.h"
#include "bsp_lora.h"
#include <string.h>
#include "main.h"
#include "usart.h"

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

static uint8_t is_matlab_mode = 0;
static uint8_t rx3_buf[sizeof(MatlabRxFrame_t)];
static struct PID **global_pid_vector = NULL;

#define PID_KP 15.0f
#define PID_KI 0.5f
#define PID_KD 2.5f
#define OUT_MAX 1000.0f
#define OUT_MIN -1000.0f
#define INTEGRAL_MAX 500.0f

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

void PID_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)"[IRQ]\r\n", 7, 10);
    
    if (! is_matlab_mode) {
        HAL_UART_Receive_IT(&huart3, rx3_buf, sizeof(MatlabRxFrame_t));
        return;
    }

    MatlabRxFrame_t *rx_frame = (MatlabRxFrame_t*)rx3_buf;
    
    char debug[80];
    sprintf(debug, "[RX] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            rx3_buf[0], rx3_buf[1], rx3_buf[2], rx3_buf[3], rx3_buf[4],
            rx3_buf[5], rx3_buf[6], rx3_buf[7], rx3_buf[8], rx3_buf[9]);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);
    
    if (rx_frame->header == 0xA5 && rx_frame->tail == 0x5A)
    {
        float target = rx_frame->target;
        float current = rx_frame->current;

        static float last_error = 0.0f;
        static float integral = 0.0f;

        float error = target - current;
        integral += error;
        
        if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
        else if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

        float p_out = PID_KP * error;
        float i_out = PID_KI * integral;
        float d_out = PID_KD * (error - last_error);
        float total_out = p_out + i_out + d_out;
        
        last_error = error;

        if (total_out > OUT_MAX) total_out = OUT_MAX;
        else if (total_out < OUT_MIN) total_out = OUT_MIN;

        int t_int = (int)target;
        int c_int = (int)current;
        int o_int = (int)total_out;
        sprintf(debug, "[PID] T:%d C:%d E: %.1f O:%d\r\n", t_int, c_int, error, o_int);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);

        MatlabTxFrame_t tx_frame;
        tx_frame.header = 0xA5;
        tx_frame.output = total_out;
        tx_frame.tail = 0x5A;
        
        HAL_UART_Transmit(&huart3, (uint8_t*)&tx_frame, sizeof(MatlabTxFrame_t), 10);
        HAL_UART_Transmit(&huart1, (uint8_t*)"[TX OK]\r\n", 9, 10);
    }
    else
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)"[FRAME ERR]\r\n", 13, 10);
    }

    HAL_UART_Receive_IT(&huart3, rx3_buf, sizeof(MatlabRxFrame_t));
}

extern volatile uint8_t is_lora_enable;

void pid_matlab(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: pid_matlab [0/1]\n");
        return;
    }

    int enable = atoi(argv[1]);

    if (enable) {
        is_lora_enable = 0;
        rt_kprintf("[System] LoRa PAUSED.\n");
        
        if (global_pid_vector == NULL) {
            global_pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);
            if (! global_pid_vector) {
                rt_kprintf("Memory Error!\n");
                return;
            }
        }
        
        is_matlab_mode = 1;
        
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
        
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart3, rx3_buf, sizeof(MatlabRxFrame_t));
        
        if (status == HAL_OK) {
            rt_kprintf("\n=== MATLAB HIL Mode STARTED ===\n");
            rt_kprintf("UART3 Interrupt Enabled Successfully\n");
            rt_kprintf("Waiting for MATLAB data on PB10/PB11 @ 115200.. .\n");
        } else {
            rt_kprintf("ERROR:  UART3 Init Failed (code %d)\n", status);
        }
        
    } else {
        is_matlab_mode = 0;
        is_lora_enable = 1;
        HAL_UART_AbortReceive_IT(&huart3);
        rt_kprintf("MATLAB HIL Mode STOPPED.\n");
    }
}
MSH_CMD_EXPORT(pid_matlab, Start/Stop Matlab HIL Mode);

void test_uart3(void)
{
    const char* msg = "Hello MATLAB!\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    rt_kprintf("Test message sent to UART3\n");
}
MSH_CMD_EXPORT(test_uart3, Test UART3 TX);
