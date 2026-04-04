#include "control.h"
#include "stm32h7xx_hal_uart.h"
#include <string.h>
#include "motion.h"
MotorCmd_t  cmd_0 = {0}, cmd_1 = {0}, cmd_2 = {0}, cmd_3 = {0},
            cmd_4 = {0}, cmd_5 = {0}, cmd_6 = {0}, cmd_7 = {0};
MotorData_t data_0 = {0}, data_1 = {0}, data_2 = {0}, data_3 = {0},
            data_4 = {0}, data_5 = {0}, data_6 = {0}, data_7 = {0};
extern uint8_t start;
extern JointParam joint_param_0, joint_param_1;
/*
 * DMA 缓冲区 - 必须位于 RAM_D2 (0x30000000)，DMA1 可访问
 * UART2(ch1): 电机 0~3；UART3(ch2): 电机 4~7
 * 每路 UART 同一时刻仅操作一个电机，共享一对 TX/RX 缓冲区即可
 */
__attribute__((section(".RAM_D2")))
static uint8_t dma_tx_buf_ch1[sizeof(RIS_ControlData_t)];  /* UART2 TX */
__attribute__((section(".RAM_D2")))
static uint8_t dma_rx_buf_ch1[sizeof(RIS_MotorData_t)];    /* UART2 RX */
__attribute__((section(".RAM_D2")))
static uint8_t dma_tx_buf_ch2[sizeof(RIS_ControlData_t)];  /* UART3 TX */
__attribute__((section(".RAM_D2")))
static uint8_t dma_rx_buf_ch2[sizeof(RIS_MotorData_t)];    /* UART3 RX */

MotorCmd_t *cmd_list[TOTAL_MOTOR_COUNT] = {
    &cmd_0, &cmd_1, &cmd_2, &cmd_3,
    &cmd_4, &cmd_5, &cmd_6, &cmd_7
};
MotorData_t *data_list[TOTAL_MOTOR_COUNT] = {
    &data_0, &data_1, &data_2, &data_3,
    &data_4, &data_5, &data_6, &data_7
};

static uint8_t is_motor_enabled(RS485_Scheduler_t *sch, uint8_t idx)
{
    return (sch->motor_enable_mask & (1U << idx)) != 0U;
}

/* 从 start_idx 开始查找下一个启用电机；无可用电机返回 TOTAL_MOTOR_COUNT */
static uint8_t find_next_enabled_motor(RS485_Scheduler_t *sch, uint8_t start_idx)
{
    uint8_t i;

    for(i = 0; i < TOTAL_MOTOR_COUNT; i++)
    {
        uint8_t idx = (start_idx + i) % TOTAL_MOTOR_COUNT;
        if(is_motor_enabled(sch, idx))
        {
            return idx;
        }
    }

    return TOTAL_MOTOR_COUNT;
}

static void move_to_next_enabled_motor(RS485_Scheduler_t *sch)
{
    uint8_t next = find_next_enabled_motor(sch, (sch->current_motor + 1U) % TOTAL_MOTOR_COUNT);
    if(next < TOTAL_MOTOR_COUNT)
    {
        sch->current_motor = next;
    }
}

/* 根据电机索引返回对应的 UART 外设 */
static UART_HandleTypeDef *get_motor_uart(RS485_Scheduler_t *sch, uint8_t idx)
{
    return (idx < UART2_MOTOR_COUNT) ? sch->huart_ch1 : sch->huart_ch2;
}

/*
 * 统一调度函数 - 在 main loop 中不断调用，无需 Delay
 * 顺序: motor0(UART2) -> motor1 -> motor2 -> motor3
 *     -> motor4(UART3) -> motor5 -> motor6 -> motor7 -> 回到 motor0
 * 每个电机 发送->等发完->接收->等收完 后才轮到下一个电机
 */
void RS485_Schedule(RS485_Scheduler_t *sch)
{
    uint8_t idx;
    UART_HandleTypeDef *huart;

    if((sch->motor_enable_mask & RS485_ALL_MOTOR_MASK) == 0U)
    {
        return;
    }

    if(sch->tx_busy)   return;      // 正在发送，不操作

    if(!sch->rx_ready)
    {
        if(sch->waiting_rx)
        {
            uint32_t elapsed = HAL_GetTick() - sch->rx_start_tick;
            if(elapsed < sch->rx_timeout_ms)
            {
                return;
            }

            idx = sch->current_motor;
            huart = get_motor_uart(sch, idx);
            (void)HAL_UART_AbortReceive(huart);
            data_list[idx]->timeout++;

            sch->waiting_rx = 0;
            sch->rx_ready = 1;
            move_to_next_enabled_motor(sch);
        }
        else
        {
            return;
        }
    }

    idx = find_next_enabled_motor(sch, sch->current_motor);
    if(idx >= TOTAL_MOTOR_COUNT)
    {
        return;
    }

    sch->current_motor = idx;
    huart = get_motor_uart(sch, idx);

    sch->rx_ready = 0;
    sch->waiting_rx = 0;

    modify_data(cmd_list[idx]);

    /* 将发送数据复制到 RAM_D2 DMA 缓冲区 */
    uint8_t *tx_buf = (idx < UART2_MOTOR_COUNT) ? dma_tx_buf_ch1 : dma_tx_buf_ch2;
    memcpy(tx_buf, &cmd_list[idx]->motor_send_data, sizeof(RIS_ControlData_t));

    if(HAL_UART_Transmit_DMA(huart, tx_buf, sizeof(RIS_ControlData_t)) != HAL_OK)// 表示启动 DMA 发送失败
    {
        sch->rx_ready = 1;
        move_to_next_enabled_motor(sch);
        return;
    }

    sch->tx_busy = 1;// 发送成功，等待发送完成回调
}

/*
 * 发送完成回调处理 - 在 HAL_UART_TxCpltCallback 中调用
 * 发送结束后，立即在同一 UART 开启接收
 */
void RS485_TxCpltHandler(RS485_Scheduler_t *sch, UART_HandleTypeDef *huart)
{
    uint8_t idx = sch->current_motor;
    UART_HandleTypeDef *expected = get_motor_uart(sch, idx);

    if(huart->Instance != expected->Instance) return;

    /* 在 RAM_D2 DMA 缓冲区中接收 */
    uint8_t *rx_buf = (idx < UART2_MOTOR_COUNT) ? dma_rx_buf_ch1 : dma_rx_buf_ch2;
    if(HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf, sizeof(RIS_MotorData_t)) == HAL_OK)// 表示启用空闲中断接收成功
    {
        if(huart->hdmarx != NULL)/// 表示DMA接收通道有效
        {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
        sch->waiting_rx = 1;
        sch->rx_start_tick = HAL_GetTick();
    }
    else
    {
        data_list[idx]->timeout++;
        sch->waiting_rx = 0;
        sch->rx_ready = 1;
        move_to_next_enabled_motor(sch);
    }

    sch->tx_busy = 0;
}

/*
 * 接收完成回调处理 - 在 HAL_UARTEx_RxEventCallback 中调用
 * 解析数据后，推进到下一个电机，标记 rx_ready 触发下一轮调度
 */
void RS485_RxCpltHandler(RS485_Scheduler_t *sch, UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t idx = sch->current_motor;
    UART_HandleTypeDef *expected = get_motor_uart(sch, idx);
    const uint16_t expected_len = (uint16_t)sizeof(RIS_MotorData_t);

    if(!sch->waiting_rx) return;
    if(huart->Instance != expected->Instance) return;

    if(size != expected_len)
    {
        data_list[idx]->timeout++;
        sch->waiting_rx = 0;
        move_to_next_enabled_motor(sch);
        sch->rx_ready = 1;
        return;
    }

    /* 将 RAM_D2 DMA 缓冲区数据复制回电机结构体，再解析 */
    uint8_t *rx_buf = (idx < UART2_MOTOR_COUNT) ? dma_rx_buf_ch1 : dma_rx_buf_ch2;
    memcpy(&data_list[idx]->motor_recv_data, rx_buf, sizeof(RIS_MotorData_t));
    extract_data(data_list[idx]);
    // if(start == 0)
    // {
    //     joint_param_0.rotor_zero = data_0.Pos;
    //     joint_param_1.rotor_zero = data_1.Pos;
    //     start = 1;
    // }  
    sch->waiting_rx = 0;
    move_to_next_enabled_motor(sch);
    sch->rx_ready = 1;
}

void RS485_SetMotorMask(RS485_Scheduler_t *sch, uint16_t mask)
{
    sch->motor_enable_mask = mask & RS485_ALL_MOTOR_MASK;

    if(sch->motor_enable_mask == 0U)
    {
        sch->waiting_rx = 0;
        sch->rx_ready = 1;
        return;
    }

    sch->current_motor = find_next_enabled_motor(sch, sch->current_motor);
}

void RS485_SetRxTimeout(RS485_Scheduler_t *sch, uint32_t timeout_ms)
{
    sch->rx_timeout_ms = (timeout_ms == 0U) ? RS485_RX_TIMEOUT_DEFAULT_MS : timeout_ms;
}
