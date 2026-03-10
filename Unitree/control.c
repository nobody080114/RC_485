#include "control.h"
#include "stm32h7xx_hal_uart.h"
#include <string.h>

MotorCmd_t  cmd_0 = {0}, cmd_1 = {0}, cmd_2 = {0}, cmd_3 = {0},
            cmd_4 = {0}, cmd_5 = {0}, cmd_6 = {0}, cmd_7 = {0};
MotorData_t data_0 = {0}, data_1 = {0}, data_2 = {0}, data_3 = {0},
            data_4 = {0}, data_5 = {0}, data_6 = {0}, data_7 = {0};

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
    if(sch->tx_busy)   return;      // 正在发送，不操作
    if(!sch->rx_ready) return;      // 上一帧未收完，不发下一帧

    sch->rx_ready = 0;

    uint8_t idx = sch->current_motor;
    UART_HandleTypeDef *huart = get_motor_uart(sch, idx);

    modify_data(cmd_list[idx]);

    /* 将发送数据复制到 RAM_D2 DMA 缓冲区 */
    uint8_t *tx_buf = (idx < UART2_MOTOR_COUNT) ? dma_tx_buf_ch1 : dma_tx_buf_ch2;
    memcpy(tx_buf, &cmd_list[idx]->motor_send_data, sizeof(RIS_ControlData_t));

    HAL_UART_Transmit_DMA(huart, tx_buf, sizeof(RIS_ControlData_t));
    sch->tx_busy = 1;
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
    HAL_UART_Receive_DMA(huart, rx_buf, sizeof(RIS_MotorData_t));
    sch->tx_busy = 0;
}

/*
 * 接收完成回调处理 - 在 HAL_UART_RxCpltCallback 中调用
 * 解析数据后，推进到下一个电机，标记 rx_ready 触发下一轮调度
 */
void RS485_RxCpltHandler(RS485_Scheduler_t *sch, UART_HandleTypeDef *huart)
{
    uint8_t idx = sch->current_motor;
    UART_HandleTypeDef *expected = get_motor_uart(sch, idx);

    if(huart->Instance != expected->Instance) return;

    /* 将 RAM_D2 DMA 缓冲区数据复制回电机结构体，再解析 */
    uint8_t *rx_buf = (idx < UART2_MOTOR_COUNT) ? dma_rx_buf_ch1 : dma_rx_buf_ch2;
    memcpy(&data_list[idx]->motor_recv_data, rx_buf, sizeof(RIS_MotorData_t));
    extract_data(data_list[idx]);
    sch->current_motor = (sch->current_motor + 1) % TOTAL_MOTOR_COUNT;
    sch->rx_ready = 1;
}
