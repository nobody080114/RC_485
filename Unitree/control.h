#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32h7xx_hal.h"
#include "gom_protocol.h"

#define UART2_MOTOR_COUNT  4    // UART2 上的电机数量 (id 0~3)
#define UART3_MOTOR_COUNT  4    // UART3 上的电机数量 (id 4~7)
#define TOTAL_MOTOR_COUNT  (UART2_MOTOR_COUNT + UART3_MOTOR_COUNT)

typedef struct
{
  uint8_t current_motor;              // 全局电机索引 0 ~ TOTAL_MOTOR_COUNT-1
  uint8_t tx_busy;                    // 正在发送
  uint8_t rx_ready;                   // 已收到反馈，可发下一帧
  UART_HandleTypeDef *huart_ch1;      // UART2 (电机 0~3)
  UART_HandleTypeDef *huart_ch2;      // UART3 (电机 4~7)
} RS485_Scheduler_t;

void RS485_Schedule(RS485_Scheduler_t *sch);
void RS485_TxCpltHandler(RS485_Scheduler_t *sch, UART_HandleTypeDef *huart);
void RS485_RxCpltHandler(RS485_Scheduler_t *sch, UART_HandleTypeDef *huart);

extern MotorCmd_t  cmd_0, cmd_1, cmd_2, cmd_3, cmd_4, cmd_5, cmd_6, cmd_7;
extern MotorData_t data_0, data_1, data_2, data_3, data_4, data_5, data_6, data_7;
extern MotorCmd_t  *cmd_list[TOTAL_MOTOR_COUNT];
extern MotorData_t *data_list[TOTAL_MOTOR_COUNT];

#endif
