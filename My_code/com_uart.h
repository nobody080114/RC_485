#ifndef __COM_UART_H__
#define __COM_UART_H__

#include "main.h"

void COM_UART_Init(void);
void COM_UART_Handle(void);
void COM_GetData(int8_t *go_dir);
#endif // __COM_UART_H__