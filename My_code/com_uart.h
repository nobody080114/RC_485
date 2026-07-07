#ifndef __COM_UART_H__
#define __COM_UART_H__

#include "main.h"

void COM_UART_Init(void);
void COM_UART_Handle(void);
void COM_GetData(int8_t *go_dir,float *angle,uint8_t *flag,uint8_t *set_flag);

extern int16_t switch_speed, go_speed;

#endif // __COM_UART_H__
