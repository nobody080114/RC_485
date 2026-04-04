#ifndef __CRSF_H__
#define __CRSF_H__

#include "stm32h7xx_hal.h"

extern uint8_t UARTRxData;

void CRSF_Init(void);
void CRSF_AcceptData(void);
void CRSF_Decode(void);
uint16_t CRSF_GetData(uint8_t Channelx);
void Key_Control(void);
#endif

