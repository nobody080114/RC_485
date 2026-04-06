#ifndef __CRSF_H__
#define __CRSF_H__

#include "main.h"

void CRSF_Init(void);
void CRSF_AcceptData(void);
void CRSF_Decode(void);
uint16_t CRSF_GetData(uint8_t Channelx);
void CRSF_Key_Get(uint16_t *key);

#endif

