#include "CRSF.h"
#include "usart.h"

// __attribute__((section(".RAM_D2")))
// uint8_t UARTRxData = 0;
// __attribute__((section(".RAM_D2")))
// static uint8_t UARTRxBuff[26];
// __attribute__((section(".RAM_D2")))
// static uint16_t CRSF[16];

uint8_t UARTRxData = 0;
static uint8_t UARTRxBuff[26];
static uint16_t CRSF[16];
/**
  * 函    数：CRSF初始化
  * 参    数：无
  * 返 回 值：无
  */
void CRSF_Init(void)
{
    HAL_UART_Receive_IT(&huart5, UARTRxBuff,26);
}



/**
  * 函    数：CRSF接收数据
  * 参    数：无
  * 返 回 值：无
  */
void CRSF_AcceptData(void)
{
    static uint8_t RxState = 0;
	static uint8_t TempRxBuff[26];
	static uint8_t Rxi = 0;
	uint8_t i = 0;
    if(RxState == 0)
		{
			if(UARTRxData == 0xC8)
			{
				RxState = 1;
			}
		}
		else if(RxState == 1)
		{
			if(Rxi < 25)
			{
				TempRxBuff[Rxi] = UARTRxData;
				Rxi++;
			}
			else if(Rxi >= 25)
			{
				RxState = 0;
				for(i = 0;i < 26; i++)
				{
					UARTRxBuff[i] = TempRxBuff[i];
				}
				Rxi = 0;
			}
		}

}

/**
  * 函    数：CRSF解码
  * 参    数：无
  * 返 回 值：无
  */
void CRSF_Decode(void)
{
	if(UARTRxBuff[1] == 22)
	{
        CRSF[0] = ((UARTRxBuff[2]>>0)|(UARTRxBuff[3]<<8)) & 0x07FF;
        CRSF[1] = ((UARTRxBuff[3]>>3)|(UARTRxBuff[4]<<5)) & 0x07FF;
        CRSF[2] = ((UARTRxBuff[4]>>6)|(UARTRxBuff[5]<<2)|(UARTRxBuff[6]<<10)) & 0x07FF;
        CRSF[3] = ((UARTRxBuff[6]>>1)|(UARTRxBuff[7]<<7)) & 0x07FF;
        CRSF[4] = ((UARTRxBuff[7]>>4)|(UARTRxBuff[8]<<4)) & 0x07FF;
        CRSF[5] = ((UARTRxBuff[8]>>7)|(UARTRxBuff[9]<<1)|(UARTRxBuff[10]<<9)) & 0x07FF;
        CRSF[6] = ((UARTRxBuff[10]>>2)|(UARTRxBuff[11]<<6)) & 0x07FF;
        CRSF[7] = ((UARTRxBuff[11]>>5)|(UARTRxBuff[12]<<3)) & 0x07FF;
        CRSF[8] = ((UARTRxBuff[13]>>0)|(UARTRxBuff[14]<<8)) & 0x07FF;
        CRSF[9] = ((UARTRxBuff[14]>>3)|(UARTRxBuff[15]<<5)) & 0x07FF;
	}
	
}

/**
  * 函    数：CRSF返回数据
  * 参    数：Channelx 通道号
  * 返 回 值：数据
  */
uint16_t CRSF_GetData(uint8_t Channelx)
{
    uint16_t temp = 0;
    switch (Channelx)
    {
        case 1: temp = ((UARTRxBuff[2]>>0)|(UARTRxBuff[3]<<8)) & 0x07FF;break;
        case 2: temp = ((UARTRxBuff[3]>>3)|(UARTRxBuff[4]<<5)) & 0x07FF;break;
        case 3: temp = ((UARTRxBuff[4]>>6)|(UARTRxBuff[5]<<2)|(UARTRxBuff[6]<<10)) & 0x07FF;break;
        case 4: temp = ((UARTRxBuff[6]>>1)|(UARTRxBuff[7]<<7)) & 0x07FF;break;
        case 5: temp = ((UARTRxBuff[7]>>4)|(UARTRxBuff[8]<<4)) & 0x07FF;break;
        case 6: temp = ((UARTRxBuff[8]>>7)|(UARTRxBuff[9]<<1)|(UARTRxBuff[10]<<9)) & 0x07FF;break;
        case 7: temp = ((UARTRxBuff[10]>>2)|(UARTRxBuff[11]<<6)) & 0x07FF;break;
        case 8: temp = ((UARTRxBuff[11]>>5)|(UARTRxBuff[12]<<3)) & 0x07FF;break;
        case 9: temp = ((UARTRxBuff[13]>>0)|(UARTRxBuff[14]<<8)) & 0x07FF;break;
        case 10:temp = ((UARTRxBuff[14]>>3)|(UARTRxBuff[15]<<5)) & 0x07FF;break;
    }
    return temp;
}

void Key_Control(void)
{
  int16_t Key1  = CRSF_GetData(1);
	int16_t Key2  = CRSF_GetData(2);
	int16_t Key3  = CRSF_GetData(3);
  int16_t Key4  = CRSF_GetData(4);
	int16_t Key5  = CRSF_GetData(5);
	int16_t Key6  = CRSF_GetData(6);
  int16_t Key7  = CRSF_GetData(7);
	int16_t Key8  = CRSF_GetData(8);
	int16_t Key9  = CRSF_GetData(9);
  int16_t Key10 = CRSF_GetData(10);
}

