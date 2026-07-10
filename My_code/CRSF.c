#include "CRSF.h"
#include <stdint.h>

__attribute__((section(".RAM_D2")))
static uint8_t TempRxBuff[26];
uint16_t CRSF[16];
int16_t need_angle;
extern DMA_HandleTypeDef hdma_uart5_rx;
static uint8_t UARTRxBuff[26];
extern uint16_t Key[10];
extern int8_t start_1,run,flag_1,mode,control_mode,Enable,go_dir;
extern int8_t jump_start_req,jump_armed;
extern uint16_t speed;
extern float angle;
extern uint8_t turn;
uint8_t DM_VCC_Flag,Last_DM_VCC_Flag,DM_Enable_Flag,DM_Enable;
uint8_t control_motor = 0;
/**
  * 函    数：CRSF初始化
  * 参    数：无
  * 返 回 值：无
  */
void CRSF_Init(void)
{
    // HAL_UART_Receive_DMA(&huart5, (uint8_t *)UARTRxBuff,26);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t *)TempRxBuff,26);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

void CRSF_AcceptData(void)
{
	  uint8_t i = 0;
    if(crsf_frame_crc_ok(TempRxBuff, 26))
    {
        for(i = 1;i < 26; i++)
        {
          UARTRxBuff[i-1] = TempRxBuff[i];
        }
    }

}


// /**
//   * 函    数：CRSF接收数据
//   * 参    数：无
//   * 返 回 值：无
//   */
// void CRSF_AcceptData(void)
// {
//     static uint8_t RxState = 0;
// 	static uint8_t TempRxBuff[26];
// 	static uint8_t Rxi = 0;
// 	uint8_t i = 0;
//     if(RxState == 0)
// 		{
// 			if(UARTRxData == 0xC8)
// 			{
// 				RxState = 1;
// 			}
// 		}
// 		else if(RxState == 1)
// 		{
// 			if(Rxi < 25)
// 			{
// 				TempRxBuff[Rxi] = UARTRxData;
// 				Rxi++;
// 			}
// 			else if(Rxi >= 25)
// 			{
// 				RxState = 0;
// 				for(i = 0;i < 26; i++)
// 				{
// 					UARTRxBuff[i] = TempRxBuff[i];
// 				}
// 				Rxi = 0;
// 			}
// 		}

// }

/**
  * 函    数：CRSF解码
  * 参    数：无
  * 返 回 值：无
  */
void CRSF_Decode(void)
{
	if(UARTRxBuff[1] == 0x16)
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

void CRSF_Key_Get(uint16_t *key)
{
    key[0] = CRSF_GetData(1);
    key[1] = CRSF_GetData(2);
    key[2] = CRSF_GetData(3);
    key[3] = CRSF_GetData(4);
    key[4] = CRSF_GetData(5);
    key[5] = CRSF_GetData(6);
    key[6] = CRSF_GetData(7);
    key[7] = CRSF_GetData(8);
    key[8] = CRSF_GetData(9);
    key[9] = CRSF_GetData(10);
}

void CRSF_Schedule(void)
{
    CRSF_Key_Get(Key);
    if(Key[9] < 800) //狗身控制
    {
      if(control_mode == 0 || control_mode == 1)
      {
        DM_VCC_Flag = 0;
        if(mode == 2)
        {
          if(Key[1] >= 1200 && Key[0]>=800 && Key[0]<=1200) go_dir = 1; //前进拨键
          else if(Key[1] <= 800 && Key[0]>=800 && Key[0]<=1200) go_dir = 2; //后退拨键
          else if(Key[0] >= 1200 && Key[1]>=800 && Key[1]<=1200) go_dir = 3;//原地右转拨键
          else if(Key[0] <= 800 && Key[1]>=800 && Key[1]<=1200) go_dir = 4;//原地左转拨键
          else if (Key[0] >= 1200 && Key[1]>=1200) go_dir = 5;//右前斜移拨键
          else if (Key[0] <= 800 && Key[1]>=1200) go_dir = 6;//左前斜移拨键
          else if (Key[0] >= 1200 && Key[1]<=800) go_dir = 7;//右后斜移拨键
          else if (Key[0] <= 800 && Key[1]<=800) go_dir = 8;//左后斜移拨键
          else go_dir = 0;//停
        }
        else if(mode == 3)
        {
          go_dir = 0;
          if(Key[1] >= 1200 && jump_armed && run == 1)
          {
            jump_start_req = 1;
            jump_armed = 0;
          }
          else if(Key[1] < 1100)
          {
            jump_armed = 1;
          }
        }
      }
      if(run == 1) speed = Key[2]-174; else speed = 0;//左拨杆
      
      if(Key[7] == 1792) run = 1; else run = 0;///右按键
      if(Key[8] == 1792) start_1 = 1; else start_1 = 0;//左上长按键
      if(Key[4] == 1792) Enable = 1; else Enable = 0;//左按键
      
      if(Key[5] == 191) mode = 2; else if(Key[5] == 997) mode = 1; else if(Key[5] == 1792) mode = 3;//左拨键 //1-站立，2-行走，3-跳跃
      // if(Key[9]<800) stand_state = 0; else if(Key[9]<1300 && Key[9]>=800) stand_state = 1; else stand_state = 2;//右上长按键
      if(Key[6] == 191) control_mode = 0; else if(Key[6] == 997) control_mode = 1; else if(Key[6] == 1792) control_mode = 2;//右拨键 //0-位控，1-力控，2-自动控制
    }
    else if(Key[9] > 800) //机械臂控制
    {
      DM_VCC_Flag = 1;
      if(DM_VCC_Flag == 1 && Last_DM_VCC_Flag == 0) DM_Enable_Flag = 1;
      if(Key[5] == 191) control_motor = 1; else if(Key[5] == 997) control_motor = 2; else if(Key[5] == 1792) control_motor = 3;//左拨键 //1-机械臂电机1，2-机械臂电机2，3-机械臂电机3
      need_angle = Key[1]-992;
      if(Key[4] == 1792) DM_Enable = 1; else DM_Enable = 0;//左按键
    }
    Last_DM_VCC_Flag = DM_VCC_Flag;
}
