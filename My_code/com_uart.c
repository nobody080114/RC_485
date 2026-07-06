#include "com_uart.h"

__attribute__((section(".RAM_D2")))
static uint8_t COM_UART_TempRxBuff[12];
uint8_t COM_UART_RxData[12]; //
extern DMA_HandleTypeDef hdma_usart1_rx;
extern int8_t go_dir;
int16_t switch_speed = 0, go_speed = 0;

static int16_t COM_ClampSpeed(int16_t value, int16_t min_value, int16_t max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

void COM_UART_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)COM_UART_TempRxBuff, sizeof(COM_UART_TempRxBuff));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void COM_UART_Handle(void)
{
    if(COM_UART_TempRxBuff[0] == 0xAA && COM_UART_TempRxBuff[11] == 0xFF)
    {
        for(int i = 0; i < sizeof(COM_UART_RxData); i++)
        {
            COM_UART_RxData[i] = (COM_UART_TempRxBuff[i]);   
        }
        COM_GetData(&go_dir);
    }
    // Handle the received data in COM_UART_RxBuff
    // COM_GetData(&go_dir);
}

void COM_GetData(int8_t *go_dir)
{
    switch_speed = (int16_t)(COM_UART_RxData[10] << 8 | COM_UART_RxData[9]);
    go_speed = (int16_t)( (uint8_t)COM_UART_RxData[6] << 8 | (uint8_t)COM_UART_RxData[5] );
    switch_speed = COM_ClampSpeed(switch_speed, -20, 20);
    go_speed = COM_ClampSpeed(go_speed, -10, 10);

    if(switch_speed > 0 && go_speed == 0) *go_dir = 3; // Turn right
    else if(switch_speed < 0 && go_speed == 0) *go_dir = 4; // Turn left
    else if(go_speed > 0 && switch_speed == 0) *go_dir = 1; // Forward
    else if(go_speed < 0 && switch_speed == 0) *go_dir = 2; // Backward
    else if(go_speed > 0 && switch_speed > 0) *go_dir = 5; // Forward right
    else if(go_speed > 0 && switch_speed < 0) *go_dir = 6; // Forward left
    else if(go_speed < 0 && switch_speed > 0) *go_dir = 7; // Backward right
    else if(go_speed < 0 && switch_speed < 0) *go_dir = 8; // Backward left
    else *go_dir = 0; // Stop
}
