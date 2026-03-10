#include "my_system.h"

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能DWT
    DWT->CYCCNT = 0;                                 // 清零计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // 开始计数
}

void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}
