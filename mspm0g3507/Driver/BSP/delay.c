#include "delay.h"

/* 该函数阻塞延时指定的毫秒数，参数ms表示延时时长。 */
void delay_ms(uint32_t ms)
{
    DL_Common_delayCycles(ms * (CPUCLK_FREQ / 1000U));
}

/* 该函数阻塞延时指定的微秒数，参数us表示延时时长。 */
void delay_us(uint32_t us)
{
    DL_Common_delayCycles(us * (CPUCLK_FREQ / 1000000U));
}
