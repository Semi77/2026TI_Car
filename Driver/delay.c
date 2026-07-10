#include "delay.h"

void delay_us(uint32_t us)
{
    DL_Common_delayCycles(us * (CPUCLK_FREQ / 1000000U));
}
