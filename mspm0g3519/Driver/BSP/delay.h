#ifndef DELAY_H
#define DELAY_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 该函数阻塞延时指定的毫秒数，参数ms表示延时时长。 */
void delay_ms(uint32_t ms);

/* 该函数阻塞延时指定的微秒数，参数us表示延时时长。 */
void delay_us(uint32_t us);

#endif
