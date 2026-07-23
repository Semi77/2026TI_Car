#ifndef ADC_H
#define ADC_H

#include "ti_msp_dl_config.h"

/* 该函数启动一次ADC转换并返回本次12位采样结果。 */
unsigned int adc_getValue(void);

#endif
