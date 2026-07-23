#include "ADC.h"
volatile bool gCheckADC;

#define ADC_WAIT_TIMEOUT_LOOPS (100000U)

/* 该函数启动一次ADC转换并返回本次12位采样结果。 */
unsigned int adc_getValue(void)
{
        unsigned int gAdcResult = 0;
        uint32_t wait_count = 0U;

        gCheckADC = false;
        DL_ADC12_startConversion(ADC12_0_INST);

        while ((false == gCheckADC) &&
            (wait_count < ADC_WAIT_TIMEOUT_LOOPS)) {
            wait_count++;
        }
        if (false == gCheckADC) {
            return 0U;
        }

        gAdcResult = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_ADC_CH0);

        return gAdcResult;
}

/* 该函数处理ADC转换完成中断并通知阻塞采样函数读取结果。 */
void ADC12_0_INST_IRQHandler(void)
{
        switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST))
        {
              case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
                        gCheckADC = true;
                        break;
              default:
                        break;
        }
}
