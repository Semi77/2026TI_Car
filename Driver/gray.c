#include "gray.h"
#include "ti_msp_dl_config.h"

static void Gray_Set_Address_Pin(uint32_t pin, uint8_t state)
{
    if (state != 0U) {
        DL_GPIO_setPins(Gray_Address_PORT, pin);
    } else {
        DL_GPIO_clearPins(Gray_Address_PORT, pin);
    }
}

static uint8_t Gray_Read_Channel(uint8_t channel)
{
    Gray_Set_Address_Pin(Gray_Address_AD0_PIN, (channel >> 0U) & 0x01U);
    Gray_Set_Address_Pin(Gray_Address_AD1_PIN, (channel >> 1U) & 0x01U);
    Gray_Set_Address_Pin(Gray_Address_AD2_PIN, (channel >> 2U) & 0x01U);

    DL_Common_delayCycles((CPUCLK_FREQ / 1000000U) * 40U);

    return (DL_GPIO_readPins(Gray_OUT_PORT, Gray_OUT_OUT_PIN) != 0U) ? 1U : 0U;
}

float Gray_Test(void)
{
    static const float weights[8] = {
        -4.0f, -3.0f, -2.0f, -1.0f,
         1.0f,  2.0f,  3.0f,  4.0f
    };
    uint8_t sensor_sum = 0U;
    float weight_sum = 0.0f;

    for (uint8_t channel = 0U; channel < 8U; channel++) {
        if (Gray_Read_Channel(channel) != 0U) {
            weight_sum += weights[channel];
            sensor_sum++;
        }
    }

    if (sensor_sum >= 5U) {
        return 99.0f;
    }

    if (sensor_sum == 0U) {
        return 100.0f;
    }

    return weight_sum / (float)sensor_sum;
}

bool Check_Line_Found(float g_Gray_Error)
{
    if (g_Gray_Error == 100) 
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

