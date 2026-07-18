#include "gray.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

void Get_Analog_value(unsigned short *result);

float Gray_Test(void)
{
    static const float weights[8] = {
        -4.0f, -3.0f, -2.0f, -1.0f,
         1.0f,  2.0f,  3.0f,  4.0f
    };
    unsigned short adc_value[8];
    unsigned short min_value;
    unsigned short max_value;
    unsigned short threshold;
    uint8_t sensor_sum = 0U;
    float weight_sum = 0.0f;

    Get_Analog_value(adc_value);

    min_value = adc_value[0];
    max_value = adc_value[0];
    for (uint8_t channel = 1U; channel < 8U; channel++) {
        if (adc_value[channel] < min_value) {
            min_value = adc_value[channel];
        }
        if (adc_value[channel] > max_value) {
            max_value = adc_value[channel];
        }
    }

    if ((max_value - min_value) < 20U) {
        return 100.0f;
    }

    threshold = (unsigned short)((min_value + max_value) / 2U);
    for (uint8_t channel = 0U; channel < 8U; channel++) {
        if (adc_value[channel] < threshold) {
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

