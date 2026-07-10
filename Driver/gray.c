#include "gray.h"
#include "ti_msp_dl_config.h"

float Gray_Test(void)
{
    uint32_t port_val=DL_GPIO_readPins(GPIO_GRAY_PORT,GPIO_GRAY_L1_PIN | GPIO_GRAY_L2_PIN | GPIO_GRAY_L3_PIN | GPIO_GRAY_L4_PIN | GPIO_GRAY_R1_PIN | GPIO_GRAY_R2_PIN | GPIO_GRAY_R3_PIN | GPIO_GRAY_R4_PIN);
    int error[8];
    error[0]=(port_val & GPIO_GRAY_L1_PIN) ? 0 : 1;
    error[1]=(port_val & GPIO_GRAY_L2_PIN) ? 0 : 1;
    error[2]=(port_val & GPIO_GRAY_L3_PIN) ? 0 : 1;
    error[3]=(port_val & GPIO_GRAY_L4_PIN) ? 0 : 1;
    error[4]=(port_val & GPIO_GRAY_R4_PIN) ? 0 : 1;
    error[5]=(port_val & GPIO_GRAY_R3_PIN) ? 0 : 1;
    error[6]=(port_val & GPIO_GRAY_R2_PIN) ? 0 : 1;
    error[7]=(port_val & GPIO_GRAY_R1_PIN) ? 0 : 1;

    int sensor_sum = 0;
    float weight_sum = 0;
    //加权计算
    if(!error[0]){weight_sum -= 4.0; sensor_sum++;}
    if(!error[1]){weight_sum -= 3.0; sensor_sum++;}
    if(!error[2]){weight_sum -= 2.0; sensor_sum++;}
    if(!error[3]){weight_sum -= 1.0; sensor_sum++;}
    if(!error[4]){weight_sum += 1.0; sensor_sum++;}
    if(!error[5]){weight_sum += 2.0; sensor_sum++;}
    if(!error[6]){weight_sum += 3.0; sensor_sum++;}
    if(!error[7]){weight_sum += 4.0; sensor_sum++;}

    if(sensor_sum >= 5)
    {
        return 99;                       //五个传感器都检测到说明横线
    }

    if(sensor_sum == 0)
    {
        return 100;                      //无传感器检测到 表示丢线
    }

    return weight_sum / sensor_sum;      //返回正常数据
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

