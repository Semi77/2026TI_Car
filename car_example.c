#include "ti_msp_dl_config.h"
#include "oled_hardware_i2c.h"
#include "mpu6050.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

#include <stdint.h>

#define OLED_FONT_8              8U
#define SENSOR_UPDATE_PERIOD_MS  10U
#define OLED_REFRESH_PERIOD_MS   100U

static volatile uint8_t g_10ms_flag = 0U;
static volatile uint32_t g_10ms_ticks = 0U;

static No_MCU_Sensor g_gray_sensor;
static unsigned short g_gray_analog[8] = {0};
static unsigned short g_gray_normal[8] = {0};
static unsigned char g_gray_digital = 0U;

static unsigned short g_gray_white[8] = {
    1800, 1800, 1800, 1800,
    1800, 1800, 1800, 1800
};

static unsigned short g_gray_black[8] = {
    300, 300, 300, 300,
    300, 300, 300, 300
};

static void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t value, uint8_t len)
{
    if (value < 0) {
        OLED_ShowChar(x, y, '-', OLED_FONT_8);
        value = -value;
    } else {
        OLED_ShowChar(x, y, '+', OLED_FONT_8);
    }

    OLED_ShowNum((uint8_t)(x + 6U), y, (uint32_t)value, len, OLED_FONT_8);
}

static void OLED_ClearField(uint8_t x, uint8_t y, uint8_t chars)
{
    for (uint8_t i = 0U; i < chars; i++) {
        OLED_ShowChar((uint8_t)(x + i * 6U), y, ' ', OLED_FONT_8);
    }
}

static void OLED_ShowDigitalByte(uint8_t x, uint8_t y, uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        uint8_t bit = (value >> i) & 0x01U;
        OLED_ShowChar((uint8_t)(x + i * 6U), y, (uint8_t)('0' + bit), OLED_FONT_8);
    }
}

static void OLED_DrawSensorLabels(void)
{
    OLED_ShowString(0, 0, (uint8_t *)"Y:", OLED_FONT_8);
    OLED_ShowString(60, 0, (uint8_t *)"GZ:", OLED_FONT_8);
    OLED_ShowString(0, 1, (uint8_t *)"D:", OLED_FONT_8);

    OLED_ShowString(0, 2, (uint8_t *)"0:", OLED_FONT_8);
    OLED_ShowString(66, 2, (uint8_t *)"1:", OLED_FONT_8);
    OLED_ShowString(0, 3, (uint8_t *)"2:", OLED_FONT_8);
    OLED_ShowString(66, 3, (uint8_t *)"3:", OLED_FONT_8);
    OLED_ShowString(0, 4, (uint8_t *)"4:", OLED_FONT_8);
    OLED_ShowString(66, 4, (uint8_t *)"5:", OLED_FONT_8);
    OLED_ShowString(0, 5, (uint8_t *)"6:", OLED_FONT_8);
    OLED_ShowString(66, 5, (uint8_t *)"7:", OLED_FONT_8);

    OLED_ShowString(0, 6, (uint8_t *)"N0:", OLED_FONT_8);
    OLED_ShowString(60, 6, (uint8_t *)"N7:", OLED_FONT_8);
}

static void OLED_UpdateGrayPair(uint8_t row, uint8_t left_index, uint8_t right_index)
{
    OLED_ClearField(12, row, 4);
    OLED_ShowNum(12, row, g_gray_analog[left_index], 4, OLED_FONT_8);

    OLED_ClearField(78, row, 4);
    OLED_ShowNum(78, row, g_gray_analog[right_index], 4, OLED_FONT_8);
}

static void OLED_UpdateSensorValues(void)
{
    int32_t yaw = (int32_t)MPU_Get_Yaw();
    int32_t gyro_z = (int32_t)MPU_Get_Gyro_Z();

    OLED_ClearField(12, 0, 4);
    OLED_ShowSignedNum(12, 0, yaw, 3);

    OLED_ClearField(78, 0, 4);
    OLED_ShowSignedNum(78, 0, gyro_z, 3);

    OLED_ClearField(12, 1, 8);
    OLED_ShowDigitalByte(12, 1, g_gray_digital);

    OLED_UpdateGrayPair(2, 0, 1);
    OLED_UpdateGrayPair(3, 2, 3);
    OLED_UpdateGrayPair(4, 4, 5);
    OLED_UpdateGrayPair(5, 6, 7);

    OLED_ClearField(18, 6, 4);
    OLED_ShowNum(18, 6, g_gray_normal[0], 4, OLED_FONT_8);

    OLED_ClearField(78, 6, 4);
    OLED_ShowNum(78, 6, g_gray_normal[7], 4, OLED_FONT_8);
}

int main(void)
{
    uint32_t last_oled_tick = 0U;

    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    No_MCU_Ganv_Sensor_Init(&g_gray_sensor, g_gray_white, g_gray_black);

    MPU6050_Init();
    delay_ms(2000);
    MPU6050_Calibration();

    OLED_Init();
    OLED_Clear();
    OLED_DrawSensorLabels();

    DL_TimerA_startCounter(CONTROL_TIMER_INST);
    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);

    while (1) {
        if (g_10ms_flag != 0U) {
            g_10ms_flag = 0U;

            MPU6050_Update_Yaw((float)SENSOR_UPDATE_PERIOD_MS);

            No_Mcu_Ganv_Sensor_Task_Without_tick(&g_gray_sensor);
            g_gray_digital = Get_Digtal_For_User(&g_gray_sensor);
            (void)Get_Anolog_Value(&g_gray_sensor, g_gray_analog);
            (void)Get_Normalize_For_User(&g_gray_sensor, g_gray_normal);

            if ((g_10ms_ticks - last_oled_tick) >=
                (OLED_REFRESH_PERIOD_MS / SENSOR_UPDATE_PERIOD_MS)) {
                last_oled_tick = g_10ms_ticks;
                OLED_UpdateSensorValues();
            }
        }
    }
}

void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST) == DL_TIMER_IIDX_ZERO) {
        DL_TimerA_clearInterruptStatus(CONTROL_TIMER_INST, DL_TIMER_IIDX_ZERO);
        g_10ms_ticks++;
        g_10ms_flag = 1U;
    }
}
