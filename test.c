#include "test.h"

#include "motor.h"
#include "oled_hardware_i2c.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

static void Test_DelayMs(uint32_t ms)
{
    DL_Common_delayCycles(ms * (CPUCLK_FREQ / 1000U));
}

static void Test_Motor_Step(int16_t left_speed, int16_t right_speed)
{
    Motor_Control(left_speed, right_speed);
    Test_DelayMs(MOTOR_RUN_TIME_MS);
    Motor_Stop();
    Test_DelayMs(MOTOR_STOP_TIME_MS);
}

void Test_OLED_Run(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"OLED TEST", 16);
    OLED_ShowString(0, 2, (uint8_t *)"MSPM0G3507", 16);
}

void Test_Motor_Run(void)
{
    Motor_Stop();
    Test_DelayMs(MOTOR_CYCLE_PAUSE_MS);

    while (1) {
        Test_Motor_Step(MOTOR_TEST_SPEED, 0);
        Test_Motor_Step(-MOTOR_TEST_SPEED, 0);
        Test_Motor_Step(0, MOTOR_TEST_SPEED);
        Test_Motor_Step(0, -MOTOR_TEST_SPEED);
        Test_Motor_Step(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED);
        Test_Motor_Step(-MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED);

        Test_DelayMs(MOTOR_CYCLE_PAUSE_MS);
    }
}
