/* Original vehicle control application retained for later recovery. */
#include "ti_msp_dl_config.h"
#include "oled_hardware_i2c.h"
#include "IMU.h"
#include "imu_drdy.h"
#include "imu_host.h"
#include "imu_uart_port.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "motor.h"
#include "maxicam_uart.h"

#include <stdint.h>

#if 0
#define SENSOR_UPDATE_PERIOD_MS  10U
#define OLED_REFRESH_PERIOD_MS   100U
#define MOTOR_BASE_SPEED         400
#define MOTOR_MAX_SPEED          800
#define GRAY_PID_KP              30.0f
#define GRAY_PID_KI              0.0f
#define GRAY_PID_KD              5.0f
#define BLIND_BASE_SPEED         280
#define BLIND_LEFT_COMPENSATION  20
#define GYRO_PID_KP              1.5f
#define GYRO_PID_KI              0.0f
#define GYRO_PID_KD              0.8f
#define GYRO_PID_MAX_OUTPUT      100.0f
#define TURN_SPEED               180
#define TURN_ANGLE_DEGREES       83.0f
#define TURN_ANGLE_TOLERANCE     5.0f
#define TURN_ADVANCE_TIME_MS     800U
#define STATE_SWITCH_MIN_TICKS   100U
#define STATE_CONFIRM_TICKS      3U

typedef enum {
    CAR_STATE_TRACK = 0,
    CAR_STATE_ADVANCE_BEFORE_TURN,
    CAR_STATE_BLIND,
    CAR_STATE_TURN_LEFT,
    CAR_STATE_TURN_RIGHT
} CarState_t;

static volatile uint8_t g_10ms_flag = 0U;
static volatile uint32_t g_10ms_ticks = 0U;
static CarState_t g_car_state = CAR_STATE_TRACK;
static uint32_t g_state_ticks = 0U;
static uint8_t g_state_confirm_count = 0U;
static float g_blind_target_yaw = 0.0f;
static float g_turn_target_yaw = 0.0f;
static CarState_t g_pending_turn = CAR_STATE_TURN_LEFT;
static uint32_t g_turn_advance_start_tick = 0U;

static No_MCU_Sensor g_gray_sensor;
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

static float g_last_gray_error = 0.0f;
static float g_gray_integral = 0.0f;
static float g_last_gyro_error = 0.0f;
static float g_gyro_integral = 0.0f;

    /* A轮反向、B轮正向时，两轮才是车体的同向前进。 */
static float Gyro_WrapError(float error)
{
    while (error > 180.0f) {
        error -= 360.0f;
    }
    while (error < -180.0f) {
        error += 360.0f;
    }

    return error;
}

    /* 盲走实验：关闭角度 PID，右轮较快，因此提高左轮速度进行补偿。 */
static void Car_EnterBlind(float target_yaw)
{
    g_car_state = CAR_STATE_BLIND;
    g_pending_turn = CAR_STATE_TURN_LEFT;
    g_blind_target_yaw = target_yaw;
    g_last_gyro_error = 0.0f;
    g_gyro_integral = 0.0f;
    g_state_ticks = 0U;
    g_state_confirm_count = 0U;
}

static void Car_EnterAdvanceBeforeTurn(CarState_t turn_state)
{
    g_car_state = CAR_STATE_ADVANCE_BEFORE_TURN;
    g_pending_turn = turn_state;
    g_turn_advance_start_tick = g_10ms_ticks;
    g_last_gyro_error = 0.0f;
    g_gyro_integral = 0.0f;
    g_state_ticks = 0U;
    g_state_confirm_count = 0U;
}

static void Car_EnterTurn(CarState_t turn_state);

static void Car_RunAdvanceBeforeTurn(void)
{
    const uint32_t advance_ticks =
        (TURN_ADVANCE_TIME_MS + SENSOR_UPDATE_PERIOD_MS - 1U) /
        SENSOR_UPDATE_PERIOD_MS;

    if ((g_10ms_ticks - g_turn_advance_start_tick) >= advance_ticks) {
        Car_EnterTurn(g_pending_turn);
    } else {
        Motor_Forward(BLIND_BASE_SPEED);
    }
}

static void Car_EnterTurn(CarState_t turn_state)
{
    g_car_state = turn_state;
    g_turn_target_yaw = IMU_GetYaw();
    if (turn_state == CAR_STATE_TURN_LEFT) {
        g_turn_target_yaw += TURN_ANGLE_DEGREES;
    } else {
        g_turn_target_yaw -= TURN_ANGLE_DEGREES;
    }

    g_last_gyro_error = 0.0f;
    g_gyro_integral = 0.0f;
    g_state_ticks = 0U;
    g_state_confirm_count = 0U;
}

static void Car_RunTurn(CarState_t turn_state)
{
    float yaw_error = Gyro_WrapError(g_turn_target_yaw - IMU_GetYaw());

    if ((yaw_error <= TURN_ANGLE_TOLERANCE) &&
        (yaw_error >= -TURN_ANGLE_TOLERANCE)) {
        Motor_Stop();
        Car_EnterBlind(g_turn_target_yaw);
    } else if (turn_state == CAR_STATE_TURN_LEFT) {
        Motor_Control(-TURN_SPEED, -TURN_SPEED);
    } else {
        Motor_Control(TURN_SPEED, TURN_SPEED);
    }
}

static const char *Car_GetStateName(CarState_t state)
{
    switch (state) {
    case CAR_STATE_TRACK:
        return "TRACK";
    case CAR_STATE_BLIND:
        return "BLIND";
    case CAR_STATE_TURN_LEFT:
        return "TURN_LEFT";
    case CAR_STATE_TURN_RIGHT:
        return "TURN_RIGHT";
    default:
        return "UNKNOWN";
    }
}

static uint8_t Gray_CountBlack(uint8_t digital)
{
    uint8_t black_count = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if (((digital >> i) & 0x01U) == 0U) {
            black_count++;
        }
    }

    return black_count;
}

static uint8_t Gray_LeftTurnDetected(uint8_t digital)
{
    return (((digital & 0x0FU) == 0U) &&
            ((digital & 0xF0U) != 0U));
}

static uint8_t Gray_RightTurnDetected(uint8_t digital)
{
    return (((digital & 0xF0U) == 0U) &&
            ((digital & 0x0FU) != 0U));
}

static void Car_UpdateState(uint8_t digital)
{
    uint8_t black_count = Gray_CountBlack(digital);

    g_state_ticks++;

    switch (g_car_state) {
    case CAR_STATE_TRACK:
        if (g_state_ticks < STATE_SWITCH_MIN_TICKS) {
            g_state_confirm_count = 0U;
        } else if (Gray_LeftTurnDetected(digital) != 0U) {
            Car_EnterAdvanceBeforeTurn(CAR_STATE_TURN_LEFT);
        } else if (Gray_RightTurnDetected(digital) != 0U) {
            Car_EnterAdvanceBeforeTurn(CAR_STATE_TURN_RIGHT);
        } else if (black_count == 0U) {
            g_state_confirm_count++;
            if (g_state_confirm_count >= STATE_CONFIRM_TICKS) {
                Car_EnterBlind(IMU_GetYaw());
            }
        } else {
            g_state_confirm_count = 0U;
        }
        break;

    case CAR_STATE_BLIND:
        if ((g_state_ticks >= STATE_SWITCH_MIN_TICKS) &&
            (black_count > 0U)) {
            g_state_confirm_count++;
            if (g_state_confirm_count >= STATE_CONFIRM_TICKS) {
                g_car_state = CAR_STATE_TRACK;
                g_last_gray_error = 0.0f;
                g_gray_integral = 0.0f;
                g_state_ticks = 0U;
                g_state_confirm_count = 0U;
            }
        } else {
            g_state_confirm_count = 0U;
        }
        break;

    case CAR_STATE_ADVANCE_BEFORE_TURN:
    case CAR_STATE_TURN_LEFT:
    case CAR_STATE_TURN_RIGHT:
        break;

    default:
        g_car_state = CAR_STATE_TRACK;
        g_last_gray_error = 0.0f;
        g_gray_integral = 0.0f;
        g_last_gyro_error = 0.0f;
        g_gyro_integral = 0.0f;
        g_state_ticks = 0U;
        g_state_confirm_count = 0U;
        break;
    }
}

int main(void)
{
    int imu_calibration_status;
    uint32_t last_oled_tick = 0U;

    SYSCFG_DL_init();

    MaxiCam_UART_Init();

    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    No_MCU_Ganv_Sensor_Init(&g_gray_sensor, g_gray_white, g_gray_black);

    if (IMU_Init() != 0) {
        Motor_Stop();
        while (1) {
        }
    }
    delay_ms(2000);
    imu_calibration_status = IMU_Calibrate();
    /* 校准质量不足仍启动业务；传感器读取失败则保持停车。 */
    if ((imu_calibration_status != IMU_CALIBRATION_OK) &&
        (imu_calibration_status != IMU_CALIBRATION_QUALITY_WARNING)) {
        Motor_Stop();
        while (1) {
        }
    }
    IMU_DRDY_Init();
    IMU_UART_Init();
    (void)IMU_HostTxInit(IMU_UART_Write);

    OLED_Init();
    OLED_Clear();
    OLED_DrawSensorLabels();

    DL_TimerA_startCounter(CONTROL_TIMER_INST);
    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);

    while (1) {
        MaxiCam_UART_Process();
        IMU_DRDY_Process();

        if (g_10ms_flag != 0U) {
            g_10ms_flag = 0U;

            No_Mcu_Ganv_Sensor_Task_Without_tick(&g_gray_sensor);
            g_gray_digital = Get_Digtal_For_User(&g_gray_sensor);
            (void)Get_Normalize_For_User(&g_gray_sensor, g_gray_normal);

            Car_UpdateState(g_gray_digital);

            switch (g_car_state) {
            case CAR_STATE_TRACK:
                {
                    static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
                    int16_t weighted_sum = 0;
                    uint8_t line_black_count = 0U;
                    float gray_error;
                    float gray_derivative;
                    float gray_output;
                    float correction;
                    int32_t left_speed;
                    int32_t right_speed;

                    for (uint8_t i = 0U; i < 8U; i++) {
                        if (((g_gray_digital >> i) & 0x01U) == 0U) {
                            weighted_sum += weights[i];
                            line_black_count++;
                        }
                    }

                    if (line_black_count == 0U) {
                        gray_error = g_last_gray_error;
                    } else {
                        gray_error = (float)weighted_sum /
                            (float)line_black_count;
                    }

                    gray_derivative = gray_error - g_last_gray_error;
                    g_gray_integral += gray_error;
                    if (g_gray_integral > 100.0f) {
                        g_gray_integral = 100.0f;
                    } else if (g_gray_integral < -100.0f) {
                        g_gray_integral = -100.0f;
                    }

                    gray_output = GRAY_PID_KP * gray_error +
                        GRAY_PID_KI * g_gray_integral +
                        GRAY_PID_KD * gray_derivative;
                    if (gray_output > (float)MOTOR_BASE_SPEED) {
                        gray_output = (float)MOTOR_BASE_SPEED;
                    } else if (gray_output < -(float)MOTOR_BASE_SPEED) {
                        gray_output = -(float)MOTOR_BASE_SPEED;
                    }

                    g_last_gray_error = gray_error;
                    correction = -gray_output;
                    left_speed = (int32_t)MOTOR_BASE_SPEED +
                        (int32_t)correction;
                    right_speed = (int32_t)MOTOR_BASE_SPEED -
                        (int32_t)correction;

                    if (left_speed < 0) {
                        left_speed = 0;
                    } else if (left_speed > MOTOR_MAX_SPEED) {
                        left_speed = MOTOR_MAX_SPEED;
                    }
                    if (right_speed < 0) {
                        right_speed = 0;
                    } else if (right_speed > MOTOR_MAX_SPEED) {
                        right_speed = MOTOR_MAX_SPEED;
                    }

                    Motor_Control((int16_t)-left_speed, (int16_t)right_speed);
                }
                break;

            case CAR_STATE_BLIND:
                Motor_Control(
                    (int16_t)-(BLIND_BASE_SPEED + BLIND_LEFT_COMPENSATION),
                    (int16_t)BLIND_BASE_SPEED);
                break;

            case CAR_STATE_ADVANCE_BEFORE_TURN:
                Car_RunAdvanceBeforeTurn();
                break;

            case CAR_STATE_TURN_LEFT:
            case CAR_STATE_TURN_RIGHT:
                Car_RunTurn(g_car_state);
                break;

            default:
                Motor_Stop();
                break;
            }

            if ((g_10ms_ticks - last_oled_tick) >=
                (OLED_REFRESH_PERIOD_MS / SENSOR_UPDATE_PERIOD_MS)) {
                last_oled_tick = g_10ms_ticks;
                OLED_UpdateSensorValues((int32_t)IMU_GetYaw(),
                    (int32_t)IMU_GetGyroZ(), g_gray_digital,
                    Car_GetStateName(g_car_state),
                    g_gray_normal);
            }
        }

        (void)IMU_HostTxProcess();
    }
}
void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST) == DL_TIMER_IIDX_ZERO) {
        DL_TimerA_clearInterruptStatus(CONTROL_TIMER_INST, DL_TIMER_IIDX_ZERO);
        g_10ms_ticks++;
        g_10ms_flag = 1U;
        IMU_HostTimerTick10ms();
    }
}
#endif

#if 0
/* MaxiCam UART and OLED test retained for later use. */
#include "ti_msp_dl_config.h"
#include "maxicam_uart.h"
#include "oled_hardware_i2c.h"
#include "uart.h"

int main(void)
{
    MaxiCam_Point point;

    SYSCFG_DL_init();
    MaxiCam_UART_Init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"MAXICAM DATA", 16U);
    OLED_ShowString(0U, 2U, (uint8_t *)"X:", 16U);
    OLED_ShowString(0U, 4U, (uint8_t *)"Y:", 16U);

    while (1) {
        MaxiCam_UART_Process();

        if (MaxiCam_UART_GetLatestPoint(&point)) {
            UART_Printf("(x:%u y:%u)\r\n",
                (unsigned int)point.x,
                (unsigned int)point.y);

            OLED_ShowNum(24U, 2U, point.x, 4U, 16U);
            OLED_ShowNum(24U, 4U, point.y, 4U, 16U);
        }
    }
}
#endif

#include "ti_msp_dl_config.h"
#include "test.h"

#if 0
/* 原蓝牙OLED测试主程序暂时停用，以便独立验证SPI彩屏。 */
int main(void)
{
    SYSCFG_DL_init();
    Test_BluetoothOLED_Init();

    while (1) {
        Test_BluetoothOLED_Process();
    }
}
#endif

/* 该主函数仅运行ST7735S红绿蓝白四色循环测试。 */
int main(void)
{
    SYSCFG_DL_init();

    if (!Test_ST7735S_ColorCycleInit()) {
        while (1) {
        }
    }

    while (1) {
        Test_ST7735S_ColorCycleProcess();
    }
}
