/* 该编译块启用串口陀螺仪与SPI屏幕联合测试入口。 */
#if 0
#include "ti_msp_dl_config.h"
#include "test.h"

/* 该测试入口调用串口陀螺仪与SPI屏幕联合测试。 */
int main(void)
{
    Test_UARTGyroST7735_Run();

    while (1) {
    }
}
#endif

#if 1
/* 该编译块启用整车传感器采集、状态机控制、电机驱动和屏幕状态显示。 */
#include "ti_msp_dl_config.h"
#include "Display/st7735s.h"
#include "BSP/key_input.h"
#include "BSP/vofa_firewater.h"
#include "IMU/uart_gyro/uart_gyro.h"
#include "delay.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "motor.h"
#include "maxicam_uart.h"

#include <stdint.h>

#define SENSOR_UPDATE_PERIOD_MS  10U
#define SPI_REFRESH_PERIOD_MS    100U
#define MOTOR_BASE_SPEED         400
#define MOTOR_MAX_SPEED          800
#define GRAY_PID_KP              30.0f
#define GRAY_PID_KI              0.0f
#define GRAY_PID_KD              5.0f
#define BLIND_BASE_SPEED         280
#define BLIND_LEFT_COMPENSATION  -5
#define GYRO_PID_KP              1.5f
#define GYRO_PID_KI              0.0f
#define GYRO_PID_KD              0.8f
#define GYRO_PID_MAX_OUTPUT      100.0f
#define TURN_SPEED               180
#define TURN_ANGLE_DEGREES       80.0f
#define TURN_ANGLE_TOLERANCE     5.0f
#define TURN_ADVANCE_TIME_MS     800U
#define STATE_SWITCH_MIN_TICKS   50U
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
static float g_last_track_yaw = 0.0f;
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
static float g_uart_yaw_deg = 0.0f;
static float g_uart_gyro_z_dps = 0.0f;
static bool g_uart_yaw_ready = false;

#if 0
/* 以下超声波显示代码已停用，保留以便后续需要时恢复。 */
/* 该函数返回无符号十进制整数在屏幕上需要显示的字符数量。 */
static uint8_t Ultrasonic_GetDigitCount(uint32_t value)
{
    uint8_t count = 1U;

    while (value >= 10U) {
        value /= 10U;
        count++;
    }
    return count;
}

/* 该函数初始化ST7735S上的超声波距离和状态显示区域。 */
static void Ultrasonic_DisplayInit(void)
{
    (void)ST7735S_FillScreen(ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(24U, 8U, "HC-SR04",
        ST7735S_COLOR_CYAN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 40U, "DIST:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(56U, 40U, "---.-cm",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 72U, "STATE:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(64U, 72U, "WAIT",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
}

/* 该函数在ST7735S上局部刷新超声波距离和测量状态。 */
static void Ultrasonic_DisplayUpdate(
    uint32_t distance_mm, HC_SR04_Status status)
{
    uint16_t cursor_x;
    const char *status_text;
    uint16_t status_color;

    (void)ST7735S_FillRect(56U, 40U, 72U, 16U, ST7735S_COLOR_BLACK);
    if (status == HC_SR04_STATUS_VALID) {
        uint32_t distance_cm = distance_mm / 10U;
        uint32_t decimal = distance_mm % 10U;

        (void)ST7735S_DrawInteger(56U, 40U, (int32_t)distance_cm,
            ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
        cursor_x = (uint16_t)(56U +
            (uint16_t)Ultrasonic_GetDigitCount(distance_cm) * 8U);
        (void)ST7735S_DrawChar(cursor_x, 40U, '.',
            ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
        (void)ST7735S_DrawInteger((uint16_t)(cursor_x + 8U), 40U,
            (int32_t)decimal, ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
        (void)ST7735S_DrawString((uint16_t)(cursor_x + 16U), 40U, "cm",
            ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    } else {
        (void)ST7735S_DrawString(56U, 40U, "---.-cm",
            ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    }

    switch (status) {
    case HC_SR04_STATUS_VALID:
        status_text = "OK";
        status_color = ST7735S_COLOR_GREEN;
        break;
    case HC_SR04_STATUS_TIMEOUT:
        status_text = "NO ECHO";
        status_color = ST7735S_COLOR_RED;
        break;
    case HC_SR04_STATUS_OUT_OF_RANGE:
        status_text = "RANGE";
        status_color = ST7735S_COLOR_MAGENTA;
        break;
    case HC_SR04_STATUS_WAITING:
    default:
        status_text = "WAIT";
        status_color = ST7735S_COLOR_WHITE;
        break;
    }

    (void)ST7735S_FillRect(64U, 72U, 64U, 16U, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(64U, 72U, status_text,
        status_color, ST7735S_COLOR_BLACK);
}
#endif

    /* 当前底盘A轮正向、B轮反向时，两轮构成车体向前运动。 */
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

/* 该函数进入盲走状态并锁定参数target_yaw指定的目标航向角。 */
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

/* 该函数在盲走阶段通过航向角PID维持锁存角度并限制左右轮目标速度。 */
static void Car_RunBlind(void)
{
    float yaw_error = Gyro_WrapError(g_blind_target_yaw - g_uart_yaw_deg);
    float derivative = yaw_error - g_last_gyro_error;
    float output;
    int32_t left_speed;
    int32_t right_speed;

    g_gyro_integral += yaw_error;
    if (g_gyro_integral > GYRO_PID_MAX_OUTPUT) {
        g_gyro_integral = GYRO_PID_MAX_OUTPUT;
    } else if (g_gyro_integral < -GYRO_PID_MAX_OUTPUT) {
        g_gyro_integral = -GYRO_PID_MAX_OUTPUT;
    }

    output = GYRO_PID_KP * yaw_error +
        GYRO_PID_KI * g_gyro_integral +
        GYRO_PID_KD * derivative;
    if (output > GYRO_PID_MAX_OUTPUT) {
        output = GYRO_PID_MAX_OUTPUT;
    } else if (output < -GYRO_PID_MAX_OUTPUT) {
        output = -GYRO_PID_MAX_OUTPUT;
    }
    g_last_gyro_error = yaw_error;

    left_speed = BLIND_BASE_SPEED + BLIND_LEFT_COMPENSATION -
        (int32_t)output;
    right_speed = BLIND_BASE_SPEED + (int32_t)output;
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

    Motor_Control((int16_t)left_speed, (int16_t)-right_speed);
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
    g_turn_target_yaw = g_uart_yaw_deg;
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
    float yaw_error = Gyro_WrapError(g_turn_target_yaw - g_uart_yaw_deg);

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
    case CAR_STATE_ADVANCE_BEFORE_TURN:
        return "ADVANCE";
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
    return (((digital & 0xF0U) == 0U) &&
            ((digital & 0x0FU) != 0U));
}

static uint8_t Gray_RightTurnDetected(uint8_t digital)
{
    return (((digital & 0x0FU) == 0U) &&
            ((digital & 0xF0U) != 0U));
}

static void Car_UpdateState(uint8_t digital)
{
    uint8_t black_count = Gray_CountBlack(digital);

    g_state_ticks++;

    switch (g_car_state) {
    case CAR_STATE_TRACK:
        if (black_count > 0U) {
            g_last_track_yaw = g_uart_yaw_deg;
        }
        if (g_state_ticks < STATE_SWITCH_MIN_TICKS) {
            g_state_confirm_count = 0U;
        } else if (Gray_LeftTurnDetected(digital) != 0U) {
            Car_EnterAdvanceBeforeTurn(CAR_STATE_TURN_LEFT);
        } else if (Gray_RightTurnDetected(digital) != 0U) {
            Car_EnterAdvanceBeforeTurn(CAR_STATE_TURN_RIGHT);
        } else if (black_count == 0U) {
            g_state_confirm_count++;
            if (g_state_confirm_count >= STATE_CONFIRM_TICKS) {
                Car_EnterBlind(g_last_track_yaw);
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
    uint32_t last_spi_tick = 0U;
    uint8_t key_event;

    SYSCFG_DL_init();

    MaxiCam_UART_Init();

    /* 该功能块按参考例程顺序初始化灰度传感器并启用ADC中断。 */
    No_MCU_Ganv_Sensor_Init(&g_gray_sensor, g_gray_white, g_gray_black);
    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    delay_ms(100U);
    UARTGyro_Init();

    KeyInput_Init();

    if (!ST7735S_Init(ST7735S_ROTATION_0)) {
        Motor_Stop();
        while (1) {
        }
    }
    (void)ST7735S_FillScreen(ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(16U, 48U, "CALIBRATING",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(24U, 80U, "KEEP STILL",
        ST7735S_COLOR_RED, ST7735S_COLOR_BLACK);
    if (UARTGyro_CalibrateBias()) {
        (void)ST7735S_DrawString(32U, 112U, "CAL OK",
            ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    } else {
        (void)ST7735S_DrawString(24U, 112U, "CAL FAIL",
            ST7735S_COLOR_RED, ST7735S_COLOR_BLACK);
    }
    delay_ms(1000U);
    (void)ST7735S_CarStatusInit();

    DL_TimerA_startCounter(CONTROL_TIMER_INST);
    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);

    while (1) {
        MaxiCam_UART_Process();
        if (UARTGyro_GetYaw(&g_uart_yaw_deg)) {
            g_uart_yaw_ready = true;
        }
        (void)UARTGyro_GetGyroZ(&g_uart_gyro_z_dps);

        if (g_10ms_flag != 0U) {
            g_10ms_flag = 0U;

            KeyInput_Process10ms();

            key_event = KeyInput_GetPressEvent();
            if (key_event != KEY_INPUT_NONE) {
                (void)ST7735S_FillRect(40U, 120U, 72U, 16U,
                    ST7735S_COLOR_BLACK);

                if (key_event == KEY_INPUT_KEY1) {
                    (void)ST7735S_DrawString(40U, 120U, "KEY1",
                        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
                } else if (key_event == KEY_INPUT_KEY2) {
                    (void)ST7735S_DrawString(40U, 120U, "KEY2",
                        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
                } else if (key_event == KEY_INPUT_KEY3) {
                    (void)ST7735S_DrawString(40U, 120U, "KEY3",
                        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
                }
            }
            No_Mcu_Ganv_Sensor_Task_Without_tick(&g_gray_sensor);
            g_gray_digital = Get_Digtal_For_User(&g_gray_sensor);
            (void)Get_Normalize_For_User(&g_gray_sensor, g_gray_normal);

            /* 该功能块在陀螺仪数据就绪后执行灰度巡线、盲区直行和转向控制。 */
            if (!g_uart_yaw_ready) {
                Motor_Stop();
            } else {
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

                    Motor_Control((int16_t)left_speed, (int16_t)-right_speed);
                }
                break;

            case CAR_STATE_BLIND:
                Car_RunBlind();
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
            }

            if ((g_10ms_ticks - last_spi_tick) >=
                (SPI_REFRESH_PERIOD_MS / SENSOR_UPDATE_PERIOD_MS)) {
                const char *display_state = g_uart_yaw_ready ?
                    Car_GetStateName(g_car_state) : "GYRO_WAIT";

                last_spi_tick = g_10ms_ticks;
                /* 该测试功能块每100毫秒通过UART_TEST发送数值1至8的FireWater固定数据帧。 */
                // VOFA_FireWater_Send8S32(
                //      "test", 1, 2, 3, 4, 5, 6, 7, 8);
                (void)ST7735S_CarStatusUpdate(g_uart_yaw_deg,
                    g_uart_gyro_z_dps, g_gray_digital, display_state);
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
#endif

#if 0
#include "ti_msp_dl_config.h"
#include "Display/st7735s.h"
#include "delay.h"
#include "hc_sr04.h"

#include <stdint.h>

/* 该函数返回无符号十进制整数所占用的显示字符数量。 */
static uint8_t UltrasonicTest_GetDigitCount(uint32_t value)
{
    uint8_t count = 1U;

    while (value >= 10U) {
        value /= 10U;
        count++;
    }
    return count;
}

/* 该函数初始化ST7735S超声波测试界面并显示等待状态。 */
static void UltrasonicTest_DisplayInit(void)
{
    (void)ST7735S_FillScreen(ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(16U, 24U, "HC-SR04 TEST",
        ST7735S_COLOR_CYAN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 64U, "DIST:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(56U, 64U, "---.-cm",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
}

/* 该函数在ST7735S上局部刷新本次超声波测量距离。 */
static void UltrasonicTest_DisplayDistance(
    uint32_t distance_mm, HC_SR04_Status status)
{
    uint16_t cursor_x;

    (void)ST7735S_FillRect(56U, 64U, 72U, 16U, ST7735S_COLOR_BLACK);
    if (status != HC_SR04_STATUS_VALID) {
        (void)ST7735S_DrawString(56U, 64U, "---.-cm",
            ST7735S_COLOR_RED, ST7735S_COLOR_BLACK);
        return;
    }

    (void)ST7735S_DrawInteger(56U, 64U, (int32_t)(distance_mm / 10U),
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    cursor_x = (uint16_t)(56U +
        (uint16_t)UltrasonicTest_GetDigitCount(distance_mm / 10U) * 8U);
    (void)ST7735S_DrawChar(cursor_x, 64U, '.',
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawInteger((uint16_t)(cursor_x + 8U), 64U,
        (int32_t)(distance_mm % 10U),
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString((uint16_t)(cursor_x + 16U), 64U, "cm",
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
}

/* 该函数初始化板级外设、HC-SR04和彩屏后持续测距并刷新距离。 */
int main(void)
{
    uint32_t distance_mm = 0U;
    HC_SR04_Status status = HC_SR04_STATUS_WAITING;

    SYSCFG_DL_init();
    HC_SR04_Init();

    if (!ST7735S_Init(ST7735S_ROTATION_0)) {
        while (1) {
        }
    }
    UltrasonicTest_DisplayInit();

    while (1) {
        HC_SR04_Process10ms();
        if (HC_SR04_TakeResult(&distance_mm, &status)) {
            UltrasonicTest_DisplayDistance(distance_mm, status);
        }
        delay_ms(10U);
    }
}
#endif

#if 0
#include "ti_msp_dl_config.h"
#include "Display/st7735s.h"
#include "IMU/uart_gyro/uart_gyro.h"
#include "delay.h"

#include <stdint.h>

/* 该函数初始化ST7735S串口陀螺仪测试界面。 */
static void UARTGyroTest_DisplayInit(void)
{
    (void)ST7735S_FillScreen(ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(16U, 16U, "UART GYRO",
        ST7735S_COLOR_CYAN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 48U, "YAW:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(48U, 48U, "WAITING",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 80U, "UART0 115200",
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 104U, "RX:PA31 TX:PB0",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
}

/* 该函数把Yaw角度转换为百分之一度的有符号整数。 */
static int32_t UARTGyroTest_ScaleYaw(float yaw_deg)
{
    if (yaw_deg >= 0.0f) {
        return (int32_t)(yaw_deg * 100.0f + 0.5f);
    }
    return (int32_t)(yaw_deg * 100.0f - 0.5f);
}

/* 该函数返回无符号十进制整数在屏幕上占用的字符数量。 */
static uint8_t UARTGyroTest_GetDigitCount(uint32_t value)
{
    uint8_t count = 1U;

    while (value >= 10U) {
        value /= 10U;
        count++;
    }
    return count;
}

/* 该函数在ST7735S上显示带两位小数的Yaw角度。 */
static void UARTGyroTest_DisplayYaw(float yaw_deg)
{
    int32_t scaled_yaw = UARTGyroTest_ScaleYaw(yaw_deg);
    uint32_t magnitude;
    uint32_t integer_part;
    uint16_t cursor_x = 48U;

    (void)ST7735S_FillRect(48U, 48U, 80U, 16U,
        ST7735S_COLOR_BLACK);

    if (scaled_yaw < 0) {
        magnitude = (uint32_t)(-scaled_yaw);
        (void)ST7735S_DrawChar(cursor_x, 48U, '-',
            ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
        cursor_x = (uint16_t)(cursor_x + 8U);
    } else {
        magnitude = (uint32_t)scaled_yaw;
    }

    integer_part = magnitude / 100U;
    (void)ST7735S_DrawInteger(cursor_x, 48U, (int32_t)integer_part,
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    cursor_x = (uint16_t)(cursor_x +
        (uint16_t)UARTGyroTest_GetDigitCount(integer_part) * 8U);
    (void)ST7735S_DrawChar(cursor_x, 48U, '.',
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawInteger((uint16_t)(cursor_x + 8U), 48U,
        (int32_t)((magnitude / 10U) % 10U),
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawInteger((uint16_t)(cursor_x + 16U), 48U,
        (int32_t)(magnitude % 10U),
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString((uint16_t)(cursor_x + 24U), 48U, "deg",
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
}

/* 该函数初始化板级外设、串口陀螺仪和彩屏并持续显示Yaw角度。 */
int main(void)
{
    float yaw_deg = 0.0f;

    SYSCFG_DL_init();
    UARTGyro_Init();


    if (!ST7735S_Init(ST7735S_ROTATION_0)) {
        while (1) {
        }
    }
    UARTGyroTest_DisplayInit();

    (void)ST7735S_FillRect(48U, 48U, 80U, 16U,
        ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(48U, 48U, "CAL 21S",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 128U, "KEEP STILL",
        ST7735S_COLOR_RED, ST7735S_COLOR_BLACK);
    UARTGyro_CalibrateBias();
    (void)ST7735S_FillRect(48U, 48U, 80U, 16U,
        ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(48U, 48U, "CAL DONE",
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    (void)ST7735S_FillRect(8U, 128U, 96U, 16U,
        ST7735S_COLOR_BLACK);

    while (1) {
        if (UARTGyro_GetYaw(&yaw_deg)) {
            UARTGyroTest_DisplayYaw(yaw_deg);
        }
        delay_ms(20U);
    }
}
#endif

#if 0
#include "ti_msp_dl_config.h"
#include "Display/st7735s.h"
#include "BSP/maxicam_uart.h"
#include "delay.h"

#include <stdint.h>

/* 该函数返回无符号整数在屏幕上显示所需的字符数量。 */
static uint8_t MaxiCamKey_GetDigitCount(uint32_t value)
{
    uint8_t count = 1U;

    while (value >= 10U) {
        value /= 10U;
        count++;
    }
    return count;
}

/* 该函数初始化按键控制MaixCAM的彩屏提示界面。 */
static void MaxiCamKey_DisplayInit(void)
{
    (void)ST7735S_FillScreen(ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 8U, "MAXICAM KEY",
        ST7735S_COLOR_CYAN, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 36U, "PB21 KEY",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 56U, "PRESS:NEXT",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 76U, "RECT/CIRCLE",
        ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 92U, "KEY:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 104U, "CMD:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(8U, 128U, "P:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
}

/* 该函数在屏幕上显示最近一次按键发送给MaixCAM的命令。 */
static void MaxiCamKey_DisplayCommand(const char *command)
{
    (void)ST7735S_FillRect(48U, 104U, 80U, 16U,
        ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(48U, 104U, command,
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
}

/* 该函数直接读取PB21功能按键是否处于按下状态。 */
static uint8_t MaxiCamKey_IsPressed(void)
{
    return ((DL_GPIO_readPins(KEY_INPUT_B_PORT, KEY_INPUT_B_KEY3_PIN) &
        KEY_INPUT_B_KEY3_PIN) == 0U) ? 1U : 0U;
}

/* 该函数在屏幕上显示PB21功能按键当前的原始状态。 */
static void MaxiCamKey_DisplayKeyState(uint8_t pressed)
{
    (void)ST7735S_FillRect(48U, 92U, 64U, 16U,
        ST7735S_COLOR_BLACK);
    if (pressed != 0U) {
        (void)ST7735S_DrawString(48U, 92U, "DOWN",
            ST7735S_COLOR_RED, ST7735S_COLOR_BLACK);
    } else {
        (void)ST7735S_DrawString(48U, 92U, "UP",
            ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    }
}

/* 该函数在屏幕上显示MaixCAM返回的最新坐标点。 */
static void MaxiCamKey_DisplayPoint(const MaxiCam_Point *point)
{
    uint16_t cursor_x = 28U;

    if (point == NULL) {
        return;
    }

    (void)ST7735S_FillRect(28U, 128U, 100U, 16U,
        ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawInteger(cursor_x, 128U, (int32_t)point->x,
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    cursor_x = (uint16_t)(cursor_x +
        (uint16_t)MaxiCamKey_GetDigitCount(point->x) * 8U);
    (void)ST7735S_DrawChar(cursor_x, 128U, ',',
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
    cursor_x = (uint16_t)(cursor_x + 8U);
    (void)ST7735S_DrawInteger(cursor_x, 128U, (int32_t)point->y,
        ST7735S_COLOR_GREEN, ST7735S_COLOR_BLACK);
}

/* 该函数根据按键事件向MaixCAM发送对应的识别模式命令。 */
static void MaxiCamKey_HandlePress(void)
{
    MaxiCamKey_DisplayCommand("NEXT");
    MaxiCam_UART_SendCommand("NEXT");
}

/* 该函数初始化板级外设并循环扫描按键来控制MaixCAM。 */
int main(void)
{
    uint8_t key_pressed;
    uint8_t last_key_pressed = 0U;
    MaxiCam_Point point;

    SYSCFG_DL_init();
    MaxiCam_UART_Init();

    if (!ST7735S_Init(ST7735S_ROTATION_0)) {
        while (1) {
        }
    }
    MaxiCamKey_DisplayInit();

    while (1) {
        MaxiCam_UART_Process();

        if (MaxiCam_UART_GetLatestPoint(&point)) {
            MaxiCamKey_DisplayPoint(&point);
        }

        key_pressed = MaxiCamKey_IsPressed();
        MaxiCamKey_DisplayKeyState(key_pressed);
        if ((key_pressed != 0U) && (last_key_pressed == 0U)) {
            MaxiCamKey_HandlePress();
        }
        last_key_pressed = key_pressed;

        delay_ms(10U);
    }
}
#endif
