#include "ti_msp_dl_config.h"
#include "Display/oled_hardware_i2c.h"
#include "Display/st7735s.h"
#include "BSP/key_input.h"
#include "delay.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "hc_sr04.h"
#include "motor.h"
#include "maxicam_uart.h"
#include "Driver/IMU/uart_gyro/uart_gyro.h"

#include <stdint.h>

#define SENSOR_UPDATE_PERIOD_MS  10U
#define OLED_REFRESH_PERIOD_MS   100U
#define MOTOR_BASE_SPEED         400
#define MOTOR_MAX_SPEED          800
#define GRAY_PID_KP              30.0f
#define GRAY_PID_KI              0.0f
#define GRAY_PID_KD              5.0f
#define BLIND_BASE_SPEED         280
#define BLIND_LEFT_COMPENSATION  20
#define TURN_SPEED               180
#define TURN_ANGLE_DEGREES       83.0f
#define TURN_ANGLE_TOLERANCE     5.0f
#define TURN_ADVANCE_TIME_MS     800U
#define STATE_SWITCH_MIN_TICKS   100U
#define STATE_CONFIRM_TICKS      3U
#define GYRO_FIRST_SAMPLE_TIMEOUT_TICKS 200U

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
static float g_latest_yaw = 0.0f;
static float g_latest_gyro_z = 0.0f;

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

/* 该函数从串口陀螺仪驱动拉取最新角度和角速度并缓存。 */
static void Gyro_UpdateLatest(void)
{
    float value;

    if (UARTGyro_GetYaw(&value)) {
        g_latest_yaw = value;
    }
    if (UARTGyro_GetGyroZ(&value)) {
        g_latest_gyro_z = value;
    }
}

/* 该函数等待串口陀螺仪输出第一帧Yaw数据。 */
static uint8_t Gyro_WaitForFirstYaw(void)
{
    float yaw;

    for (uint32_t i = 0U; i < GYRO_FIRST_SAMPLE_TIMEOUT_TICKS; i++) {
        if (UARTGyro_GetYaw(&yaw)) {
            g_latest_yaw = yaw;
            return 1U;
        }
        delay_ms(10U);
    }

    return 0U;
}

/* 该函数返回最近一次缓存的串口陀螺仪Yaw角度。 */
static float Gyro_GetYaw(void)
{
    return g_latest_yaw;
}

/* 该函数返回最近一次缓存的串口陀螺仪Z轴角速度。 */
static float Gyro_GetGyroZ(void)
{
    return g_latest_gyro_z;
}

/* 该函数把角度误差限制到负180度到正180度之间。 */
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

/* 该函数让小车进入丢线后的盲走状态。 */
static void Car_EnterBlind(float target_yaw)
{
    g_car_state = CAR_STATE_BLIND;
    g_pending_turn = CAR_STATE_TURN_LEFT;
    g_blind_target_yaw = target_yaw;
    g_state_ticks = 0U;
    g_state_confirm_count = 0U;
}

/* 该函数让小车进入转弯前的短距离前进状态。 */
static void Car_EnterAdvanceBeforeTurn(CarState_t turn_state)
{
    g_car_state = CAR_STATE_ADVANCE_BEFORE_TURN;
    g_pending_turn = turn_state;
    g_turn_advance_start_tick = g_10ms_ticks;
    g_state_ticks = 0U;
    g_state_confirm_count = 0U;
}

static void Car_EnterTurn(CarState_t turn_state);

/* 该函数在转弯前保持前进并在到时后切换到转弯状态。 */
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

/* 该函数根据当前Yaw设置左右转的目标角度。 */
static void Car_EnterTurn(CarState_t turn_state)
{
    g_car_state = turn_state;
    g_turn_target_yaw = Gyro_GetYaw();
    if (turn_state == CAR_STATE_TURN_LEFT) {
        g_turn_target_yaw += TURN_ANGLE_DEGREES;
    } else {
        g_turn_target_yaw -= TURN_ANGLE_DEGREES;
    }

    g_state_ticks = 0U;
    g_state_confirm_count = 0U;
}

/* 该函数按照串口陀螺仪Yaw误差执行原地转向。 */
static void Car_RunTurn(CarState_t turn_state)
{
    float yaw_error = Gyro_WrapError(g_turn_target_yaw - Gyro_GetYaw());

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

/* 该函数返回小车状态对应的显示字符串。 */
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

/* 该函数统计灰度传感器数字值中检测到黑线的探头数量。 */
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

/* 该函数判断灰度传感器是否检测到左转特征。 */
static uint8_t Gray_LeftTurnDetected(uint8_t digital)
{
    return (((digital & 0x0FU) == 0U) &&
            ((digital & 0xF0U) != 0U));
}

/* 该函数判断灰度传感器是否检测到右转特征。 */
static uint8_t Gray_RightTurnDetected(uint8_t digital)
{
    return (((digital & 0xF0U) == 0U) &&
            ((digital & 0x0FU) != 0U));
}

/* 该函数根据灰度传感器结果更新小车状态机。 */
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
                Car_EnterBlind(Gyro_GetYaw());
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
        g_state_ticks = 0U;
        g_state_confirm_count = 0U;
        break;
    }
}

/* 该函数按照灰度传感器偏差计算巡线电机速度。 */
static void Car_RunTrack(void)
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
        gray_error = (float)weighted_sum / (float)line_black_count;
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
    left_speed = (int32_t)MOTOR_BASE_SPEED + (int32_t)correction;
    right_speed = (int32_t)MOTOR_BASE_SPEED - (int32_t)correction;

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

/* 该函数根据当前小车状态执行对应的电机控制。 */
static void Car_RunStateMachine(void)
{
    switch (g_car_state) {
    case CAR_STATE_TRACK:
        Car_RunTrack();
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
}

/* 该函数在彩屏上刷新最近一次按键事件。 */
static void Key_DisplayEvent(uint8_t key_event)
{
    if (key_event == KEY_INPUT_NONE) {
        return;
    }

    (void)ST7735S_FillRect(56U, 120U, 48U, 16U, ST7735S_COLOR_BLACK);
    if (key_event == KEY_INPUT_KEY1) {
        (void)ST7735S_DrawString(56U, 120U, "KEY1",
            ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    } else if (key_event == KEY_INPUT_KEY2) {
        (void)ST7735S_DrawString(56U, 120U, "KEY2",
            ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    } else if (key_event == KEY_INPUT_KEY3) {
        (void)ST7735S_DrawString(56U, 120U, "KEY3",
            ST7735S_COLOR_WHITE, ST7735S_COLOR_BLACK);
    }
}

/* 该函数初始化板级外设和小车驱动模块。 */
static uint8_t Board_AppInit(void)
{
    SYSCFG_DL_init();
    MaxiCam_UART_Init();
    UARTGyro_Init();

    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    No_MCU_Ganv_Sensor_Init(&g_gray_sensor, g_gray_white, g_gray_black);

    delay_ms(2000U);
    UARTGyro_CalibrateBias();
    UARTGyro_ResetYaw();
    if (Gyro_WaitForFirstYaw() == 0U) {
        Motor_Stop();
        return 0U;
    }

    KeyInput_Init();
    HC_SR04_Init();

    OLED_Init();
    OLED_Clear();
    OLED_DrawSensorLabels();

    (void)ST7735S_Init(ST7735S_ROTATION_0);
    Ultrasonic_DisplayInit();
    (void)ST7735S_DrawString(16U, 120U, "KEY:",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);

    DL_TimerA_startCounter(CONTROL_TIMER_INST);
    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);

    return 1U;
}

/* 该函数运行使用串口陀螺仪的整车控制主循环。 */
int main(void)
{
    uint32_t last_oled_tick = 0U;
    uint32_t ultrasonic_distance_mm = 0U;
    HC_SR04_Status ultrasonic_status = HC_SR04_STATUS_WAITING;
    uint8_t key_event;

    if (Board_AppInit() == 0U) {
        while (1) {
        }
    }

    while (1) {
        MaxiCam_UART_Process();

        if (g_10ms_flag != 0U) {
            g_10ms_flag = 0U;

            Gyro_UpdateLatest();
            KeyInput_Process10ms();
            HC_SR04_Process10ms();
            (void)HC_SR04_TakeResult(
                &ultrasonic_distance_mm, &ultrasonic_status);

            key_event = KeyInput_GetPressEvent();
            Key_DisplayEvent(key_event);

            No_Mcu_Ganv_Sensor_Task_Without_tick(&g_gray_sensor);
            g_gray_digital = Get_Digtal_For_User(&g_gray_sensor);
            (void)Get_Normalize_For_User(&g_gray_sensor, g_gray_normal);

            Car_UpdateState(g_gray_digital);
            Car_RunStateMachine();

            if ((g_10ms_ticks - last_oled_tick) >=
                (OLED_REFRESH_PERIOD_MS / SENSOR_UPDATE_PERIOD_MS)) {
                last_oled_tick = g_10ms_ticks;
                OLED_UpdateSensorValues((int32_t)Gyro_GetYaw(),
                    (int32_t)Gyro_GetGyroZ(), g_gray_digital,
                    Car_GetStateName(g_car_state),
                    g_gray_normal);
                Ultrasonic_DisplayUpdate(
                    ultrasonic_distance_mm, ultrasonic_status);
            }
        }
    }
}

/* 该函数处理10毫秒控制定时器中断并唤醒主循环任务。 */
void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST) ==
        DL_TIMER_IIDX_ZERO) {
        DL_TimerA_clearInterruptStatus(CONTROL_TIMER_INST,
            DL_TIMER_IIDX_ZERO);
        g_10ms_ticks++;
        g_10ms_flag = 1U;
    }
}
