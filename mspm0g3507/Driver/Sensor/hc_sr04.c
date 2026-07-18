#include "hc_sr04.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define HC_SR04_TRIGGER_INTERVAL_TICKS    (6U)
#define HC_SR04_ECHO_TIMEOUT_TICKS        (3U)
#define HC_SR04_TRIGGER_PULSE_US          (12U)
#define HC_SR04_MIN_PULSE_US              (116U)
#define HC_SR04_MAX_PULSE_US              (26100U)
#define HC_SR04_FILTER_SIZE               (5U)

typedef enum {
    HC_SR04_STATE_IDLE = 0,
    HC_SR04_STATE_WAIT_ECHO,
    HC_SR04_STATE_CAPTURED
} HC_SR04_State;

static volatile HC_SR04_State s_state = HC_SR04_STATE_IDLE;
static volatile uint32_t s_captured_pulse_us = 0U;
static uint32_t s_filter_values[HC_SR04_FILTER_SIZE] = {0U};
static uint8_t s_filter_count = 0U;
static uint8_t s_filter_index = 0U;
static uint8_t s_trigger_interval_ticks = HC_SR04_TRIGGER_INTERVAL_TICKS;
static uint8_t s_echo_wait_ticks = 0U;
static uint32_t s_latest_distance_mm = 0U;
static HC_SR04_Status s_latest_status = HC_SR04_STATUS_WAITING;
static bool s_result_pending = false;

/* 该函数发送一个12微秒高电平并启动一次HC-SR04测量。 */
static void HC_SR04_StartMeasurement(void)
{
    s_echo_wait_ticks = 0U;
    s_state = HC_SR04_STATE_WAIT_ECHO;

    DL_TimerG_setTimerCount(
        HC_SR04_CAPTURE_INST, HC_SR04_CAPTURE_INST_LOAD_VALUE);
    DL_TimerG_clearInterruptStatus(
        HC_SR04_CAPTURE_INST, DL_TIMERG_INTERRUPT_CC1_DN_EVENT);

    DL_GPIO_clearPins(HC_SR04_TRIG_PORT, HC_SR04_TRIG_TRIG_PIN);
    delay_us(2U);
    DL_GPIO_setPins(HC_SR04_TRIG_PORT, HC_SR04_TRIG_TRIG_PIN);
    delay_us(HC_SR04_TRIGGER_PULSE_US);
    DL_GPIO_clearPins(HC_SR04_TRIG_PORT, HC_SR04_TRIG_TRIG_PIN);
}

/* 该函数对当前有效样本执行中值滤波并返回毫米距离。 */
static uint32_t HC_SR04_FilterDistance(uint32_t distance_mm)
{
    uint32_t sorted[HC_SR04_FILTER_SIZE];
    uint8_t count;
    uint8_t i;
    uint8_t j;

    s_filter_values[s_filter_index] = distance_mm;
    s_filter_index = (uint8_t)((s_filter_index + 1U) % HC_SR04_FILTER_SIZE);
    if (s_filter_count < HC_SR04_FILTER_SIZE) {
        s_filter_count++;
    }

    count = s_filter_count;
    for (i = 0U; i < count; i++) {
        sorted[i] = s_filter_values[i];
    }
    for (i = 1U; i < count; i++) {
        uint32_t value = sorted[i];
        j = i;
        while ((j > 0U) && (sorted[j - 1U] > value)) {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }

    return sorted[count / 2U];
}

/* 该函数把捕获到的回波脉宽转换为距离并发布测量状态。 */
static void HC_SR04_ProcessCapturedPulse(void)
{
    uint32_t pulse_us = s_captured_pulse_us;

    if ((pulse_us < HC_SR04_MIN_PULSE_US) ||
        (pulse_us > HC_SR04_MAX_PULSE_US)) {
        s_latest_status = HC_SR04_STATUS_OUT_OF_RANGE;
    } else {
        uint32_t distance_mm = (pulse_us * 10U + 29U) / 58U;
        s_latest_distance_mm = HC_SR04_FilterDistance(distance_mm);
        s_latest_status = HC_SR04_STATUS_VALID;
    }

    s_result_pending = true;
    s_trigger_interval_ticks = 0U;
    s_state = HC_SR04_STATE_IDLE;
}

/* 该函数初始化HC-SR04的软件状态、触发引脚和捕获中断。 */
void HC_SR04_Init(void)
{
    uint8_t i;

    DL_GPIO_clearPins(HC_SR04_TRIG_PORT, HC_SR04_TRIG_TRIG_PIN);
    DL_TimerG_setTimerCount(
        HC_SR04_CAPTURE_INST, HC_SR04_CAPTURE_INST_LOAD_VALUE);
    DL_TimerG_clearInterruptStatus(
        HC_SR04_CAPTURE_INST, DL_TIMERG_INTERRUPT_CC1_DN_EVENT);
    NVIC_ClearPendingIRQ(HC_SR04_CAPTURE_INST_INT_IRQN);
    NVIC_EnableIRQ(HC_SR04_CAPTURE_INST_INT_IRQN);

    s_state = HC_SR04_STATE_IDLE;
    s_captured_pulse_us = 0U;
    s_filter_count = 0U;
    s_filter_index = 0U;
    for (i = 0U; i < HC_SR04_FILTER_SIZE; i++) {
        s_filter_values[i] = 0U;
    }
    s_trigger_interval_ticks = HC_SR04_TRIGGER_INTERVAL_TICKS;
    s_echo_wait_ticks = 0U;
    s_latest_distance_mm = 0U;
    s_latest_status = HC_SR04_STATUS_WAITING;
    s_result_pending = false;
}

/* 该函数每10毫秒调度一次非阻塞测距并处理捕获结果或超时。 */
void HC_SR04_Process10ms(void)
{
    if (s_state == HC_SR04_STATE_CAPTURED) {
        HC_SR04_ProcessCapturedPulse();
        return;
    }

    if (s_state == HC_SR04_STATE_WAIT_ECHO) {
        s_echo_wait_ticks++;
        if (s_echo_wait_ticks >= HC_SR04_ECHO_TIMEOUT_TICKS) {
            s_latest_status = HC_SR04_STATUS_TIMEOUT;
            s_result_pending = true;
            s_trigger_interval_ticks = 0U;
            s_state = HC_SR04_STATE_IDLE;
        }
        return;
    }

    if (s_trigger_interval_ticks < HC_SR04_TRIGGER_INTERVAL_TICKS) {
        s_trigger_interval_ticks++;
    }
    if (s_trigger_interval_ticks >= HC_SR04_TRIGGER_INTERVAL_TICKS) {
        HC_SR04_StartMeasurement();
    }
}

/* 该函数读取尚未消费的最新测量结果并返回本次是否取得新结果。 */
bool HC_SR04_TakeResult(uint32_t *distance_mm, HC_SR04_Status *status)
{
    if (!s_result_pending) {
        return false;
    }

    if (distance_mm != NULL) {
        *distance_mm = s_latest_distance_mm;
    }
    if (status != NULL) {
        *status = s_latest_status;
    }
    s_result_pending = false;
    return true;
}

/* 该函数在ECHO下降沿中断中读取上升沿和下降沿时间并保存脉宽。 */
void HC_SR04_CAPTURE_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(HC_SR04_CAPTURE_INST)) {
    case DL_TIMERG_IIDX_CC1_DN:
        if (s_state == HC_SR04_STATE_WAIT_ECHO) {
            uint32_t rising_count = DL_TimerG_getCaptureCompareValue(
                HC_SR04_CAPTURE_INST, DL_TIMER_CC_0_INDEX);
            uint32_t falling_count = DL_TimerG_getCaptureCompareValue(
                HC_SR04_CAPTURE_INST, DL_TIMER_CC_1_INDEX);

            if (rising_count >= falling_count) {
                s_captured_pulse_us = rising_count - falling_count;
            } else {
                s_captured_pulse_us = rising_count +
                    (HC_SR04_CAPTURE_INST_LOAD_VALUE + 1U) - falling_count;
            }
            s_state = HC_SR04_STATE_CAPTURED;
        }
        break;

    default:
        break;
    }
}
