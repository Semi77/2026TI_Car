#include "vofa_firewater.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

/* 该函数阻塞等待UART_TEST发送FIFO可用后发送data参数指定的一个字节。 */
static void VOFA_FireWater_SendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_TEST_INST)) {
    }
    DL_UART_Main_transmitData(UART_TEST_INST, data);
}

/* 该函数发送text参数指定的以空字符结尾的字符串。 */
static void VOFA_FireWater_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        VOFA_FireWater_SendByte((uint8_t)*text);
        text++;
    }
}

/* 该函数将value参数指定的无符号整数转换为十进制字符并发送。 */
static void VOFA_FireWater_SendU32(uint32_t value)
{
    char buffer[10];
    uint8_t index = 0U;

    if (value == 0U) {
        VOFA_FireWater_SendByte((uint8_t)'0');
        return;
    }

    while (value > 0U) {
        buffer[index] = (char)('0' + (value % 10U));
        index++;
        value /= 10U;
    }

    while (index > 0U) {
        index--;
        VOFA_FireWater_SendByte((uint8_t)buffer[index]);
    }
}

/* 该函数将value参数指定的有符号整数转换为十进制字符并发送。 */
static void VOFA_FireWater_SendOneS32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        VOFA_FireWater_SendByte((uint8_t)'-');
        magnitude = 0U - (uint32_t)value;
    } else {
        magnitude = (uint32_t)value;
    }
    VOFA_FireWater_SendU32(magnitude);
}

/* 该函数发送FireWater整数数据帧，prefix参数指定帧前缀，channels参数指向通道数组，channel_count参数指定通道数量。 */
void VOFA_FireWater_SendS32(
    const char *prefix,
    const int32_t *channels,
    uint8_t channel_count)
{
    uint8_t index;

    if ((channels == NULL) || (channel_count == 0U)) {
        return;
    }

    if ((prefix != NULL) && (*prefix != '\0')) {
        VOFA_FireWater_SendString(prefix);
        VOFA_FireWater_SendByte((uint8_t)':');
    }

    for (index = 0U; index < channel_count; index++) {
        if (index > 0U) {
            VOFA_FireWater_SendByte((uint8_t)',');
        }
        VOFA_FireWater_SendOneS32(channels[index]);
    }
    VOFA_FireWater_SendByte((uint8_t)'\n');
}

/* 该函数发送两个整数通道，prefix参数指定帧前缀，ch0和ch1参数指定通道数据。 */
void VOFA_FireWater_Send2S32(
    const char *prefix,
    int32_t ch0,
    int32_t ch1)
{
    const int32_t channels[2] = {ch0, ch1};

    VOFA_FireWater_SendS32(prefix, channels, 2U);
}

/* 该函数发送四个整数通道，prefix参数指定帧前缀，ch0至ch3参数指定通道数据。 */
void VOFA_FireWater_Send4S32(
    const char *prefix,
    int32_t ch0,
    int32_t ch1,
    int32_t ch2,
    int32_t ch3)
{
    const int32_t channels[4] = {ch0, ch1, ch2, ch3};

    VOFA_FireWater_SendS32(prefix, channels, 4U);
}

/* 该函数发送六个整数通道，prefix参数指定帧前缀，ch0至ch5参数指定通道数据。 */
void VOFA_FireWater_Send6S32(
    const char *prefix,
    int32_t ch0,
    int32_t ch1,
    int32_t ch2,
    int32_t ch3,
    int32_t ch4,
    int32_t ch5)
{
    const int32_t channels[6] = {ch0, ch1, ch2, ch3, ch4, ch5};

    VOFA_FireWater_SendS32(prefix, channels, 6U);
}

/* 该函数发送八个整数通道，prefix参数指定帧前缀，ch0至ch7参数指定通道数据。 */
void VOFA_FireWater_Send8S32(
    const char *prefix,
    int32_t ch0,
    int32_t ch1,
    int32_t ch2,
    int32_t ch3,
    int32_t ch4,
    int32_t ch5,
    int32_t ch6,
    int32_t ch7)
{
    const int32_t channels[8] = {
        ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7
    };

    VOFA_FireWater_SendS32(prefix, channels, 8U);
}

/* 该函数发送PID调试帧，target、feedback、error和output参数依次指定目标值、反馈值、误差值和输出值。 */
void VOFA_FireWater_SendPID(
    int32_t target,
    int32_t feedback,
    int32_t error,
    int32_t output)
{
    VOFA_FireWater_Send4S32("pid", target, feedback, error, output);
}
