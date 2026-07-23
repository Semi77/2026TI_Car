#ifndef VOFA_FIREWATER_H
#define VOFA_FIREWATER_H

#include <stdint.h>

/* 该函数发送FireWater整数数据帧，prefix参数指定帧前缀，channels参数指向通道数组，channel_count参数指定通道数量。 */
void VOFA_FireWater_SendS32(
    const char *prefix,
    const int32_t *channels,
    uint8_t channel_count);

/* 该函数发送两个整数通道，prefix参数指定帧前缀，ch0和ch1参数指定通道数据。 */
void VOFA_FireWater_Send2S32(
    const char *prefix,
    int32_t ch0,
    int32_t ch1);

/* 该函数发送四个整数通道，prefix参数指定帧前缀，ch0至ch3参数指定通道数据。 */
void VOFA_FireWater_Send4S32(
    const char *prefix,
    int32_t ch0,
    int32_t ch1,
    int32_t ch2,
    int32_t ch3);

/* 该函数发送六个整数通道，prefix参数指定帧前缀，ch0至ch5参数指定通道数据。 */
void VOFA_FireWater_Send6S32(
    const char *prefix,
    int32_t ch0,
    int32_t ch1,
    int32_t ch2,
    int32_t ch3,
    int32_t ch4,
    int32_t ch5);

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
    int32_t ch7);

/* 该函数发送PID调试帧，target、feedback、error和output参数依次指定目标值、反馈值、误差值和输出值。 */
void VOFA_FireWater_SendPID(
    int32_t target,
    int32_t feedback,
    int32_t error,
    int32_t output);

#endif
