#ifndef HC_SR04_H
#define HC_SR04_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 该枚举用于表示HC-SR04最近一次测量的工作状态。 */
typedef enum {
    HC_SR04_STATUS_WAITING = 0,
    HC_SR04_STATUS_VALID,
    HC_SR04_STATUS_TIMEOUT,
    HC_SR04_STATUS_OUT_OF_RANGE
} HC_SR04_Status;

/* 该函数初始化HC-SR04的软件状态、触发引脚和捕获中断。 */
void HC_SR04_Init(void);

/* 该函数每10毫秒调度一次非阻塞测距并处理捕获结果或超时。 */
void HC_SR04_Process10ms(void);

/* 该函数读取尚未消费的最新测量结果并返回本次是否取得新结果。 */
bool HC_SR04_TakeResult(uint32_t *distance_mm, HC_SR04_Status *status);

#ifdef __cplusplus
}
#endif

#endif
