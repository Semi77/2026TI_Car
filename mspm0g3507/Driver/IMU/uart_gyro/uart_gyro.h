#ifndef UART_GYRO_H
#define UART_GYRO_H

#include <stdbool.h>

/* 该函数初始化串口陀螺仪的接收状态和UART中断。 */
void UARTGyro_Init(void);

/* 该函数在收到新的有效角度帧时返回最新Yaw角度。 */
bool UARTGyro_GetYaw(float *yaw_deg);

/* 该函数在收到新的有效角速度帧时返回最新Z轴角速度。 */
bool UARTGyro_GetGyroZ(float *gyro_z_dps);

/* 该函数发送命令将模块当前Yaw角度设置为零并保存。 */
void UARTGyro_ResetYaw(void);

/* 该函数执行零偏校准、读取校准状态并返回是否成功。 */
bool UARTGyro_CalibrateBias(void);

#endif
