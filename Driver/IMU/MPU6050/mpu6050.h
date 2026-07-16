#ifndef __MPU6050_H
#define __MPU6050_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <math.h>
#include <stdlib.h>


void MPU6050_Calibration(void);

/* ============================================================
 * 函数声明
 * ============================================================ */

/**
 * @brief 初始化 MPU6050
 * @note  包含复位、唤醒、配置低通滤波和量程，并执行零点校准
 * 上电调用时，车子必须保持绝对静止！
 */
void MPU6050_Init(void);

/**
 * @brief 更新 Yaw 角度 (核心算法)
 * @note  必须在定时器中断里周期调用
 * @param dt_ms 定时器的时间间隔 (毫秒)，例如 5ms 定时器传 5.0f
 */
void MPU6050_Update_Yaw(float dt_ms);

/**
 * @brief 获取当前的 Yaw (航向角)
 * @return 角度值 (浮点数)
 */
float MPU_Get_Yaw(void);

/**
 * @brief 重置 Yaw 角度为 0
 * @note  通常在进入新的直线段 (如从 C 点出来准备跑 C->D) 时调用
 */
void MPU_Reset_Yaw(void);

/**
 * @brief 获取当前 Z 轴角速度 (用于调试)
 * @return角速度 dps (度/秒)
 */
float MPU_Get_Gyro_Z(void);

#endif