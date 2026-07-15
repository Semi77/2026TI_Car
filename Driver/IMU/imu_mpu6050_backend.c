#include "imu_backend_config.h"

#if IMU_SELECTED_BACKEND == IMU_BACKEND_MPU6050

#include "IMU.h"
#include "mpu6050.h"

#include <stddef.h>
#include <stdint.h>

int IMU_Init(void)
{
    MPU6050_Init();
    return 0;
}

int IMU_ReadData(float accel_mg[3], float gyro_dps[3],
        float *temperature_degc)
{
    if (accel_mg != NULL) {
        accel_mg[0] = 0.0f;
        accel_mg[1] = 0.0f;
        accel_mg[2] = 0.0f;
    }
    if (gyro_dps != NULL) {
        gyro_dps[0] = 0.0f;
        gyro_dps[1] = 0.0f;
        gyro_dps[2] = MPU_Get_Gyro_Z();
    }
    if (temperature_degc != NULL) {
        *temperature_degc = 0.0f;
    }

    return -1;
}

int IMU_Calibrate(void)
{
    MPU6050_Calibration();
    return 0;
}

int IMU_Update(float dt_seconds)
{
    if (dt_seconds <= 0.0f) {
        return -1;
    }

    MPU6050_Update_Yaw(dt_seconds * 1000.0f);
    return 0;
}

void IMU_GetAccel(float accel_mg[3])
{
    if (accel_mg == NULL) {
        return;
    }

    accel_mg[0] = 0.0f;
    accel_mg[1] = 0.0f;
    accel_mg[2] = 0.0f;
}

void IMU_GetGyro(float gyro_dps[3])
{
    if (gyro_dps == NULL) {
        return;
    }

    gyro_dps[0] = 0.0f;
    gyro_dps[1] = 0.0f;
    gyro_dps[2] = MPU_Get_Gyro_Z();
}

void IMU_GetEuler(float angle_deg[3])
{
    if (angle_deg == NULL) {
        return;
    }

    angle_deg[0] = 0.0f;
    angle_deg[1] = 0.0f;
    angle_deg[2] = MPU_Get_Yaw();
}

void IMU_ResetAttitude(void)
{
    MPU_Reset_Yaw();
}

float IMU_GetTemperature(void)
{
    return 0.0f;
}

float IMU_GetGyroZ(void)
{
    return MPU_Get_Gyro_Z();
}

float IMU_GetYaw(void)
{
    return MPU_Get_Yaw();
}

float IMU_GetGyroBiasZ(void)
{
    return 0.0f;
}

float IMU_GetGyroVarianceZ(void)
{
    return 0.0f;
}

float IMU_GetDeadband(void)
{
    return 0.5f;
}

#endif
