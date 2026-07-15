#ifndef ICM45686_DEMO_IMU_H
#define ICM45686_DEMO_IMU_H

/* 质量告警表示校准结果不理想但设备仍可工作；读取错误不可放行。 */
typedef enum {
    IMU_CALIBRATION_OK = 0,
    IMU_CALIBRATION_READ_ERROR = -1,
    IMU_CALIBRATION_QUALITY_WARNING = -2
} IMU_CalibrationStatus;

int IMU_Init(void);
int IMU_ReadData(float accel_mg[3], float gyro_dps[3], float *temperature_degc);
int IMU_Calibrate(void);
int IMU_Update(float dt_seconds);

void IMU_GetAccel(float accel_mg[3]);
void IMU_GetGyro(float gyro_dps[3]);
void IMU_GetEuler(float angle_deg[3]);
void IMU_ResetAttitude(void);

float IMU_GetTemperature(void);
float IMU_GetGyroZ(void);
float IMU_GetYaw(void);
float IMU_GetGyroBiasZ(void);
float IMU_GetGyroVarianceZ(void);
float IMU_GetDeadband(void);

#endif
