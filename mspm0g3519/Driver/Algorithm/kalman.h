#ifndef __KALMAN_H
#define __KALMAN_H


#include <math.h>
#include <stdint.h>
#include <stddef.h>   


typedef struct{
    float Q_angle;   //角度噪声协方差
    float Q_gyro;    //漂移噪声协方差
    float R_angle;   //测量噪声协方差
    float new_angle; //测得最新角速度
    float angle;      //最新角度
    float bias;      //初始偏置
    float P[2][2];
}KalmanFilter_t;

typedef struct {
    int16_t Acc_X;
    int16_t Acc_Y;
    int16_t Acc_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    int16_t Temp_raw;
    float yaw;
    float roll;
    float pitch;
    float temp;
} MPU6050;

KalmanFilter_t yawKF;

void KalmanFilter_Init(KalmanFilter_t*KF, float Q_angle,float Q_gyro,float R_angle,float new_angle,float bias,float P[2][2]);
float KalmanFilter_Calculate(KalmanFilter_t*KF,float angle,float newGyro,float dt);

#endif