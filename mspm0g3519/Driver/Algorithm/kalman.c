#include "kalman.h"

//卡尔曼滤波初始化

void KalmanFilter_Init(KalmanFilter_t*KF, float Q_angle,float Q_gyro,float R_angle,float new_angle,float bias,float P[2][2])
{
    KF->Q_angle = Q_angle; // 角度协方差
    KF->Q_gyro  = Q_gyro;  // 角速度协方差
    KF->R_angle = R_angle; // 观测噪声协方差

    KF->new_angle = new_angle; //最新角度
    KF->bias  = bias;          //偏置噪声

    //如果设置了矩阵数值

    if (P != NULL) {
        KF->P[0][0] = P[0][0];
        KF->P[0][1] = P[0][1];
        KF->P[1][0] = P[1][0];
        KF->P[1][1] = P[1][1];
    } 
    //如果没设置矩阵，填入常用值
    else {
        KF->P[0][0] = 1.0f;
        KF->P[0][1] = 0.0f;
        KF->P[1][0] = 0.0f;
        KF->P[1][1] = 1.0f;
    }
}

// 卡尔曼滤波计算

float KalmanFilter_Calculate(KalmanFilter_t*KF,float angle,float newGyro,float dt)
{
    float gyro = newGyro - KF->bias;   //新角速度减去噪声漂移
    KF->angle += dt * gyro;            //角度 = 前一角度+（去噪声）角速度*时间

    /*协方差先验估计   过程噪声Q=[Q_angle  0
                                    0    Q_gyro] */
    KF->P[0][0] += dt * (dt*KF->P[1][1] - KF->P[0][1] - KF->P[1][0] + KF->Q_angle);
    KF->P[0][1] -= dt * KF->P[1][1];
    KF->P[1][0] -= dt * KF->P[1][1];
    KF->P[1][1] += KF->Q_gyro * dt;    //角速度协方差*时间  

    //卡尔曼增益计算
    float S = KF->P[0][0] + KF->R_angle;
    float K[2];
    K[0] = KF->P[0][0] / S;
    K[1] = KF->P[1][0] / S;

    //状态后验估计
    float y = angle - KF->angle;
    KF->angle += K[0] * y;
    KF->bias += K[1] * y;

    //协方差后验估计
    float P00_temp = KF->P[0][0];
    float P01_temp = KF->P[0][1];

    KF->P[0][0] -= K[0] * P00_temp;
    KF->P[0][1] -= K[0] * P01_temp;
    KF->P[1][0] -= K[1] * P00_temp;
    KF->P[1][1] -= K[1] * P01_temp;

    //返回最优角度
    return KF->angle;
}