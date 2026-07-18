#ifndef MAHONY6D_H
#define MAHONY6D_H

typedef struct {
    float kp;
    float ki;
    float integral_limit;
    float accel_full_confidence_g;
    float accel_reject_g;
} Mahony6D_Config;

typedef struct {
    float q[4];
    float integral_feedback[3];
    Mahony6D_Config config;
} Mahony6D;

void Mahony6D_Init(Mahony6D *filter, const Mahony6D_Config *config);
void Mahony6D_Reset(Mahony6D *filter);
void Mahony6D_SetInitialAttitude(Mahony6D *filter,
    const float accel_mg[3]);

int Mahony6D_Update(Mahony6D *filter, const float accel_mg[3],
    const float gyro_dps[3], float dt_seconds);

void Mahony6D_GetEuler(const Mahony6D *filter, float angle_deg[3]);
float Mahony6D_GetYaw(const Mahony6D *filter);

#endif

