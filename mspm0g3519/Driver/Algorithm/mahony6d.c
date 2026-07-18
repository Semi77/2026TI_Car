#include "mahony6d.h"

#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD  (0.01745329252f)
#define RAD_TO_DEG  (57.29577951f)

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float GetAccelConfidence(const Mahony6D *filter,
    float accel_norm_g)
{
    float error_g = fabsf(accel_norm_g - 1.0f);

    if (error_g <= filter->config.accel_full_confidence_g) {
        return 1.0f;
    }
    if (error_g >= filter->config.accel_reject_g) {
        return 0.0f;
    }

    return (filter->config.accel_reject_g - error_g) /
        (filter->config.accel_reject_g -
         filter->config.accel_full_confidence_g);
}

void Mahony6D_Init(Mahony6D *filter, const Mahony6D_Config *config)
{
    if ((filter == NULL) || (config == NULL)) {
        return;
    }

    filter->config = *config;
    Mahony6D_Reset(filter);
}

void Mahony6D_Reset(Mahony6D *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;

    for (unsigned int axis = 0U; axis < 3U; axis++) {
        filter->integral_feedback[axis] = 0.0f;
    }
}

void Mahony6D_SetInitialAttitude(Mahony6D *filter,
    const float accel_mg[3])
{
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    float cr;
    float sr;
    float cp;
    float sp;

    if ((filter == NULL) || (accel_mg == NULL)) {
        return;
    }

    roll = atan2f(accel_mg[1], accel_mg[2]);
    pitch = atan2f(-accel_mg[0],
        sqrtf(accel_mg[1] * accel_mg[1] +
              accel_mg[2] * accel_mg[2]));
    half_roll = 0.5f * roll;
    half_pitch = 0.5f * pitch;
    cr = cosf(half_roll);
    sr = sinf(half_roll);
    cp = cosf(half_pitch);
    sp = sinf(half_pitch);

    filter->q[0] = cr * cp;
    filter->q[1] = sr * cp;
    filter->q[2] = cr * sp;
    filter->q[3] = -sr * sp;

    for (unsigned int axis = 0U; axis < 3U; axis++) {
        filter->integral_feedback[axis] = 0.0f;
    }
}

int Mahony6D_Update(Mahony6D *filter, const float accel_mg[3],
    const float gyro_dps[3], float dt_seconds)
{
    float gyro_rad[3];
    float accel_norm;
    float accel_confidence;
    float ax;
    float ay;
    float az;
    float vx;
    float vy;
    float vz;
    float error[3];
    float qa;
    float qb;
    float qc;
    float quaternion_norm;

    if ((filter == NULL) || (accel_mg == NULL) ||
        (gyro_dps == NULL) || (dt_seconds <= 0.0f)) {
        return -1;
    }

    for (unsigned int axis = 0U; axis < 3U; axis++) {
        gyro_rad[axis] = gyro_dps[axis] * DEG_TO_RAD;
    }

    accel_norm = sqrtf(accel_mg[0] * accel_mg[0] +
        accel_mg[1] * accel_mg[1] +
        accel_mg[2] * accel_mg[2]);
    accel_confidence = GetAccelConfidence(filter,
        accel_norm / 1000.0f);

    if ((accel_norm > 1.0f) && (accel_confidence > 0.0f)) {
        ax = accel_mg[0] / accel_norm;
        ay = accel_mg[1] / accel_norm;
        az = accel_mg[2] / accel_norm;

        vx = 2.0f * (filter->q[1] * filter->q[3] -
            filter->q[0] * filter->q[2]);
        vy = 2.0f * (filter->q[0] * filter->q[1] +
            filter->q[2] * filter->q[3]);
        vz = filter->q[0] * filter->q[0] -
            filter->q[1] * filter->q[1] -
            filter->q[2] * filter->q[2] +
            filter->q[3] * filter->q[3];

        error[0] = (ay * vz - az * vy) * accel_confidence;
        error[1] = (az * vx - ax * vz) * accel_confidence;
        error[2] = (ax * vy - ay * vx) * accel_confidence;

        for (unsigned int axis = 0U; axis < 3U; axis++) {
            filter->integral_feedback[axis] = Clamp(
                filter->integral_feedback[axis] +
                    filter->config.ki * error[axis] * dt_seconds,
                -filter->config.integral_limit,
                filter->config.integral_limit);
            gyro_rad[axis] += filter->config.kp * error[axis];
        }
    }

    for (unsigned int axis = 0U; axis < 3U; axis++) {
        gyro_rad[axis] += filter->integral_feedback[axis];
        gyro_rad[axis] *= 0.5f * dt_seconds;
    }

    qa = filter->q[0];
    qb = filter->q[1];
    qc = filter->q[2];
    filter->q[0] += -qb * gyro_rad[0] - qc * gyro_rad[1] -
        filter->q[3] * gyro_rad[2];
    filter->q[1] += qa * gyro_rad[0] + qc * gyro_rad[2] -
        filter->q[3] * gyro_rad[1];
    filter->q[2] += qa * gyro_rad[1] - qb * gyro_rad[2] +
        filter->q[3] * gyro_rad[0];
    filter->q[3] += qa * gyro_rad[2] + qb * gyro_rad[1] -
        qc * gyro_rad[0];

    quaternion_norm = sqrtf(filter->q[0] * filter->q[0] +
        filter->q[1] * filter->q[1] +
        filter->q[2] * filter->q[2] +
        filter->q[3] * filter->q[3]);
    if (quaternion_norm <= 0.0f) {
        Mahony6D_Reset(filter);
        return -1;
    }

    for (unsigned int axis = 0U; axis < 4U; axis++) {
        filter->q[axis] /= quaternion_norm;
    }

    return 0;
}

void Mahony6D_GetEuler(const Mahony6D *filter, float angle_deg[3])
{
    float pitch_sine;

    if ((filter == NULL) || (angle_deg == NULL)) {
        return;
    }

    angle_deg[0] = atan2f(
        2.0f * (filter->q[0] * filter->q[1] +
                filter->q[2] * filter->q[3]),
        1.0f - 2.0f * (filter->q[1] * filter->q[1] +
                       filter->q[2] * filter->q[2])) * RAD_TO_DEG;
    pitch_sine = 2.0f * (filter->q[0] * filter->q[2] -
        filter->q[3] * filter->q[1]);
    angle_deg[1] = asinf(Clamp(pitch_sine, -1.0f, 1.0f)) * RAD_TO_DEG;
    angle_deg[2] = Mahony6D_GetYaw(filter);
}

float Mahony6D_GetYaw(const Mahony6D *filter)
{
    if (filter == NULL) {
        return 0.0f;
    }

    return atan2f(
        2.0f * (filter->q[0] * filter->q[3] +
                filter->q[1] * filter->q[2]),
        1.0f - 2.0f * (filter->q[2] * filter->q[2] +
                       filter->q[3] * filter->q[3])) * RAD_TO_DEG;
}

