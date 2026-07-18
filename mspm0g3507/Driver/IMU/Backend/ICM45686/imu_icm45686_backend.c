#include "imu_backend_config.h"

#if IMU_SELECTED_BACKEND == IMU_BACKEND_ICM45686

#include "IMU.h"
#include "icm45686_port.h"
#include "mahony6d.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define GYRO_LPF_TIME_CONSTANT_S  (0.008f)
#define GYRO_MIN_DEADBAND_DPS     (0.05f)
#define GYRO_MAX_DEADBAND_DPS     (0.50f)
#define MAX_UPDATE_DT_S           (0.025f)
/* 启动校准采用宽松阈值，避免轻微振动导致业务长时间无法启动。 */
#define CAL_STILL_SAMPLES          (20U)
#define CAL_SAMPLE_COUNT           (200U)
#define CAL_TIMEOUT_SAMPLES        (800U)
#define CAL_GYRO_STILL_LIMIT_DPS   (5.0f)
#define CAL_ACCEL_MIN_MG           (850.0f)
#define CAL_ACCEL_MAX_MG           (1150.0f)
#define CAL_GYRO_MAX_VARIANCE      (0.250f)
#define CAL_ACCEL_MAX_VARIANCE     (2500.0f)

static const Mahony6D_Config s_mahony_config = {
    .kp = 2.0f,
    .ki = 0.05f,
    .integral_limit = 0.08726646f,
    .accel_full_confidence_g = 0.05f,
    .accel_reject_g = 0.20f
};

static float s_accel_mg[3];
static float s_gyro_dps[3];
static float s_temperature_degc;
static float s_gyro_bias_dps[3];
static float s_gyro_variance_z_dps2;
static float s_filtered_gyro_dps[3];
static float s_gyro_deadband_z_dps = GYRO_MIN_DEADBAND_DPS;
static Mahony6D s_attitude_filter;

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

static void ResetGyroFilter(void)
{
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        s_filtered_gyro_dps[axis] = 0.0f;
    }
}

int IMU_Init(void)
{
    int rc = setup_imu(1, 1, 1);

    if (rc == 0) {
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            s_gyro_bias_dps[axis] = 0.0f;
        }
        s_gyro_variance_z_dps2 = 0.0f;
        s_gyro_deadband_z_dps = GYRO_MIN_DEADBAND_DPS;
        Mahony6D_Init(&s_attitude_filter, &s_mahony_config);
        ResetGyroFilter();
    }

    return rc;
}

int IMU_ReadData(float accel_mg[3], float gyro_dps[3],
        float *temperature_degc)
{
    return bsp_IcmGetRawData(accel_mg, gyro_dps, temperature_degc);
}

int IMU_Calibrate(void)
{
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_square_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_variance[3];
    float accel_norm_sum = 0.0f;
    float accel_norm_square_sum = 0.0f;
    float accel_norm_variance;
    uint16_t still_samples = 0U;
    uint16_t valid_samples = 0U;

    for (uint16_t attempt = 0U; attempt < CAL_TIMEOUT_SAMPLES; attempt++) {
        float accel_norm;
        uint8_t stationary = 1U;

        if (bsp_IcmGetRawData(s_accel_mg, s_gyro_dps,
                &s_temperature_degc) != 0) {
            return IMU_CALIBRATION_READ_ERROR;
        }

        accel_norm = sqrtf(s_accel_mg[0] * s_accel_mg[0] +
            s_accel_mg[1] * s_accel_mg[1] +
            s_accel_mg[2] * s_accel_mg[2]);

        if ((accel_norm < CAL_ACCEL_MIN_MG) ||
            (accel_norm > CAL_ACCEL_MAX_MG)) {
            stationary = 0U;
        }
        for (uint8_t axis = 0U; axis < 3U; axis++) {
            if (fabsf(s_gyro_dps[axis]) > CAL_GYRO_STILL_LIMIT_DPS) {
                stationary = 0U;
            }
        }

        if (stationary == 0U) {
            still_samples = 0U;
            valid_samples = 0U;
            accel_norm_sum = 0.0f;
            accel_norm_square_sum = 0.0f;
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                accel_sum[axis] = 0.0f;
                gyro_sum[axis] = 0.0f;
                gyro_square_sum[axis] = 0.0f;
            }
        } else if (still_samples < CAL_STILL_SAMPLES) {
            still_samples++;
        } else {
            for (uint8_t axis = 0U; axis < 3U; axis++) {
                accel_sum[axis] += s_accel_mg[axis];
                gyro_sum[axis] += s_gyro_dps[axis];
                gyro_square_sum[axis] +=
                    s_gyro_dps[axis] * s_gyro_dps[axis];
            }
            accel_norm_sum += accel_norm;
            accel_norm_square_sum += accel_norm * accel_norm;
            valid_samples++;
        }

        if (valid_samples >= CAL_SAMPLE_COUNT) {
            float sample_count = (float)valid_samples;
            uint8_t quality_ok = 1U;

            accel_norm_variance = accel_norm_square_sum / sample_count -
                (accel_norm_sum / sample_count) *
                (accel_norm_sum / sample_count);
            if (accel_norm_variance < 0.0f) {
                accel_norm_variance = 0.0f;
            }
            if (accel_norm_variance > CAL_ACCEL_MAX_VARIANCE) {
                quality_ok = 0U;
            }

            for (uint8_t axis = 0U; axis < 3U; axis++) {
                float gyro_mean = gyro_sum[axis] / sample_count;

                gyro_variance[axis] =
                    gyro_square_sum[axis] / sample_count -
                    gyro_mean * gyro_mean;
                if (gyro_variance[axis] < 0.0f) {
                    gyro_variance[axis] = 0.0f;
                }
                if (gyro_variance[axis] > CAL_GYRO_MAX_VARIANCE) {
                    quality_ok = 0U;
                }
            }

            for (uint8_t axis = 0U; axis < 3U; axis++) {
                accel_sum[axis] /= sample_count;
                s_gyro_bias_dps[axis] =
                    gyro_sum[axis] / sample_count;
            }

            s_gyro_variance_z_dps2 = gyro_variance[2];
            s_gyro_deadband_z_dps = Clamp(
                3.0f * sqrtf(s_gyro_variance_z_dps2),
                GYRO_MIN_DEADBAND_DPS,
                GYRO_MAX_DEADBAND_DPS);

            Mahony6D_SetInitialAttitude(
                &s_attitude_filter, accel_sum);
            ResetGyroFilter();

            /* 采用当前样本的最佳结果，质量不足由上层作为告警处理。 */
            return (quality_ok != 0U) ? IMU_CALIBRATION_OK :
                IMU_CALIBRATION_QUALITY_WARNING;
        }

        ICM45686_DelayMs(5U);
    }

    return IMU_CALIBRATION_QUALITY_WARNING;
}

int IMU_Update(float dt_seconds)
{
    float alpha;

    if ((dt_seconds <= 0.0f) || (dt_seconds > MAX_UPDATE_DT_S)) {
        return -1;
    }

    if (bsp_IcmGetRawData(s_accel_mg, s_gyro_dps,
            &s_temperature_degc) != 0) {
        return -1;
    }

    alpha = dt_seconds / (GYRO_LPF_TIME_CONSTANT_S + dt_seconds);
    for (uint8_t axis = 0U; axis < 3U; axis++) {
        float corrected_gyro = s_gyro_dps[axis] -
            s_gyro_bias_dps[axis];

        s_filtered_gyro_dps[axis] += alpha *
            (corrected_gyro - s_filtered_gyro_dps[axis]);
    }

    if (fabsf(s_filtered_gyro_dps[2]) < s_gyro_deadband_z_dps) {
        s_filtered_gyro_dps[2] = 0.0f;
    }

    if (Mahony6D_Update(&s_attitude_filter, s_accel_mg,
            s_filtered_gyro_dps, dt_seconds) != 0) {
        IMU_ResetAttitude();
        return -1;
    }

    return 0;
}

void IMU_GetAccel(float accel_mg[3])
{
    if (accel_mg == NULL) {
        return;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        accel_mg[axis] = s_accel_mg[axis];
    }
}

void IMU_GetGyro(float gyro_dps[3])
{
    if (gyro_dps == NULL) {
        return;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++) {
        gyro_dps[axis] = s_gyro_dps[axis];
    }
}

void IMU_GetEuler(float angle_deg[3])
{
    Mahony6D_GetEuler(&s_attitude_filter, angle_deg);
}

void IMU_ResetAttitude(void)
{
    Mahony6D_Reset(&s_attitude_filter);
    ResetGyroFilter();
}

float IMU_GetTemperature(void)
{
    return s_temperature_degc;
}

float IMU_GetGyroZ(void)
{
    return s_filtered_gyro_dps[2];
}

float IMU_GetYaw(void)
{
    return Mahony6D_GetYaw(&s_attitude_filter);
}

float IMU_GetGyroBiasZ(void)
{
    return s_gyro_bias_dps[2];
}

float IMU_GetGyroVarianceZ(void)
{
    return s_gyro_variance_z_dps2;
}

float IMU_GetDeadband(void)
{
    return s_gyro_deadband_z_dps;
}


#endif
