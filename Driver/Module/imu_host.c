#include "imu_host.h"
#include "IMU.h"

#include <stddef.h>
#include <string.h>

#define IMU_HOST_FRAME_SIZE       (56U)
#define IMU_HOST_SEND_PERIOD_S    (0.020f)

static IMU_HostWriteFn s_write_fn;
static float s_send_elapsed_s;
static volatile uint32_t s_send_request_count;
static uint32_t s_send_processed_count;
static uint8_t s_10ms_divider;

static void PackInt16LE(uint8_t *destination, int16_t value)
{
    uint16_t bits = (uint16_t)value;

    destination[0] = (uint8_t)bits;
    destination[1] = (uint8_t)(bits >> 8U);
}

static void PackFloatLE(uint8_t *destination, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    destination[0] = (uint8_t)bits;
    destination[1] = (uint8_t)(bits >> 8U);
    destination[2] = (uint8_t)(bits >> 16U);
    destination[3] = (uint8_t)(bits >> 24U);
}

static void BuildFrame(uint8_t frame[IMU_HOST_FRAME_SIZE])
{
    float accel_mg[3];
    float gyro_dps[3];
    float angle_deg[3];
    uint8_t checksum = 0U;
    uint8_t axis;
    uint8_t index;

    IMU_GetAccel(accel_mg);
    IMU_GetGyro(gyro_dps);
    IMU_GetEuler(angle_deg);

    memset(frame, 0, IMU_HOST_FRAME_SIZE);
    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = 0x02U;
    frame[3] = 0x36U;

    for (axis = 0U; axis < 3U; axis++) {
        float accel_g = accel_mg[axis] / 1000.0f;

        PackInt16LE(&frame[4U + axis * 2U],
            (int16_t)(accel_mg[axis] * 2.048f));
        PackInt16LE(&frame[10U + axis * 2U],
            (int16_t)(gyro_dps[axis] * 16.4f));
        PackFloatLE(&frame[16U + axis * 4U], angle_deg[axis]);
        PackFloatLE(&frame[30U + axis * 4U], accel_g);
        PackFloatLE(&frame[42U + axis * 4U], gyro_dps[axis]);
    }

    PackInt16LE(&frame[28],
        (int16_t)(IMU_GetTemperature() * 100.0f));

    for (index = 2U; index <= 53U; index++) {
        checksum = (uint8_t)(checksum + frame[index]);
    }

    frame[54] = checksum;
    frame[55] = 0x5AU;
}

int IMU_HostInit(IMU_HostWriteFn write_fn)
{
    if (IMU_HostTxInit(write_fn) != 0) {
        return -1;
    }

    if (IMU_Init() != 0) {
        s_write_fn = NULL;
        return -2;
    }

    if (IMU_Calibrate() != 0) {
        s_write_fn = NULL;
        return -3;
    }

    return 0;
}

int IMU_HostTxInit(IMU_HostWriteFn write_fn)
{
    if (write_fn == NULL) {
        return -1;
    }

    s_write_fn = write_fn;
    s_send_elapsed_s = 0.0f;
    s_send_request_count = 0U;
    s_send_processed_count = 0U;
    s_10ms_divider = 0U;

    return 0;
}

int IMU_HostProcess(float dt_seconds)
{
    if (s_write_fn == NULL) {
        return -1;
    }

    if (IMU_Update(dt_seconds) != 0) {
        return -2;
    }

    s_send_elapsed_s += dt_seconds;
    if (s_send_elapsed_s >= IMU_HOST_SEND_PERIOD_S) {
        s_send_elapsed_s -= IMU_HOST_SEND_PERIOD_S;
        return IMU_HostSendNow();
    }

    return 0;
}

int IMU_HostSendNow(void)
{
    uint8_t frame[IMU_HOST_FRAME_SIZE];

    if (s_write_fn == NULL) {
        return -1;
    }

    BuildFrame(frame);
    return s_write_fn(frame, IMU_HOST_FRAME_SIZE);
}

void IMU_HostTimerTick10ms(void)
{
    s_10ms_divider++;
    if (s_10ms_divider >= 2U) {
        s_10ms_divider = 0U;
        s_send_request_count++;
    }
}

int IMU_HostTxProcess(void)
{
    uint32_t request_count = s_send_request_count;

    if (request_count == s_send_processed_count) {
        return 0;
    }

    s_send_processed_count = request_count;
    return IMU_HostSendNow();
}

void IMU_HostResetAttitude(void)
{
    IMU_ResetAttitude();
}
