#ifndef ICM45686_DEMO_IMU_HOST_H
#define ICM45686_DEMO_IMU_HOST_H

#include <stdint.h>

typedef int (*IMU_HostWriteFn)(const uint8_t *data, uint16_t length);

/* Initializes and calibrates the IMU. Call after board peripherals are ready. */
int IMU_HostInit(IMU_HostWriteFn write_fn);

/* Binds the host transport without initializing or calibrating the IMU. */
int IMU_HostTxInit(IMU_HostWriteFn write_fn);

/* Updates the attitude and sends the fixed JYTech frame at 50 Hz. */
int IMU_HostProcess(float dt_seconds);

/* Sends one frame immediately using the latest IMU data. */
int IMU_HostSendNow(void);

/* Call from the existing 10 ms timer ISR. Only updates the send request. */
void IMU_HostTimerTick10ms(void);

/* Call from the main loop to build and start a pending host frame. */
int IMU_HostTxProcess(void);

void IMU_HostResetAttitude(void);

#endif
