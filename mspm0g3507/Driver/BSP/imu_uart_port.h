#ifndef ICM45686_DEMO_IMU_UART_PORT_H
#define ICM45686_DEMO_IMU_UART_PORT_H

#include <stdint.h>

void IMU_UART_Init(void);
int IMU_UART_Write(const uint8_t *data, uint16_t length);
uint8_t IMU_UART_IsBusy(void);
uint32_t IMU_UART_GetDropCount(void);

#endif
