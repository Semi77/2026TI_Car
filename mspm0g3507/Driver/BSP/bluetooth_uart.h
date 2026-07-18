#ifndef BLUETOOTH_UART_H
#define BLUETOOTH_UART_H

#include <stdint.h>

void Bluetooth_UART_Init(void);
void Bluetooth_UART_SendByte(uint8_t data);
void Bluetooth_UART_Send(const uint8_t *data, uint16_t length);
void Bluetooth_UART_SendString(const char *text);
uint16_t Bluetooth_UART_Read(uint8_t *data, uint16_t max_length);
uint16_t Bluetooth_UART_Available(void);
uint32_t Bluetooth_UART_GetOverflowCount(void);

#endif
