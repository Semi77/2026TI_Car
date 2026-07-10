#ifndef __UART_H
#define __UART_H

#include "stdint.h"

static void UART_SendByte(uint8_t data);
static void UART_SendString(const char s[]);
void UART_Printf(const char* format, ...);
#endif