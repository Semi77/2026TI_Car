#include "uart.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h> 

static void UART_SendByte(uint8_t data)
{
    while(DL_UART_isTXFIFOFull(UART_0_INST));
    DL_UART_transmitData(UART_0_INST, data);
}

static void UART_SendString(const char s[])
{
    const char *p = s;
    while (*p) 
    {
        UART_SendByte((uint8_t)*p++);
    }
}

void UART_Printf(const char* format, ...)
{
    char buf[128] = {0};
    va_list args;
    va_start(args,format);
    vsnprintf(buf, sizeof(buf) -1, format, args);
    va_end(args);
    UART_SendString(buf);
}