#ifndef MAXICAM_UART_H
#define MAXICAM_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t x;
    uint16_t y;
} MaxiCam_Point;

void MaxiCam_UART_Init(void);
void MaxiCam_UART_Process(void);
bool MaxiCam_UART_GetLatestPoint(MaxiCam_Point *point);

#endif
