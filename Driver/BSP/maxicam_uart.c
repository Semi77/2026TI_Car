#include "maxicam_uart.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define MAXICAM_COORD_DIGITS (4U)

typedef enum {
    MAXICAM_WAIT_START = 0,
    MAXICAM_RECEIVE_X,
    MAXICAM_WAIT_DELIMITER,
    MAXICAM_RECEIVE_Y,
    MAXICAM_WAIT_END
} MaxiCam_RxState;

static MaxiCam_RxState s_rx_state;
static uint16_t s_rx_x;
static uint16_t s_rx_y;
static uint8_t s_digit_count;

static volatile uint16_t s_pending_x;
static volatile uint16_t s_pending_y;
static volatile bool s_frame_pending;

static MaxiCam_Point s_latest_point;
static bool s_latest_point_updated;

static bool MaxiCam_IsDigit(uint8_t data)
{
    return (data >= (uint8_t)'0') && (data <= (uint8_t)'9');
}

static void MaxiCam_ResetParser(void)
{
    s_rx_state = MAXICAM_WAIT_START;
    s_rx_x = 0U;
    s_rx_y = 0U;
    s_digit_count = 0U;
}

static void MaxiCam_StartFrame(void)
{
    s_rx_x = 0U;
    s_rx_y = 0U;
    s_digit_count = 0U;
    s_rx_state = MAXICAM_RECEIVE_X;
}

static void MaxiCam_ReceiveByte(uint8_t data)
{
    if (data == (uint8_t)'#') {
        MaxiCam_StartFrame();
        return;
    }

    switch (s_rx_state) {
    case MAXICAM_WAIT_START:
        break;

    case MAXICAM_RECEIVE_X:
        if (!MaxiCam_IsDigit(data)) {
            MaxiCam_ResetParser();
            break;
        }

        s_rx_x = (uint16_t)(s_rx_x * 10U +
            (uint16_t)(data - (uint8_t)'0'));
        s_digit_count++;
        if (s_digit_count == MAXICAM_COORD_DIGITS) {
            s_rx_state = MAXICAM_WAIT_DELIMITER;
        }
        break;

    case MAXICAM_WAIT_DELIMITER:
        if (data == (uint8_t)'|') {
            s_digit_count = 0U;
            s_rx_state = MAXICAM_RECEIVE_Y;
        } else {
            MaxiCam_ResetParser();
        }
        break;

    case MAXICAM_RECEIVE_Y:
        if (!MaxiCam_IsDigit(data)) {
            MaxiCam_ResetParser();
            break;
        }

        s_rx_y = (uint16_t)(s_rx_y * 10U +
            (uint16_t)(data - (uint8_t)'0'));
        s_digit_count++;
        if (s_digit_count == MAXICAM_COORD_DIGITS) {
            s_rx_state = MAXICAM_WAIT_END;
        }
        break;

    case MAXICAM_WAIT_END:
        if (data == (uint8_t)'$') {
            s_pending_x = s_rx_x;
            s_pending_y = s_rx_y;
            s_frame_pending = true;
        }
        MaxiCam_ResetParser();
        break;

    default:
        MaxiCam_ResetParser();
        break;
    }
}

void MaxiCam_UART_Init(void)
{
    MaxiCam_ResetParser();
    s_pending_x = 0U;
    s_pending_y = 0U;
    s_frame_pending = false;
    s_latest_point.x = 0U;
    s_latest_point.y = 0U;
    s_latest_point_updated = false;

    DL_UART_Main_clearInterruptStatus(
        UART_MAXI_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_MAXI_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_MAXI_INST_INT_IRQN);
}

void MaxiCam_UART_Process(void)
{
    DL_UART_Main_disableInterrupt(
        UART_MAXI_INST, DL_UART_MAIN_INTERRUPT_RX);

    if (s_frame_pending) {
        s_latest_point.x = s_pending_x;
        s_latest_point.y = s_pending_y;
        s_frame_pending = false;
        s_latest_point_updated = true;
    }

    DL_UART_Main_enableInterrupt(
        UART_MAXI_INST, DL_UART_MAIN_INTERRUPT_RX);
}

bool MaxiCam_UART_GetLatestPoint(MaxiCam_Point *point)
{
    if ((point == NULL) || !s_latest_point_updated) {
        return false;
    }

    *point = s_latest_point;
    s_latest_point_updated = false;
    return true;
}

void UART_MAXI_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_MAXI_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_MAXI_INST)) {
            MaxiCam_ReceiveByte(
                DL_UART_Main_receiveData(UART_MAXI_INST));
        }
        break;

    default:
        break;
    }
}
