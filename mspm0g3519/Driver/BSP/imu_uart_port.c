#include "imu_uart_port.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define IMU_UART_TX_BUFFER_SIZE (56U)

static uint8_t s_tx_buffer[IMU_UART_TX_BUFFER_SIZE];
static volatile bool s_tx_busy;
static volatile bool s_dma_initialized;
static volatile uint32_t s_drop_count;

static int UART_WritePolling(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; index++) {
        while (DL_UART_isBusy(UART_TEST_INST) == true) {
        }
        DL_UART_Main_transmitData(UART_TEST_INST, data[index]);
    }

    return 0;
}

void IMU_UART_Init(void)
{
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    s_tx_busy = false;
    s_drop_count = 0U;
    s_dma_initialized = true;

    DL_UART_Main_clearInterruptStatus(UART_TEST_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
        DL_UART_MAIN_INTERRUPT_EOT_DONE);
    NVIC_ClearPendingIRQ(UART_TEST_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TEST_INST_INT_IRQN);
}

int IMU_UART_Write(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return -1;
    }

    if (length > IMU_UART_TX_BUFFER_SIZE) {
        return -2;
    }

    if ((s_dma_initialized == false) || (s_tx_busy == true)) {
        s_drop_count++;
        return -3;
    }

    memcpy(s_tx_buffer, data, length);
    s_tx_busy = true;

    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t)&s_tx_buffer[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t)&UART_TEST_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    return 0;
}

uint8_t IMU_UART_IsBusy(void)
{
    return s_tx_busy ? 1U : 0U;
}

uint32_t IMU_UART_GetDropCount(void)
{
    return s_drop_count;
}

void UART_TEST_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_TEST_INST)) {
        case DL_UART_MAIN_IIDX_DMA_DONE_TX:
            break;
        case DL_UART_MAIN_IIDX_EOT_DONE:
            s_tx_busy = false;
            break;
        default:
            break;
    }
}

int fputc(int ch, FILE *stream)
{
    uint8_t data = (uint8_t)ch;

    (void)stream;
    if ((s_dma_initialized == true) ||
        (UART_WritePolling(&data, 1U) != 0)) {
        return EOF;
    }
    return ch;
}

int fputs(const char *restrict text, FILE *restrict stream)
{
    int length = 0;

    (void)stream;
    while (*text != '\0') {
        uint8_t data = (uint8_t)*text++;

        if ((s_dma_initialized == true) ||
            (UART_WritePolling(&data, 1U) != 0)) {
            return EOF;
        }
        length++;
    }

    return length;
}

int puts(const char *text)
{
    (void)text;
    return 0;
}
