#include "bluetooth_uart.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define BLUETOOTH_UART_RX_BUFFER_SIZE (128U)

static volatile uint8_t s_rx_buffer[BLUETOOTH_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overflow_count;

static uint16_t Bluetooth_UART_NextIndex(uint16_t index)
{
    index++;
    if (index >= BLUETOOTH_UART_RX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

void Bluetooth_UART_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow_count = 0U;

    DL_UART_Main_clearInterruptStatus(
        UART_BLUETOOTH_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_BLUETOOTH_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

void Bluetooth_UART_SendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
    }
    DL_UART_Main_transmitData(UART_BLUETOOTH_INST, data);
}

void Bluetooth_UART_Send(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0U; index < length; index++) {
        Bluetooth_UART_SendByte(data[index]);
    }
}

void Bluetooth_UART_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        Bluetooth_UART_SendByte((uint8_t)*text);
        text++;
    }
}

uint16_t Bluetooth_UART_Read(uint8_t *data, uint16_t max_length)
{
    uint16_t count = 0U;

    if (data == NULL) {
        return 0U;
    }

    while ((count < max_length) && (s_rx_tail != s_rx_head)) {
        data[count] = s_rx_buffer[s_rx_tail];
        s_rx_tail = Bluetooth_UART_NextIndex(s_rx_tail);
        count++;
    }

    return count;
}

uint16_t Bluetooth_UART_Available(void)
{
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    if (head >= tail) {
        return head - tail;
    }
    return BLUETOOTH_UART_RX_BUFFER_SIZE - tail + head;
}

uint32_t Bluetooth_UART_GetOverflowCount(void)
{
    return s_rx_overflow_count;
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
            uint8_t data =
                DL_UART_Main_receiveData(UART_BLUETOOTH_INST);
            uint16_t next_head = Bluetooth_UART_NextIndex(s_rx_head);

            if (next_head == s_rx_tail) {
                s_rx_overflow_count++;
            } else {
                s_rx_buffer[s_rx_head] = data;
                s_rx_head = next_head;
            }
        }
        break;

    default:
        break;
    }
}
