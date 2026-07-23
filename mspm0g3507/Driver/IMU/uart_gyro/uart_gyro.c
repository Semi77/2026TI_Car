#include "uart_gyro.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <stdint.h>

#define UART_GYRO_FRAME_SIZE          (5U)
#define UART_GYRO_FRAME_HEADER        (0x5AU)
#define UART_GYRO_TYPE_GYRO_Z         (0xAAU)
#define UART_GYRO_TYPE_YAW            (0xBBU)
#define UART_GYRO_TYPE_STATUS         (0xCCU)
#define UART_GYRO_BIAS_STATUS_OK      (0x96U)
#define UART_GYRO_GYRO_SCALE_DPS      (2000.0f)
#define UART_GYRO_YAW_SCALE_DEG       (180.0f)
#define UART_GYRO_STATUS_WAIT_MS      (500U)

static uint8_t s_rx_buffer[UART_GYRO_FRAME_SIZE];
static uint8_t s_rx_count;
static volatile int16_t s_raw_gyro_z;
static volatile int16_t s_raw_yaw;
static volatile bool s_gyro_z_updated;
static volatile bool s_yaw_updated;
static volatile bool s_bias_status_received;
static volatile bool s_bias_calibration_ok;

static const uint8_t s_unlock_command[5] = {
    0x55U, 0xAAU, 0x13U, 0x8EU, 0x5FU
};
static const uint8_t s_yaw_zero_command[5] = {
    0x55U, 0xAAU, 0x15U, 0x00U, 0x00U
};
static const uint8_t s_bias_calibration_command[5] = {
    0x55U, 0xAAU, 0x0AU, 0x01U, 0x00U
};
static const uint8_t s_bias_status_command[5] = {
    0x55U, 0xAAU, 0x04U, 0x0AU, 0x00U
};
static const uint8_t s_save_command[5] = {
    0x55U, 0xAAU, 0x00U, 0x00U, 0x00U
};

/* 该函数阻塞发送一个字节到串口陀螺仪。 */
static void UARTGyro_SendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_GYRO_INST)) {
    }
    DL_UART_Main_transmitData(UART_GYRO_INST, data);
}

/* 该函数阻塞发送指定长度的命令数据到串口陀螺仪。 */
static void UARTGyro_SendBytes(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == NULL) {
        return;
    }
    for (index = 0U; index < length; index++) {
        UARTGyro_SendByte(data[index]);
    }
    while (DL_UART_Main_isBusy(UART_GYRO_INST)) {
    }
}

/* 该函数校验并保存一帧完整的陀螺仪数据。 */
static void UARTGyro_ParseFrame(void)
{
    uint8_t checksum = (uint8_t)(s_rx_buffer[0] + s_rx_buffer[1] +
        s_rx_buffer[2] + s_rx_buffer[3]);
    int16_t raw_value;

    if (s_rx_buffer[1] == UART_GYRO_TYPE_STATUS) {
        s_bias_status_received = true;
        s_bias_calibration_ok = (s_rx_buffer[2] == 0x00U) &&
            (s_rx_buffer[3] == 0x00U) &&
            ((s_rx_buffer[4] == UART_GYRO_BIAS_STATUS_OK) ||
             (s_rx_buffer[4] == checksum));
        return;
    }
    if (checksum != s_rx_buffer[4]) {
        return;
    }
    raw_value = (int16_t)(((uint16_t)s_rx_buffer[3] << 8U) |
        (uint16_t)s_rx_buffer[2]);

    if (s_rx_buffer[1] == UART_GYRO_TYPE_GYRO_Z) {
        s_raw_gyro_z = raw_value;
        s_gyro_z_updated = true;
    } else if (s_rx_buffer[1] == UART_GYRO_TYPE_YAW) {
        s_raw_yaw = raw_value;
        s_yaw_updated = true;
    }
}

/* 该函数把一个UART字节送入五字节数据帧解析状态机。 */
static void UARTGyro_ReceiveByte(uint8_t data)
{
    if (s_rx_count == 0U) {
        if (data == UART_GYRO_FRAME_HEADER) {
            s_rx_buffer[s_rx_count++] = data;
        }
        return;
    }

    if (s_rx_count == 1U) {
        if ((data != UART_GYRO_TYPE_GYRO_Z) &&
            (data != UART_GYRO_TYPE_YAW) &&
            (data != UART_GYRO_TYPE_STATUS)) {
            s_rx_count = 0U;
            if (data == UART_GYRO_FRAME_HEADER) {
                s_rx_buffer[s_rx_count++] = data;
            }
            return;
        }
    }

    s_rx_buffer[s_rx_count++] = data;
    if (s_rx_count >= UART_GYRO_FRAME_SIZE) {
        UARTGyro_ParseFrame();
        s_rx_count = 0U;
    }
}

/* 该函数初始化串口陀螺仪的接收状态和UART中断。 */
void UARTGyro_Init(void)
{
    s_rx_count = 0U;
    s_raw_gyro_z = 0;
    s_raw_yaw = 0;
    s_gyro_z_updated = false;
    s_yaw_updated = false;
    s_bias_status_received = false;
    s_bias_calibration_ok = false;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_GYRO_INST)) {
        (void)DL_UART_Main_receiveData(UART_GYRO_INST);
    }
    DL_UART_Main_clearInterruptStatus(
        UART_GYRO_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(
        UART_GYRO_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_GYRO_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_GYRO_INST_INT_IRQN);
}

/* 该函数在收到新的有效角度帧时返回最新Yaw角度。 */
bool UARTGyro_GetYaw(float *yaw_deg)
{
    int16_t raw_yaw;
    bool updated;

    if (yaw_deg == NULL) {
        return false;
    }
    DL_UART_Main_disableInterrupt(
        UART_GYRO_INST, DL_UART_MAIN_INTERRUPT_RX);
    raw_yaw = s_raw_yaw;
    updated = s_yaw_updated;
    s_yaw_updated = false;
    DL_UART_Main_enableInterrupt(
        UART_GYRO_INST, DL_UART_MAIN_INTERRUPT_RX);

    if (!updated) {
        return false;
    }
    *yaw_deg = (float)raw_yaw / 32768.0f * UART_GYRO_YAW_SCALE_DEG;
    return true;
}

/* 该函数在收到新的有效角速度帧时返回最新Z轴角速度。 */
bool UARTGyro_GetGyroZ(float *gyro_z_dps)
{
    int16_t raw_gyro_z;
    bool updated;

    if (gyro_z_dps == NULL) {
        return false;
    }
    DL_UART_Main_disableInterrupt(
        UART_GYRO_INST, DL_UART_MAIN_INTERRUPT_RX);
    raw_gyro_z = s_raw_gyro_z;
    updated = s_gyro_z_updated;
    s_gyro_z_updated = false;
    DL_UART_Main_enableInterrupt(
        UART_GYRO_INST, DL_UART_MAIN_INTERRUPT_RX);

    if (!updated) {
        return false;
    }
    *gyro_z_dps = (float)raw_gyro_z / 32768.0f *
        UART_GYRO_GYRO_SCALE_DPS;
    return true;
}

/* 该函数发送命令将模块当前Yaw角度设置为零并保存。 */
void UARTGyro_ResetYaw(void)
{
    UARTGyro_SendBytes(s_unlock_command, sizeof(s_unlock_command));
    delay_ms(100U);
    UARTGyro_SendBytes(s_yaw_zero_command, sizeof(s_yaw_zero_command));
    delay_ms(100U);
    UARTGyro_SendBytes(s_save_command, sizeof(s_save_command));
}

/* 该函数执行零偏校准、读取校准状态并返回是否成功。 */
bool UARTGyro_CalibrateBias(void)
{
    uint32_t wait_ms;

    s_bias_status_received = false;
    s_bias_calibration_ok = false;
    UARTGyro_SendBytes(s_unlock_command, sizeof(s_unlock_command));
    delay_ms(100U);
    UARTGyro_SendBytes(s_bias_calibration_command,
        sizeof(s_bias_calibration_command));
    delay_ms(20000U);
    UARTGyro_SendBytes(s_bias_status_command,
        sizeof(s_bias_status_command));
    for (wait_ms = 0U; wait_ms < UART_GYRO_STATUS_WAIT_MS; wait_ms += 10U) {
        if (s_bias_status_received) {
            break;
        }
        delay_ms(10U);
    }
    UARTGyro_SendBytes(s_save_command, sizeof(s_save_command));
    return s_bias_status_received && s_bias_calibration_ok;
}

/* 该函数处理UART_GYRO接收中断并读取FIFO中的全部字节。 */
void UART_GYRO_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_GYRO_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_GYRO_INST)) {
            UARTGyro_ReceiveByte(
                DL_UART_Main_receiveData(UART_GYRO_INST));
        }
        break;
    default:
        break;
    }
}
