#include "imu_backend_config.h"

#if IMU_SELECTED_BACKEND == IMU_BACKEND_ICM45686

#include "I2C_communication.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define ICM45686_I2C_TIMEOUT_ITERATIONS  (100000UL)

static void I2C_RecoverController(void)
{
    DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    DL_I2C_flushControllerRXFIFO(I2C_MPU6050_INST);
}

static int I2C_WaitForIdle(void)
{
    uint32_t timeout = ICM45686_I2C_TIMEOUT_ITERATIONS;

    while (timeout-- > 0U) {
        uint32_t status =
            DL_I2C_getControllerStatus(I2C_MPU6050_INST);

        if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                       DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0U) {
            I2C_RecoverController();
            return -1;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return 0;
        }
    }

    I2C_RecoverController();
    return -1;
}

static int I2C_WaitForRxData(void)
{
    uint32_t timeout = ICM45686_I2C_TIMEOUT_ITERATIONS;

    while (timeout-- > 0U) {
        uint32_t status =
            DL_I2C_getControllerStatus(I2C_MPU6050_INST);

        if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                       DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0U) {
            I2C_RecoverController();
            return -1;
        }
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) {
            return 0;
        }
    }

    I2C_RecoverController();
    return -1;
}

int I2C_WriteReg(uint8_t address, uint8_t reg_address,
        const uint8_t *reg_data, uint8_t count)
{
    uint8_t packet[ICM45686_I2C_MAX_TRANSFER + 1U];
    uint8_t index;

    if ((count > ICM45686_I2C_MAX_TRANSFER) ||
        ((count > 0U) && (reg_data == NULL))) {
        return -1;
    }
    if (I2C_WaitForIdle() != 0) {
        return -1;
    }

    packet[0] = reg_address;
    for (index = 0U; index < count; index++) {
        packet[index + 1U] = reg_data[index];
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    if (DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, packet,
            (uint32_t)count + 1U) != ((uint32_t)count + 1U)) {
        I2C_RecoverController();
        return -1;
    }

    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)count + 1U);

    if (I2C_WaitForIdle() != 0) {
        return -1;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    return 0;
}

int I2C_ReadReg(uint8_t address, uint8_t reg_address,
        uint8_t *reg_data, uint8_t count)
{
    uint8_t index;

    if ((reg_data == NULL) || (count == 0U) ||
        (count > ICM45686_I2C_MAX_TRANSFER)) {
        return -1;
    }
    if (I2C_WaitForIdle() != 0) {
        return -1;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    DL_I2C_flushControllerRXFIFO(I2C_MPU6050_INST);
    if (DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST,
            &reg_address, 1U) != 1U) {
        I2C_RecoverController();
        return -1;
    }

    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    if (I2C_WaitForIdle() != 0) {
        return -1;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, count);

    for (index = 0U; index < count; index++) {
        if (I2C_WaitForRxData() != 0) {
            return -1;
        }
        reg_data[index] =
            DL_I2C_receiveControllerData(I2C_MPU6050_INST);
    }

    return I2C_WaitForIdle();
}

#endif
