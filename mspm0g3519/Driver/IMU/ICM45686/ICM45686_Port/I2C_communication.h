#ifndef ICM45686_I2C_COMMUNICATION_H
#define ICM45686_I2C_COMMUNICATION_H

#include <stdint.h>

/* ICM45686 7-bit I2C address. The R/W bit is not included. */
#define ICM45686_I2C_ADDRESS       (0x69U)
#define ICM45686_I2C_MAX_TRANSFER  (16U)

int I2C_WriteReg(uint8_t address, uint8_t reg_address,
    const uint8_t *reg_data, uint8_t count);
int I2C_ReadReg(uint8_t address, uint8_t reg_address,
    uint8_t *reg_data, uint8_t count);

#endif
