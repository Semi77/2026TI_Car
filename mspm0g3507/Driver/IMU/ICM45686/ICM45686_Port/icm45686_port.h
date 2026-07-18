#ifndef __ICM45686_PORT_H
#define __ICM45686_PORT_H

#include <stdint.h>

/*
 * ICM45686椹卞姩涓庢湰宸ョ▼SPI/IIC搴曞眰鐨勯€傞厤灞傘€? */

int setup_imu(int use_ln, int accel_en, int gyro_en);
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);
void ICM45686_DelayMs(uint32_t ms);

#endif
