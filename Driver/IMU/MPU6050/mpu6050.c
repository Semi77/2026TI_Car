#include "mpu6050.h"
#include "delay.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <math.h>

/* -------- I2C 地址(AD0=0) -------- */
#define MPU_ADDR        0x68
#define Scale           1.7f
/* -------- 全局变量 -------- */
float g_Gyro_X_Offset = 0.0f;
float g_Gyro_Y_Offset = 0.0f;
float g_Gyro_Z_Offset = -85.0f;  // Z轴零点漂移值（可以后面校准）
float g_Current_Pitch   = 0.0f;
float g_Current_Roll   = 0.0f;
float g_Current_Yaw   = 0.0f;    // 当前累计角度
float g_Current_Gyro_Z = 0.0f;   // 当前瞬时角速度(dps)
uint32_t g_Debug_Raw_Z = 0;

/* ============================================================
 * 1. 底层 I2C 读写
 * ============================================================ */

/* 为了避免 I2C 异常时 while 等待永远不退出，给等待加上超时上限 */
#define MPU_I2C_TIMEOUT_ITER  (10000UL)

/* 写一个寄存器 */
static void MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = { reg, data };

    DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, buffer, 2);
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST,
                                   MPU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   2);

    /* 等待传输完成 */
    for (uint32_t t = 0; t < MPU_I2C_TIMEOUT_ITER; t++) {
        if (DL_I2C_getControllerStatus(I2C_MPU6050_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) {
            break;
        }
    }
}

/* 连续读多个寄存器 */
static int MPU_Read_Bytes(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    /* 第一步：写寄存器地址 */
    DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, &reg, 1);
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST,
                                   MPU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1);

    /* 等待写完成 */
    int ok = 0;
    for (uint32_t t = 0; t < MPU_I2C_TIMEOUT_ITER; t++) {
        if (DL_I2C_getControllerStatus(I2C_MPU6050_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) {
            ok = 1;
            break;
        }
    }
    if (!ok) {
        DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
        return 0;
    }

    /* 第二步：开始读 len 字节 */
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST,
                                   MPU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   len);

    for (uint8_t i = 0; i < len; i++) {
        int byte_ok = 0;
        for (uint32_t t = 0; t < MPU_I2C_TIMEOUT_ITER; t++) {
            if (!DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) {
                byte_ok = 1;
                break;
            }
        }
        if (!byte_ok) {
            DL_I2C_resetControllerTransfer(I2C_MPU6050_INST);
            return 0;
        }
        buffer[i] = DL_I2C_receiveControllerData(I2C_MPU6050_INST);
    }

    return 1;
}

/* ============================================================
 * 2. 零点校准（可选）
 * ============================================================ */

void MPU6050_Calibration(void)
{
    float   gyro_sum = 0.0f;
    uint8_t buf[2];
    int16_t raw;
    uint16_t ok_cnt = 0;

    /* 采样更多点，并丢弃前面一段输出不稳定的样本 */
    const uint16_t samples = 400;
    const uint16_t discard = 50;

    for (uint16_t i = 0; i < samples; i++) {
        /* GYRO_ZOUT_H = 0x47 */
        if (!MPU_Read_Bytes(0x47, buf, 2)) {
            /* I2C 异常时避免卡死 */
            break;
        }
        raw = (int16_t)((buf[0] << 8) | buf[1]);
        if (i >= discard) {
            gyro_sum += (float)raw;
            ok_cnt++;
        }
        delay_ms(1);                           // 简单延时
    }

    if (ok_cnt > 0) {
        g_Gyro_Z_Offset = gyro_sum / (float)ok_cnt;
    }
    g_Current_Yaw   = 0.0f;
}

/* ============================================================
 * 3. 初始化 & 算法接口
 * ============================================================ */

void MPU6050_Init(void)
{
    /* 1. 复位设备 */
    MPU_Write_Byte(0x6B, 0x80);
    delay_ms(100);

    /* 2. 唤醒设备，使用内部时钟 */
    MPU_Write_Byte(0x6B, 0x00);

    /* 3. 配置低通滤波 (DLPF)，这里设为 4 (带宽约 20Hz) */
    MPU_Write_Byte(0x1A, 0x04);

    /* 4. 配置陀螺仪量程 ±2000 dps (0x18) 或按你需要修改 */
    MPU_Write_Byte(0x1B, 0x18);

    /* 5. 可以做一次零点校准（静止放好时调用） */
    //delay_ms(2000);
    g_Gyro_Z_Offset = 0.0f;
    //MPU6050_Calibration();
    g_Current_Yaw   = 0.0f;
    /* 这里不要覆盖零偏校准结果，否则会导致 yaw 积分漂移很快 */
}

/* 放在定时器中断或固定周期里调用，用积分更新 yaw */
void MPU6050_Update_Yaw(float dt_ms)
{
    uint8_t buf[2];
    int16_t raw_z;
    float   gyro_dps;
    float   val_no_offset;

    /* 1. 读取 GYRO_Z (0x47 高字节) */
    if (!MPU_Read_Bytes(0x47, buf, 2)) {
        /* 读取失败则跳过本次积分，避免把错误值积分进 yaw */
        return;
    }
    raw_z = (int16_t)((buf[0] << 8) | buf[1]);
    

    /* 2. 减去零漂 */
    val_no_offset = (float)raw_z - g_Gyro_Z_Offset;

    /* 3. 转成 deg/s
       注意：如果你量程设的是 ±2000 dps，对应灵敏度是 16.4 LSB/(deg/s)
       如果你实际用的是其它量程（比如 250dps），要改这里的系数 */
    gyro_dps = val_no_offset / 16.4f;

    /* 4. 静止/轻微运动时做慢速零偏跟踪，抑制温漂导致的持续飘 */
    if (fabsf(gyro_dps) < 2.0f) {
        g_Gyro_Z_Offset = g_Gyro_Z_Offset + 0.0005f * ((float)raw_z - g_Gyro_Z_Offset);
        val_no_offset = (float)raw_z - g_Gyro_Z_Offset;
        gyro_dps = val_no_offset / 16.4f;
    }

    // /* 5. 死区（防止静止时轻微抖动） */
    if (fabsf(gyro_dps) < 0.5f) {
         gyro_dps = 0.0f;
    }
    g_Current_Gyro_Z = gyro_dps;

    /* 6. 积分得到角度: yaw(k) = yaw(k-1) + ω * dt */
    g_Current_Yaw += gyro_dps * (dt_ms / 1000.0f) *Scale;
}

/* 对外提供的获取接口 */
float MPU_Get_Yaw(void)
{
    return g_Current_Yaw;
}

float MPU_Get_Gyro_Z(void)
{
    return g_Current_Gyro_Z;
}

void MPU_Reset_Yaw(void)
{
    g_Current_Yaw = 0.0f;
}
