#include "imu_drdy.h"
#include "imu_backend_config.h"

#if IMU_SELECTED_BACKEND == IMU_BACKEND_ICM45686

#include "IMU.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

#define IMU_DRDY_SAMPLE_PERIOD_S    (0.005f)
#define IMU_DRDY_MAX_PENDING_COUNT  (5U)

static volatile uint8_t s_pending_count;

void IMU_DRDY_Init(void)
{
    s_pending_count = 0U;
    DL_GPIO_clearInterruptStatus(GPIO_MPU6050_PORT,
        GPIO_MPU6050_PIN_INT_PIN);
    NVIC_ClearPendingIRQ(GPIO_MPU6050_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MPU6050_INT_IRQN);
}

void IMU_DRDY_Process(void)
{
    uint8_t pending_count;

    __disable_irq();
    pending_count = s_pending_count;
    s_pending_count = 0U;
    __enable_irq();

    if (pending_count != 0U) {
        (void)IMU_Update((float)pending_count *
            IMU_DRDY_SAMPLE_PERIOD_S);
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            if (DL_GPIO_getPendingInterrupt(GPIO_MPU6050_PORT) ==
                GPIO_MPU6050_PIN_INT_IIDX) {
                if (s_pending_count < IMU_DRDY_MAX_PENDING_COUNT) {
                    s_pending_count++;
                }
            }
            break;
        default:
            break;
    }
}

#else

void IMU_DRDY_Init(void)
{
}

void IMU_DRDY_Process(void)
{
}

#endif
