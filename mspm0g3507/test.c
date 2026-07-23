#include "test.h"

#include "bluetooth_uart.h"
#include "maxicam_uart.h"
#include "motor.h"
#include "Display/oled_hardware_i2c.h"
#include "Display/st7735s.h"
#include "IMU/uart_gyro/uart_gyro.h"
#include "Sensor/No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "ti_msp_dl_config.h"
#include "uart.h"

#include <stdint.h>

#define BLUETOOTH_OLED_FONT_SIZE       (8U)
#define BLUETOOTH_OLED_CHAR_WIDTH      (6U)
#define BLUETOOTH_OLED_COLUMNS         (21U)
#define BLUETOOTH_OLED_ROWS            (8U)
#define BLUETOOTH_OLED_READ_SIZE       (32U)
#define ST7735S_COLOR_HOLD_MS           (1000U)
#define UART_GYRO_GRAY_REFRESH_MS       (100U)
#define UART_GYRO_GRAY_FONT_SIZE        (8U)

static uint8_t s_bluetooth_oled_column;
static uint8_t s_bluetooth_oled_row;
static uint8_t s_st7735s_color_index;
static No_MCU_Sensor s_uart_gyro_gray_sensor;
static unsigned short s_uart_gyro_gray_white[8] = {
    1800U, 1800U, 1800U, 1800U,
    1800U, 1800U, 1800U, 1800U,
};
static unsigned short s_uart_gyro_gray_black[8] = {
    300U, 300U, 300U, 300U,
    300U, 300U, 300U, 300U,
};

static void Test_BluetoothOLED_NewLine(void)
{
    s_bluetooth_oled_column = 0U;
    s_bluetooth_oled_row++;

    if (s_bluetooth_oled_row >= BLUETOOTH_OLED_ROWS) {
        OLED_Clear();
        s_bluetooth_oled_row = 0U;
    }
}

static void Test_BluetoothOLED_ShowByte(uint8_t data)
{
    if (data == (uint8_t)'\r') {
        return;
    }

    if (data == (uint8_t)'\n') {
        Test_BluetoothOLED_NewLine();
        return;
    }

    if ((data < (uint8_t)' ') || (data > (uint8_t)'~')) {
        data = (uint8_t)'?';
    }

    OLED_ShowChar(
        (uint8_t)(s_bluetooth_oled_column * BLUETOOTH_OLED_CHAR_WIDTH),
        s_bluetooth_oled_row, data, BLUETOOTH_OLED_FONT_SIZE);

    s_bluetooth_oled_column++;
    if (s_bluetooth_oled_column >= BLUETOOTH_OLED_COLUMNS) {
        Test_BluetoothOLED_NewLine();
    }
}

static void Test_DelayMs(uint32_t ms)
{
    DL_Common_delayCycles(ms * (CPUCLK_FREQ / 1000U));
}

/* 该函数在OLED指定位置显示一位小数的有符号数，参数x和row表示位置，value表示数值，integer_digits表示整数位数。 */
static void Test_OLED_ShowSignedTenth(
    uint8_t x, uint8_t row, float value, uint8_t integer_digits)
{
    int32_t scaled_value = (int32_t)(value * 10.0f);
    uint32_t magnitude;
    uint8_t number_x = (uint8_t)(x + 6U);
    uint8_t decimal_x = (uint8_t)(number_x + integer_digits * 6U);

    if (scaled_value < 0) {
        OLED_ShowChar(x, row, '-', UART_GYRO_GRAY_FONT_SIZE);
        magnitude = (uint32_t)(-scaled_value);
    } else {
        OLED_ShowChar(x, row, '+', UART_GYRO_GRAY_FONT_SIZE);
        magnitude = (uint32_t)scaled_value;
    }

    OLED_ShowNum(number_x, row, magnitude / 10U,
        integer_digits, UART_GYRO_GRAY_FONT_SIZE);
    OLED_ShowChar(decimal_x, row, '.', UART_GYRO_GRAY_FONT_SIZE);
    OLED_ShowNum((uint8_t)(decimal_x + 6U), row, magnitude % 10U,
        1U, UART_GYRO_GRAY_FONT_SIZE);
}

/* 该函数在OLED指定行按从第0路到第7路的顺序显示八路灰度数字量，参数row表示显示行，digital表示八位检测结果。 */
static void Test_OLED_ShowGrayDigital(uint8_t row, uint8_t digital)
{
    uint8_t channel;

    OLED_ShowString(0U, row, (uint8_t *)"GRAY:",
        UART_GYRO_GRAY_FONT_SIZE);
    OLED_ShowChar(30U, row, '(', UART_GYRO_GRAY_FONT_SIZE);
    for (channel = 0U; channel < 8U; channel++) {
        uint8_t bit = (uint8_t)((digital >> channel) & 0x01U);
        OLED_ShowChar((uint8_t)(36U + channel * 6U), row,
            (uint8_t)('0' + bit), UART_GYRO_GRAY_FONT_SIZE);
    }
    OLED_ShowChar(84U, row, ')', UART_GYRO_GRAY_FONT_SIZE);
}

void Test_OLED_Run(void)
{
    // OLED_Init();
    // OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"OLED TEST", 16);
    OLED_ShowString(0, 2, (uint8_t *)"MSPM0G3507", 16);
}

/* 该函数等待半秒后以固定PWM驱动左右轮持续向前。 */
void Test_Motor_Run(void)
{
    Motor_Stop();
    Test_DelayMs(500U);
    Motor_Forward(MOTOR_TEST_SPEED);

    while (1) {
    }
}

void Test_UART_Run(void)
{
    uint32_t count = 0U;

    UART_Printf("UART test start\r\n");

    while (1) {
        UART_Printf("UART TEST %lu: 0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n",
            (unsigned long)count);
        count++;
        Test_DelayMs(UART_TEST_PERIOD_MS);
    }
}

/* 该函数运行串口陀螺仪、八路灰度传感器和OLED联合显示测试，调用后不会返回。 */
/* 该函数校准并读取串口陀螺仪与八路灰度传感器，然后在SPI屏幕显示传感器数据和运行状态。 */
void Test_UARTGyroST7735_Run(void)
{
    float yaw_deg = 0.0f;
    float gyro_z_dps = 0.0f;
    uint8_t gray_digital = 0U;
    bool yaw_ready = false;
    bool calibration_ok;

    SYSCFG_DL_init();
    No_MCU_Ganv_Sensor_Init(&s_uart_gyro_gray_sensor,
        s_uart_gyro_gray_white, s_uart_gyro_gray_black);
    /* 该功能块清除并开启ADC0中断，使灰度采样完成后能够进入ADC中断服务函数。 */
    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    UARTGyro_Init();
    Motor_Stop();

    if (!ST7735S_Init(ST7735S_ROTATION_0)) {
        while (1) {
        }
    }

    (void)ST7735S_FillScreen(ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(16U, 48U, "CALIBRATING",
        ST7735S_COLOR_YELLOW, ST7735S_COLOR_BLACK);
    (void)ST7735S_DrawString(24U, 80U, "KEEP STILL",
        ST7735S_COLOR_RED, ST7735S_COLOR_BLACK);

    calibration_ok = UARTGyro_CalibrateBias();
    (void)ST7735S_DrawString(24U, 112U,
        calibration_ok ? "CAL OK" : "CAL FAIL",
        calibration_ok ? ST7735S_COLOR_GREEN : ST7735S_COLOR_RED,
        ST7735S_COLOR_BLACK);
    Test_DelayMs(1000U);

    (void)ST7735S_CarStatusInit();
    while (1) {
        if (UARTGyro_GetYaw(&yaw_deg)) {
            yaw_ready = true;
        }
        (void)UARTGyro_GetGyroZ(&gyro_z_dps);
        No_Mcu_Ganv_Sensor_Task_Without_tick(&s_uart_gyro_gray_sensor);
        gray_digital = Get_Digtal_For_User(&s_uart_gyro_gray_sensor);

        (void)ST7735S_CarStatusUpdate(yaw_deg, gyro_z_dps, gray_digital,
            yaw_ready ? "RUNNING" : "GYRO_WAIT");
        Test_DelayMs(UART_GYRO_GRAY_REFRESH_MS);
    }
}

void Test_UARTGyroGrayOLED_Run(void)
{
    float yaw_deg = 0.0f;
    float gyro_z_dps = 0.0f;
    uint8_t gray_digital;

    SYSCFG_DL_init();
    UARTGyro_Init();
    No_MCU_Ganv_Sensor_Init(&s_uart_gyro_gray_sensor,
        s_uart_gyro_gray_white, s_uart_gyro_gray_black);
    OLED_Init();
    OLED_Clear();

    /* 该功能块提示用户保持小车静止并调用现有串口陀螺仪零偏校准流程。 */
    OLED_ShowString(0U, 2U, (uint8_t *)"CALIBRATING",
        UART_GYRO_GRAY_FONT_SIZE);
    OLED_ShowString(0U, 4U, (uint8_t *)"KEEP STILL",
        UART_GYRO_GRAY_FONT_SIZE);
    UARTGyro_CalibrateBias();
    OLED_Clear();

    OLED_ShowString(0U, 0U, (uint8_t *)"GYRO GRAY TEST",
        UART_GYRO_GRAY_FONT_SIZE);
    OLED_ShowString(0U, 2U, (uint8_t *)"YAW:",
        UART_GYRO_GRAY_FONT_SIZE);
    OLED_ShowString(0U, 4U, (uint8_t *)"GZ :",
        UART_GYRO_GRAY_FONT_SIZE);

    while (1) {
        (void)UARTGyro_GetYaw(&yaw_deg);
        (void)UARTGyro_GetGyroZ(&gyro_z_dps);

        No_Mcu_Ganv_Sensor_Task_Without_tick(&s_uart_gyro_gray_sensor);
        gray_digital = Get_Digtal_For_User(&s_uart_gyro_gray_sensor);

        Test_OLED_ShowSignedTenth(30U, 2U, yaw_deg, 3U);
        Test_OLED_ShowSignedTenth(30U, 4U, gyro_z_dps, 4U);
        Test_OLED_ShowGrayDigital(6U, gray_digital);
        Test_DelayMs(UART_GYRO_GRAY_REFRESH_MS);
    }
}

void Test_BluetoothOLED_Init(void)
{
    Bluetooth_UART_Init();
    OLED_Init();

    s_bluetooth_oled_column = 0U;
    s_bluetooth_oled_row = 1U;
    OLED_ShowString(0U, 0U, (uint8_t *)"BT OLED READY", 8U);
    Bluetooth_UART_SendString("BT OLED READY\r\n");
}

void Test_BluetoothOLED_Process(void)
{
    uint8_t data[BLUETOOTH_OLED_READ_SIZE];
    uint16_t length;
    uint16_t index;

    length = Bluetooth_UART_Read(data, sizeof(data));
    for (index = 0U; index < length; index++) {
        Test_BluetoothOLED_ShowByte(data[index]);
    }
}

/* 该函数运行MaxiCam坐标接收与OLED显示测试，调用后不会返回。 */
void Test_MaxiCamOLED_Run(void)
{
    MaxiCam_Point point;

    SYSCFG_DL_init();
    MaxiCam_UART_Init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"MAXICAM DATA", 16U);
    OLED_ShowString(0U, 2U, (uint8_t *)"X:", 16U);
    OLED_ShowString(0U, 4U, (uint8_t *)"Y:", 16U);

    while (1) {
        MaxiCam_UART_Process();

        if (MaxiCam_UART_GetLatestPoint(&point)) {
            UART_Printf("(x:%u y:%u)\r\n",
                (unsigned int)point.x,
                (unsigned int)point.y);

            OLED_ShowNum(24U, 2U, point.x, 4U, 16U);
            OLED_ShowNum(24U, 4U, point.y, 4U, 16U);
        }
    }
}

/* 该函数运行蓝牙串口接收与OLED显示测试，调用后不会返回。 */
void Test_BluetoothOLED_Run(void)
{
    SYSCFG_DL_init();
    Test_BluetoothOLED_Init();

    while (1) {
        Test_BluetoothOLED_Process();
    }
}

/* 该函数初始化ST7735S为128乘160竖屏并准备从红色开始循环。 */
bool Test_ST7735S_ColorCycleInit(void)
{
    s_st7735s_color_index = 0U;
    return ST7735S_Init(ST7735S_ROTATION_0);
}

/* 该函数按红绿蓝白顺序刷新一次全屏颜色并保持一秒。 */
void Test_ST7735S_ColorCycleProcess(void)
{
    static const uint16_t colors[] = {
        ST7735S_COLOR_RED,
        ST7735S_COLOR_GREEN,
        ST7735S_COLOR_BLUE,
        ST7735S_COLOR_WHITE,
    };
    static const uint16_t text_colors[] = {
        ST7735S_COLOR_WHITE,
        ST7735S_COLOR_WHITE,
        ST7735S_COLOR_WHITE,
        ST7735S_COLOR_BLACK,
    };
    static const char *const color_names[] = {
        "RED",
        "GREEN",
        "BLUE",
        "WHITE",
    };
    uint16_t background = colors[s_st7735s_color_index];
    uint16_t foreground = text_colors[s_st7735s_color_index];

    (void)ST7735S_FillScreen(background);
    (void)ST7735S_DrawString(32U, 24U, "ST7735S", foreground, background);
    (void)ST7735S_DrawString(40U, 56U,
        color_names[s_st7735s_color_index], foreground, background);
    (void)ST7735S_DrawChar(32U, 88U, '#', foreground, background);
    (void)ST7735S_DrawInteger(48U, 88U, -123, foreground, background);
    Test_DelayMs(ST7735S_COLOR_HOLD_MS);

    s_st7735s_color_index++;
    if (s_st7735s_color_index >=
        (uint8_t)(sizeof(colors) / sizeof(colors[0]))) {
        s_st7735s_color_index = 0U;
    }
}

/* 该函数运行ST7735S红绿蓝白四色循环测试，调用后不会返回。 */
void Test_ST7735S_ColorCycleRun(void)
{
    SYSCFG_DL_init();

    if (!Test_ST7735S_ColorCycleInit()) {
        while (1) {
        }
    }

    while (1) {
        Test_ST7735S_ColorCycleProcess();
    }
}
