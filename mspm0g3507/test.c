#include "test.h"

#include "bluetooth_uart.h"
#include "maxicam_uart.h"
#include "motor.h"
#include "Display/oled_hardware_i2c.h"
#include "Display/st7735s.h"
#include "ti_msp_dl_config.h"
#include "uart.h"

#include <stdint.h>

#define BLUETOOTH_OLED_FONT_SIZE       (8U)
#define BLUETOOTH_OLED_CHAR_WIDTH      (6U)
#define BLUETOOTH_OLED_COLUMNS         (21U)
#define BLUETOOTH_OLED_ROWS            (8U)
#define BLUETOOTH_OLED_READ_SIZE       (32U)
#define ST7735S_COLOR_HOLD_MS           (1000U)

static uint8_t s_bluetooth_oled_column;
static uint8_t s_bluetooth_oled_row;
static uint8_t s_st7735s_color_index;

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

static void Test_Motor_Step(int16_t left_speed, int16_t right_speed)
{
    Motor_Control(left_speed, right_speed);
    Test_DelayMs(MOTOR_RUN_TIME_MS);
    Motor_Stop();
    Test_DelayMs(MOTOR_STOP_TIME_MS);
}

void Test_OLED_Run(void)
{
    // OLED_Init();
    // OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"OLED TEST", 16);
    OLED_ShowString(0, 2, (uint8_t *)"MSPM0G3507", 16);
}

void Test_Motor_Run(void)
{
    Motor_Stop();
    Test_DelayMs(MOTOR_CYCLE_PAUSE_MS);

    while (1) {
        Test_Motor_Step(MOTOR_TEST_SPEED, 0);
        Test_Motor_Step(-MOTOR_TEST_SPEED, 0);
        Test_Motor_Step(0, MOTOR_TEST_SPEED);
        Test_Motor_Step(0, -MOTOR_TEST_SPEED);
        Test_Motor_Step(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED);
        Test_Motor_Step(-MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED);

        Test_DelayMs(MOTOR_CYCLE_PAUSE_MS);
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
