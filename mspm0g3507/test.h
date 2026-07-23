#ifndef TEST_H
#define TEST_H

#include <stdbool.h>

#define TEST_MODE_ORIGINAL    0U
#define TEST_MODE_OLED        1U
#define TEST_MODE_MOTOR       2U
#define TEST_MODE_UART        3U

#ifndef MOTOR_TEST_SPEED
#define MOTOR_TEST_SPEED       400
#endif

#define UART_TEST_PERIOD_MS    1000U

void Test_OLED_Run(void);
void Test_Motor_Run(void);
void Test_UART_Run(void);

/* 该函数校准并读取串口陀螺仪，然后在SPI屏幕显示姿态数据和运行状态。 */
void Test_UARTGyroST7735_Run(void);

/* 该函数运行串口陀螺仪、八路灰度传感器和OLED联合显示测试，调用后不会返回。 */
void Test_UARTGyroGrayOLED_Run(void);

void Test_BluetoothOLED_Init(void);
void Test_BluetoothOLED_Process(void);

/* 该函数运行MaxiCam坐标接收与OLED显示测试，调用后不会返回。 */
void Test_MaxiCamOLED_Run(void);

/* 该函数运行蓝牙串口接收与OLED显示测试，调用后不会返回。 */
void Test_BluetoothOLED_Run(void);

/* 该函数初始化ST7735S四色测试并返回初始化是否成功。 */
bool Test_ST7735S_ColorCycleInit(void);

/* 该函数按红绿蓝白顺序刷新一次全屏颜色并保持一秒。 */
void Test_ST7735S_ColorCycleProcess(void);

/* 该函数运行ST7735S红绿蓝白四色循环测试，调用后不会返回。 */
void Test_ST7735S_ColorCycleRun(void);

#endif
