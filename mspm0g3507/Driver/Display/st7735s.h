#ifndef ST7735S_H
#define ST7735S_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 该枚举用于选择ST7735S屏幕的四种显示方向。 */
typedef enum {
    ST7735S_ROTATION_0 = 0,
    ST7735S_ROTATION_90,
    ST7735S_ROTATION_180,
    ST7735S_ROTATION_270
} ST7735S_Rotation;

/* 这些宏定义提供常用的RGB565颜色值。 */
#define ST7735S_COLOR_BLACK       ((uint16_t)0x0000U)
#define ST7735S_COLOR_WHITE       ((uint16_t)0xFFFFU)
#define ST7735S_COLOR_RED         ((uint16_t)0xF800U)
#define ST7735S_COLOR_GREEN       ((uint16_t)0x07E0U)
#define ST7735S_COLOR_BLUE        ((uint16_t)0x001FU)
#define ST7735S_COLOR_YELLOW      ((uint16_t)0xFFE0U)
#define ST7735S_COLOR_CYAN        ((uint16_t)0x07FFU)
#define ST7735S_COLOR_MAGENTA     ((uint16_t)0xF81FU)

/* 该函数初始化ST7735S屏幕，参数rotation指定屏幕方向。 */
bool ST7735S_Init(ST7735S_Rotation rotation);

/* 该函数修改屏幕方向，参数rotation指定新的屏幕方向。 */
bool ST7735S_SetRotation(ST7735S_Rotation rotation);

/* 该函数使用参数color指定的RGB565颜色填充整个屏幕。 */
bool ST7735S_FillScreen(uint16_t color);

/* 该函数在参数x、y处使用RGB565颜色color填充指定宽高的矩形。 */
bool ST7735S_FillRect(uint16_t x, uint16_t y, uint16_t width,
    uint16_t height, uint16_t color);

/* 该函数在参数x、y处绘制width乘height个RGB565像素数据colors。 */
bool ST7735S_DrawBitmap(uint16_t x, uint16_t y, uint16_t width,
    uint16_t height, const uint16_t *colors);

/* 该函数在参数x、y处使用8乘16字模及指定前景色和背景色绘制字符character。 */
bool ST7735S_DrawChar(uint16_t x, uint16_t y, char character,
    uint16_t foreground, uint16_t background);

/* 该函数在参数x、y处使用8乘16字模及指定颜色绘制字符串text。 */
bool ST7735S_DrawString(uint16_t x, uint16_t y, const char *text,
    uint16_t foreground, uint16_t background);

/* 该函数在参数x、y处使用8乘16字模及指定颜色绘制有符号整数value。 */
bool ST7735S_DrawInteger(uint16_t x, uint16_t y, int32_t value,
    uint16_t foreground, uint16_t background);

/* 该函数清屏并绘制小车Yaw、角速度、灰度和运行状态标签。 */
bool ST7735S_CarStatusInit(void);

/* 该函数刷新小车状态数据，参数yaw_deg和gyro_z_dps表示姿态数据，gray_digital表示八路灰度位图，state表示运行状态。 */
bool ST7735S_CarStatusUpdate(float yaw_deg, float gyro_z_dps,
    uint8_t gray_digital, const char *state);

/* 该函数返回当前显示方向下的屏幕宽度。 */
uint16_t ST7735S_GetWidth(void);

/* 该函数返回当前显示方向下的屏幕高度。 */
uint16_t ST7735S_GetHeight(void);

#ifdef __cplusplus
}
#endif

#endif
