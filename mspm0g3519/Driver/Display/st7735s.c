#include "st7735s.h"

#include <stddef.h>

#include "oledfont.h"
#include "ti_msp_dl_config.h"

#define ST7735S_CMD_SWRESET       ((uint8_t)0x01U)
#define ST7735S_CMD_SLPOUT        ((uint8_t)0x11U)
#define ST7735S_CMD_NORON         ((uint8_t)0x13U)
#define ST7735S_CMD_INVOFF        ((uint8_t)0x20U)
#define ST7735S_CMD_DISPON        ((uint8_t)0x29U)
#define ST7735S_CMD_CASET         ((uint8_t)0x2AU)
#define ST7735S_CMD_RASET         ((uint8_t)0x2BU)
#define ST7735S_CMD_RAMWR         ((uint8_t)0x2CU)
#define ST7735S_CMD_MADCTL        ((uint8_t)0x36U)
#define ST7735S_CMD_COLMOD        ((uint8_t)0x3AU)

#define ST7735S_TRANSFER_PIXELS   (128U)
#define ST7735S_TRANSFER_BYTES    (ST7735S_TRANSFER_PIXELS * 2U)

/* 该结构体保存每个显示方向对应的扫描参数、显存偏移和有效尺寸。 */
typedef struct {
    uint8_t madctl;
    uint8_t x_offset;
    uint8_t y_offset;
    uint16_t width;
    uint16_t height;
} ST7735S_RotationInfo;

static const ST7735S_RotationInfo s_rotation_table[4] = {
    {0xC8U, 2U, 1U, 128U, 160U},
    {0xA8U, 1U, 2U, 160U, 128U},
    {0x08U, 2U, 1U, 128U, 160U},
    {0x68U, 1U, 2U, 160U, 128U},
};

static uint8_t s_transfer_buffer[ST7735S_TRANSFER_BYTES];
static uint16_t s_width;
static uint16_t s_height;
static uint8_t s_rotation;
static bool s_initialized;

/* 该函数执行毫秒级阻塞延时，参数milliseconds指定延时时间。 */
static void ST7735S_DelayMs(uint32_t milliseconds)
{
    while (milliseconds > 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000U);
        milliseconds--;
    }
}

/* 该函数设置LCD片选电平，参数selected为true时选中屏幕。 */
static void ST7735S_Select(bool selected)
{
    if (selected) {
        DL_GPIO_clearPins(GPIO_LCD_CONTROL_PORT,
            GPIO_LCD_CONTROL_LCD_CS_PIN);
    } else {
        DL_GPIO_setPins(GPIO_LCD_CONTROL_PORT,
            GPIO_LCD_CONTROL_LCD_CS_PIN);
    }
}

/* 该函数阻塞发送一个SPI字节，参数data为待发送数据。 */
static void ST7735S_TransmitByte(uint8_t data)
{
    while (DL_SPI_isTXFIFOFull(SPI_LCD_INST)) {
    }
    DL_SPI_transmitData8(SPI_LCD_INST, data);
}

/* 该函数发送一段SPI数据，参数data为数据地址且length为字节数。 */
static void ST7735S_Transmit(const uint8_t *data, size_t length)
{
    size_t index;

    for (index = 0U; index < length; index++) {
        ST7735S_TransmitByte(data[index]);
    }
    while (DL_SPI_isBusy(SPI_LCD_INST)) {
    }
}

/* 该函数发送一个LCD命令，参数command为命令字节。 */
static void ST7735S_WriteCommand(uint8_t command)
{
    ST7735S_Select(true);
    DL_GPIO_clearPins(GPIO_LCD_CONTROL_PORT,
        GPIO_LCD_CONTROL_LCD_DC_PIN);
    ST7735S_Transmit(&command, 1U);
    ST7735S_Select(false);
}

/* 该函数发送LCD参数数据，参数data为数据地址且length为字节数。 */
static void ST7735S_WriteData(const uint8_t *data, size_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return;
    }

    ST7735S_Select(true);
    DL_GPIO_setPins(GPIO_LCD_CONTROL_PORT,
        GPIO_LCD_CONTROL_LCD_DC_PIN);
    ST7735S_Transmit(data, length);
    ST7735S_Select(false);
}

/* 该函数向LCD寄存器写入数据，参数command为命令且data和length指定参数内容。 */
static void ST7735S_WriteRegister(uint8_t command, const uint8_t *data,
    size_t length)
{
    ST7735S_WriteCommand(command);
    ST7735S_WriteData(data, length);
}

/* 该函数执行屏幕硬件复位并保持背光关闭。 */
static void ST7735S_HardwareReset(void)
{
    DL_GPIO_clearPins(GPIO_LCD_CONTROL_PORT,
        GPIO_LCD_CONTROL_LCD_BLK_PIN);
    DL_GPIO_clearPins(GPIO_LCD_CONTROL_PORT,
        GPIO_LCD_CONTROL_LCD_RES_PIN);
    ST7735S_DelayMs(100U);
    DL_GPIO_setPins(GPIO_LCD_CONTROL_PORT,
        GPIO_LCD_CONTROL_LCD_RES_PIN);
    ST7735S_DelayMs(100U);
}

/* 该函数设置显存写入窗口，参数x、y、width和height指定有效区域。 */
static void ST7735S_SetAddressWindow(uint16_t x, uint16_t y,
    uint16_t width, uint16_t height)
{
    const ST7735S_RotationInfo *info = &s_rotation_table[s_rotation];
    uint16_t x_start = (uint16_t)(x + info->x_offset);
    uint16_t x_end = (uint16_t)(x_start + width - 1U);
    uint16_t y_start = (uint16_t)(y + info->y_offset);
    uint16_t y_end = (uint16_t)(y_start + height - 1U);
    uint8_t address[4];

    address[0] = (uint8_t)(x_start >> 8U);
    address[1] = (uint8_t)x_start;
    address[2] = (uint8_t)(x_end >> 8U);
    address[3] = (uint8_t)x_end;
    ST7735S_WriteRegister(ST7735S_CMD_CASET, address, sizeof(address));

    address[0] = (uint8_t)(y_start >> 8U);
    address[1] = (uint8_t)y_start;
    address[2] = (uint8_t)(y_end >> 8U);
    address[3] = (uint8_t)y_end;
    ST7735S_WriteRegister(ST7735S_CMD_RASET, address, sizeof(address));

    ST7735S_WriteCommand(ST7735S_CMD_RAMWR);
}

/* 该函数连续写入同一种RGB565颜色，参数color为颜色且pixel_count为像素数。 */
static void ST7735S_WriteSolidColor(uint16_t color, size_t pixel_count)
{
    size_t index;
    uint8_t color_high = (uint8_t)(color >> 8U);
    uint8_t color_low = (uint8_t)color;

    for (index = 0U; index < ST7735S_TRANSFER_BYTES; index += 2U) {
        s_transfer_buffer[index] = color_high;
        s_transfer_buffer[index + 1U] = color_low;
    }

    while (pixel_count > 0U) {
        size_t batch_pixels = pixel_count;
        if (batch_pixels > ST7735S_TRANSFER_PIXELS) {
            batch_pixels = ST7735S_TRANSFER_PIXELS;
        }
        ST7735S_WriteData(s_transfer_buffer, batch_pixels * 2U);
        pixel_count -= batch_pixels;
    }
}

/* 该函数检查绘制区域是否位于当前屏幕范围内。 */
static bool ST7735S_IsAreaValid(uint16_t x, uint16_t y, uint16_t width,
    uint16_t height)
{
    if ((!s_initialized) || (width == 0U) || (height == 0U)) {
        return false;
    }

    return (((uint32_t)x + width) <= s_width) &&
        (((uint32_t)y + height) <= s_height);
}

/* 该函数写入显示方向参数，参数rotation必须是有效的四方向枚举值。 */
static void ST7735S_ApplyRotation(ST7735S_Rotation rotation)
{
    const ST7735S_RotationInfo *info =
        &s_rotation_table[(uint8_t)rotation];
    ST7735S_WriteRegister(ST7735S_CMD_MADCTL, &info->madctl, 1U);
    s_rotation = (uint8_t)rotation;
    s_width = info->width;
    s_height = info->height;
}

/* 该函数在初始化完成后修改显示方向，参数rotation指定新的屏幕方向。 */
bool ST7735S_SetRotation(ST7735S_Rotation rotation)
{
    if ((!s_initialized) ||
        ((uint32_t)rotation > (uint32_t)ST7735S_ROTATION_270)) {
        return false;
    }

    ST7735S_ApplyRotation(rotation);
    return true;
}

/* 该函数初始化ST7735S控制器，参数rotation指定上电后的显示方向。 */
bool ST7735S_Init(ST7735S_Rotation rotation)
{
    static const uint8_t frame_rate_1[] = {0x01U, 0x2CU, 0x2DU};
    static const uint8_t frame_rate_2[] = {0x01U, 0x2CU, 0x2DU};
    static const uint8_t frame_rate_3[] = {
        0x01U, 0x2CU, 0x2DU, 0x01U, 0x2CU, 0x2DU};
    static const uint8_t power_c0[] = {0xA2U, 0x02U, 0x84U};
    static const uint8_t power_c2[] = {0x0AU, 0x00U};
    static const uint8_t power_c3[] = {0x8AU, 0x2AU};
    static const uint8_t power_c4[] = {0x8AU, 0xEEU};
    static const uint8_t gamma_positive[] = {
        0x02U, 0x1CU, 0x07U, 0x12U, 0x37U, 0x32U, 0x29U, 0x2DU,
        0x29U, 0x25U, 0x2BU, 0x39U, 0x00U, 0x01U, 0x03U, 0x10U};
    static const uint8_t gamma_negative[] = {
        0x03U, 0x1DU, 0x07U, 0x06U, 0x2EU, 0x2CU, 0x29U, 0x2DU,
        0x2EU, 0x2EU, 0x37U, 0x3FU, 0x00U, 0x00U, 0x02U, 0x10U};
    uint8_t value;

    if ((uint32_t)rotation > (uint32_t)ST7735S_ROTATION_270) {
        return false;
    }

    s_initialized = false;
    s_rotation = (uint8_t)rotation;
    ST7735S_HardwareReset();

    ST7735S_WriteCommand(ST7735S_CMD_SWRESET);
    ST7735S_DelayMs(150U);
    ST7735S_WriteCommand(ST7735S_CMD_SLPOUT);
    ST7735S_DelayMs(120U);

    ST7735S_WriteRegister(0xB1U, frame_rate_1, sizeof(frame_rate_1));
    ST7735S_WriteRegister(0xB2U, frame_rate_2, sizeof(frame_rate_2));
    ST7735S_WriteRegister(0xB3U, frame_rate_3, sizeof(frame_rate_3));
    value = 0x07U;
    ST7735S_WriteRegister(0xB4U, &value, 1U);
    ST7735S_WriteRegister(0xC0U, power_c0, sizeof(power_c0));
    value = 0xC5U;
    ST7735S_WriteRegister(0xC1U, &value, 1U);
    ST7735S_WriteRegister(0xC2U, power_c2, sizeof(power_c2));
    ST7735S_WriteRegister(0xC3U, power_c3, sizeof(power_c3));
    ST7735S_WriteRegister(0xC4U, power_c4, sizeof(power_c4));
    value = 0x0EU;
    ST7735S_WriteRegister(0xC5U, &value, 1U);
    ST7735S_WriteCommand(ST7735S_CMD_INVOFF);

    ST7735S_ApplyRotation(rotation);
    value = 0x05U;
    ST7735S_WriteRegister(ST7735S_CMD_COLMOD, &value, 1U);
    ST7735S_WriteRegister(0xE0U, gamma_positive, sizeof(gamma_positive));
    ST7735S_WriteRegister(0xE1U, gamma_negative, sizeof(gamma_negative));

    ST7735S_WriteCommand(ST7735S_CMD_NORON);
    ST7735S_DelayMs(10U);
    ST7735S_WriteCommand(ST7735S_CMD_DISPON);
    ST7735S_DelayMs(100U);

    s_initialized = true;
    DL_GPIO_setPins(GPIO_LCD_CONTROL_PORT,
        GPIO_LCD_CONTROL_LCD_BLK_PIN);
    return true;
}

/* 该函数在参数x、y处使用color填充width乘height大小的矩形区域。 */
bool ST7735S_FillRect(uint16_t x, uint16_t y, uint16_t width,
    uint16_t height, uint16_t color)
{
    if (!ST7735S_IsAreaValid(x, y, width, height)) {
        return false;
    }

    ST7735S_SetAddressWindow(x, y, width, height);
    ST7735S_WriteSolidColor(color, (size_t)width * height);
    return true;
}

/* 该函数使用参数color指定的RGB565颜色填充整个有效显示区域。 */
bool ST7735S_FillScreen(uint16_t color)
{
    return ST7735S_FillRect(0U, 0U, s_width, s_height, color);
}

/* 该函数在参数x、y处绘制width乘height个RGB565像素数据colors。 */
bool ST7735S_DrawBitmap(uint16_t x, uint16_t y, uint16_t width,
    uint16_t height, const uint16_t *colors)
{
    size_t pixel_count;
    size_t pixel_offset = 0U;

    if ((colors == NULL) ||
        (!ST7735S_IsAreaValid(x, y, width, height))) {
        return false;
    }

    ST7735S_SetAddressWindow(x, y, width, height);
    pixel_count = (size_t)width * height;

    while (pixel_count > 0U) {
        size_t index;
        size_t batch_pixels = pixel_count;
        if (batch_pixels > ST7735S_TRANSFER_PIXELS) {
            batch_pixels = ST7735S_TRANSFER_PIXELS;
        }

        for (index = 0U; index < batch_pixels; index++) {
            uint16_t color = colors[pixel_offset + index];
            s_transfer_buffer[index * 2U] = (uint8_t)(color >> 8U);
            s_transfer_buffer[index * 2U + 1U] = (uint8_t)color;
        }
        ST7735S_WriteData(s_transfer_buffer, batch_pixels * 2U);
        pixel_offset += batch_pixels;
        pixel_count -= batch_pixels;
    }

    return true;
}

/* 该函数在参数x、y处将OLED格式的8乘16字模转换为RGB565字符像素。 */
bool ST7735S_DrawChar(uint16_t x, uint16_t y, char character,
    uint16_t foreground, uint16_t background)
{
    uint8_t character_index;
    uint8_t row;

    if ((character < ' ') || (character > '~')) {
        character = '?';
    }
    if (!ST7735S_IsAreaValid(x, y, 8U, 16U)) {
        return false;
    }

    character_index = (uint8_t)character - (uint8_t)' ';
    for (row = 0U; row < 16U; row++) {
        uint8_t column;
        uint8_t page = row / 8U;
        uint8_t bit_mask = (uint8_t)(1U << (row % 8U));

        for (column = 0U; column < 8U; column++) {
            uint16_t color = background;
            size_t pixel_index = (size_t)row * 8U + column;
            uint8_t font_byte =
                asc2_1608[character_index][page * 8U + column];

            if ((font_byte & bit_mask) != 0U) {
                color = foreground;
            }
            s_transfer_buffer[pixel_index * 2U] =
                (uint8_t)(color >> 8U);
            s_transfer_buffer[pixel_index * 2U + 1U] = (uint8_t)color;
        }
    }

    ST7735S_SetAddressWindow(x, y, 8U, 16U);
    ST7735S_WriteData(s_transfer_buffer, 8U * 16U * 2U);
    return true;
}

/* 该函数从参数x、y开始连续绘制8乘16 ASCII字符串text。 */
bool ST7735S_DrawString(uint16_t x, uint16_t y, const char *text,
    uint16_t foreground, uint16_t background)
{
    uint16_t cursor_x = x;

    if (text == NULL) {
        return false;
    }

    while (*text != '\0') {
        if (!ST7735S_DrawChar(cursor_x, y, *text, foreground, background)) {
            return false;
        }
        cursor_x = (uint16_t)(cursor_x + 8U);
        text++;
    }
    return true;
}

/* 该函数将参数value转换为十进制文本并在参数x、y处使用8乘16字模绘制。 */
bool ST7735S_DrawInteger(uint16_t x, uint16_t y, int32_t value,
    uint16_t foreground, uint16_t background)
{
    char text[12];
    uint8_t write_index = 0U;
    uint8_t left;
    uint8_t right;
    uint32_t magnitude;

    if (value < 0) {
        text[write_index++] = '-';
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    left = write_index;
    do {
        text[write_index++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude > 0U);

    right = (uint8_t)(write_index - 1U);
    while (left < right) {
        char temporary = text[left];
        text[left] = text[right];
        text[right] = temporary;
        left++;
        right--;
    }
    text[write_index] = '\0';

    return ST7735S_DrawString(x, y, text, foreground, background);
}

/* 该函数返回当前显示方向下的有效屏幕宽度。 */
uint16_t ST7735S_GetWidth(void)
{
    return s_width;
}

/* 该函数返回当前显示方向下的有效屏幕高度。 */
uint16_t ST7735S_GetHeight(void)
{
    return s_height;
}
