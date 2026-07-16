#ifndef KEY_INPUT_H
#define KEY_INPUT_H

#include <stdint.h>

#define KEY_INPUT_NONE    0u
#define KEY_INPUT_KEY1    1u
#define KEY_INPUT_KEY2    2u
#define KEY_INPUT_KEY3    3u

/* 初始化按键检测模块的内部状态。 */
void KeyInput_Init(void);

/* 每10ms扫描一次按键并完成软件消抖。 */
void KeyInput_Process10ms(void);

/* 获取一次已经确认的按键按下事件。 */
uint8_t KeyInput_GetPressEvent(void);

#endif