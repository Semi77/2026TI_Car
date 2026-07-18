#include "key_input.h"
#include "ti_msp_dl_config.h"

#define KEY_DEBOUNCE_COUNT  3u

typedef struct
{
    uint8_t key_value;
    GPIO_Regs *port;
    uint32_t pin;
    uint8_t last_sample;
    uint8_t stable_state;
    uint8_t debounce_count;
} KeyInput_State;

static KeyInput_State g_key_states[] =
{
    {KEY_INPUT_KEY1, KEY_INPUT_A_PORT, KEY_INPUT_A_KEY1_PIN, 1u, 1u, 0u},
    {KEY_INPUT_KEY2, KEY_INPUT_A_PORT, KEY_INPUT_A_KEY2_PIN, 1u, 1u, 0u},
    {KEY_INPUT_KEY3, KEY_INPUT_B_PORT, KEY_INPUT_B_KEY3_PIN, 1u, 1u, 0u},
};

static uint8_t g_key_event = KEY_INPUT_NONE;

/* 读取指定按键当前是否处于按下状态。 */
static uint8_t KeyInput_ReadPressed(const KeyInput_State *key)
{
    return ((DL_GPIO_readPins(key->port, key->pin) & key->pin) == 0u) ? 1u : 0u;
}

/* 初始化按键检测模块的内部状态。 */
void KeyInput_Init(void)
{
    uint32_t i;

    g_key_event = KEY_INPUT_NONE;

    for (i = 0u; i < (sizeof(g_key_states) / sizeof(g_key_states[0])); i++)
    {
        g_key_states[i].last_sample = KeyInput_ReadPressed(&g_key_states[i]);
        g_key_states[i].stable_state = g_key_states[i].last_sample;
        g_key_states[i].debounce_count = 0u;
    }
}

/* 每10ms扫描一次按键并完成软件消抖。 */
void KeyInput_Process10ms(void)
{
    uint32_t i;

    for (i = 0u; i < (sizeof(g_key_states) / sizeof(g_key_states[0])); i++)
    {
        uint8_t sample = KeyInput_ReadPressed(&g_key_states[i]);

        if (sample == g_key_states[i].last_sample)
        {
            if (g_key_states[i].debounce_count < KEY_DEBOUNCE_COUNT)
            {
                g_key_states[i].debounce_count++;
            }
        }
        else
        {
            g_key_states[i].last_sample = sample;
            g_key_states[i].debounce_count = 1u;
        }

        if (g_key_states[i].debounce_count >= KEY_DEBOUNCE_COUNT)
        {
            if (sample != g_key_states[i].stable_state)
            {
                g_key_states[i].stable_state = sample;

                if (sample != 0u)
                {
                    g_key_event = g_key_states[i].key_value;
                }
            }
        }
    }
}

/* 获取一次已经确认的按键按下事件。 */
uint8_t KeyInput_GetPressEvent(void)
{
    uint8_t event = g_key_event;

    g_key_event = KEY_INPUT_NONE;

    return event;
}
