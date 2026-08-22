#include "key.h"

void key_init(key_t *key)
{
    if (key == NULL) {
        return;
    }

    key->state.pressed = KEY_RELEASED;

    if (key->config.port == NULL || key->config.pin == 0U) {
        return;  /* 未配置引脚，保持松开 */
    }

    /* 读取一次，得到上电初始状态 */
    (void)key_read(key);
}

uint8_t key_read(key_t *key)
{
    GPIO_PinState raw;

    if (key == NULL || key->config.port == NULL || key->config.pin == 0U) {
        return KEY_RELEASED;
    }

    raw = HAL_GPIO_ReadPin(key->config.port, key->config.pin);

    if (key->config.invert != 0U) {
        /* 上拉输入: 松开=1、按下=0，取反后按下=1 */
        key->state.pressed = (raw == GPIO_PIN_SET) ? KEY_RELEASED : KEY_PRESSED;
    } else {
        key->state.pressed = (raw == GPIO_PIN_SET) ? KEY_PRESSED : KEY_RELEASED;
    }

    return key->state.pressed;
}

uint8_t key_is_pressed(const key_t *key)
{
    if (key == NULL) {
        return KEY_RELEASED;
    }
    return key->state.pressed;
}
