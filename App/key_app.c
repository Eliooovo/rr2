/**
 * @file    key_app.c
 * @brief   PA15 按键所有权与 100Hz 周期采样
 */

#include "key_app.h"

#include "key.h"
#include "main.h"

/* ---- 私有变量 ---- */
static key_t     s_key;
static uint32_t  s_last_sample_ms;

void KeyApp_Init(void)
{
    s_key.config.port   = GPIOA;
    s_key.config.pin    = GPIO_PIN_15;
    s_key.config.invert = 1U;   /* 上拉 + 按键接 GND: 按下读到 0 → 取反为按下=1 */

    key_init(&s_key);
    s_last_sample_ms = 0U;
}

void KeyApp_RunPeriodic(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (s_last_sample_ms == 0U) {
        s_last_sample_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - s_last_sample_ms) < KEY_APP_SAMPLE_PERIOD_MS) {
        return;
    }
    s_last_sample_ms = now_ms;

    /* 100Hz 采样，无滤波 */
    (void)key_read(&s_key);
}

uint8_t KeyApp_GetValue(void)
{
    return key_is_pressed(&s_key);
}
