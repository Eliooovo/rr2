#ifndef KEY_H
#define KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/** 按键逻辑电平。 */
#define KEY_PRESSED  1U
#define KEY_RELEASED 0U

/**
 * 按键配置，调用方在 init 前填写。
 *
 * 上拉输入 + 按键接 GND 时，引脚松开读到 1、按下读到 0；若希望逻辑值
 * 「按下=1」，把 invert 置 1（逻辑值 = 原始电平取反）。
 */
typedef struct {
    GPIO_TypeDef *port;   /**< GPIO 端口，如 GPIOA。                     */
    uint16_t      pin;    /**< GPIO 引脚，如 GPIO_PIN_15。               */
    uint8_t       invert; /**< 1 = 逻辑值取反，0 = 与引脚电平一致。       */
} key_config_t;

/** 按键运行状态。 */
typedef struct {
    uint8_t pressed;      /**< 最近一次读取的逻辑值：1=按下，0=松开。     */
} key_state_t;

/** 按键对象，由调用方分配。 */
typedef struct {
    key_config_t config;
    key_state_t  state;
} key_t;

/**
 * @brief 初始化按键对象并读取一次初始状态。
 *
 * @param key 按键对象指针，config 成员必须已填写。
 */
void key_init(key_t *key);

/**
 * @brief 读取一次引脚并返回逻辑值（按下=1/松开=0）。
 *
 * @param key 按键对象指针。
 * @return KEY_PRESSED 或 KEY_RELEASED。
 */
uint8_t key_read(key_t *key);

/**
 * @brief 返回最近一次读取的逻辑值，不重新读取引脚。
 *
 * @param key 按键对象指针。
 * @return KEY_PRESSED 或 KEY_RELEASED。
 */
uint8_t key_is_pressed(const key_t *key);

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */
