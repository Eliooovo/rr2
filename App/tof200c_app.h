/**
 * @file    tof200c_app.h
 * @brief   TOF200C 设备所有权、周期调度与 HAL 回调转发。
 */

#ifndef TOF200C_APP_H
#define TOF200C_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32h7xx_hal.h"

/** TOF200C 在线异步处理周期，单位 ms；掉线后不自动恢复。 */
#define TOF200C_APP_PROCESS_PERIOD_MS 20U

void Tof200cApp_Init(void);
void Tof200cApp_RunPeriodic(void);

void Tof200cApp_OnExtiCallback(uint16_t gpio_pin);
void Tof200cApp_OnI2cMemRxComplete(I2C_HandleTypeDef *hi2c);
void Tof200cApp_OnI2cMemTxComplete(I2C_HandleTypeDef *hi2c);
void Tof200cApp_OnI2cError(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* TOF200C_APP_H */
