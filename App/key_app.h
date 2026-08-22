/**
 * @file    key_app.h
 * @brief   PA15 按键 App：100Hz 采样并发布到 USB 虚拟串口
 *
 * KeyApp 是按键逻辑值的唯一写入者；CommApp 通过 KeyApp_GetValue()
 * 只读获取最近一次采样值并打包进反馈帧。逻辑值：按下=1，松开=0。
 */

#ifndef KEY_APP_H
#define KEY_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 按键采样周期，单位 ms。10ms = 100Hz，无滤波。 */
#define KEY_APP_SAMPLE_PERIOD_MS 10U

void KeyApp_Init(void);
void KeyApp_RunPeriodic(void);

/** 只读接口：返回最近一次采样值（1=按下，0=松开）。 */
uint8_t KeyApp_GetValue(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_APP_H */
