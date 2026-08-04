/**
 * @file    weapon_grip_app.h
 * @brief   端头开合 STS 舵机位置控制 App 配置与周期接口
 *
 * 舵机型号 STS3215，ID=6，挂载在 UART7，全双工接线。
 * 上位机命令和反馈使用 m；App 内部转换为舵机 raw 位置值。
 * 零点在 raw=2048（舵机固件 homing_offset 校准后）。
 * 实车联调时优先只调整 CENTER_RAW 和 RAW_PER_METER。
 */

#ifndef WEAPON_GRIP_APP_H
#define WEAPON_GRIP_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 用户可调参数。
 */
/* 位置控制周期，单位 ms。 */
#define WEAPON_GRIP_APP_CONTROL_PERIOD_MS  50U
/* 舵机反馈读取周期，单位 ms。小于 CONTROL_PERIOD_MS 以减少串口阻塞。 */
#define WEAPON_GRIP_APP_FEEDBACK_PERIOD_MS 20U
/* 舵机反馈超时判定离线的时间，单位 ms。必须大于 CONTROL_PERIOD_MS。 */
#define WEAPON_GRIP_APP_OFFLINE_TIMEOUT_MS 200U
/* 夹爪闭合位置对应的 raw 值（全开=2048，闭合=+90°=3072）。 */
#define WEAPON_GRIP_APP_CENTER_RAW         3072
/*
 * 米制位移 → raw 步数的换算系数，单位 raw/m。
 * 1024 步（-90°）/ 0.03 m ≈ 34133 raw/m。
 */
#define WEAPON_GRIP_APP_RAW_PER_METER      34133.33f
/* 舵机运动速度 */
#define WEAPON_GRIP_APP_MOVE_SPEED         1000U
/* 舵机运动加速度 */
#define WEAPON_GRIP_APP_MOVE_ACC           50U

void WeaponGripApp_Init(void);
void WeaponGripApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* WEAPON_GRIP_APP_H */
