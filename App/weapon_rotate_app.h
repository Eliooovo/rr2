/**
 * @file    weapon_rotate_app.h
 * @brief   端头旋转关节限流 CSP 位置控制 App 配置与周期接口
 */

#ifndef WEAPON_ROTATE_APP_H
#define WEAPON_ROTATE_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 用户可调参数。
 *
 * 上位机命令和反馈使用 rad；App 内部直接使用单圈角度，
 * 不使用多圈累计，因此依赖机械零位。
 */
/* CSP 控制帧下发周期，单位 ms。 */
#define WEAPON_ROTATE_APP_CTRL_PERIOD_MS    10U
/* 电机反馈超时判定离线的时间，单位 ms。 */
#define WEAPON_ROTATE_APP_OFFLINE_TIMEOUT_MS 100U
/* RS05 CSP 电流上限，单位 A；实车按所需顶紧力和温升调整。 */
#define WEAPON_ROTATE_APP_CSP_CURRENT_LIMIT_A 2.0f
/* CSP 速度上限，0.785 rad/s ≈ 45 °/s。 */
#define WEAPON_ROTATE_APP_MAX_SPEED_RAD_S   0.785f

void WeaponRotateApp_Init(void);
void WeaponRotateApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* WEAPON_ROTATE_APP_H */
