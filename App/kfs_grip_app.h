/**
 * @file    kfs_grip_app.h
 * @brief   KFS 夹爪开合 RS05 MIT 控制 App 配置与周期接口
 *
 * 电机型号 RS05 (RS_MOTOR_TYPE_5)，CAN ID=4，挂载在 FDCAN3。
 * 使用 MIT 阻抗控制（位置 + 速度阻尼），依赖机械零位。
 *
 * 上位机命令和反馈使用 m；App 内部转换为电机单圈角度 rad。
 * 实车联调时优先只调整 METERS_PER_MOTOR_RAD 和 DIRECTION。
 */

#ifndef KFS_GRIP_APP_H
#define KFS_GRIP_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 用户可调参数。
 */
/* MIT 控制帧下发周期，单位 ms。 */
#define KFS_GRIP_APP_CTRL_PERIOD_MS       10U
/* 电机反馈超时判定离线的时间，单位 ms。必须大于 CTRL_PERIOD_MS。 */
#define KFS_GRIP_APP_OFFLINE_TIMEOUT_MS    100U
/*
 * 电机转动 1 rad 对应的夹爪直线位移，单位 m/rad。
 * 实测：-0.79 rad 时直线移动 2 cm → 0.02 m / 0.79 rad。
 */
#define KFS_GRIP_APP_METERS_PER_MOTOR_RAD  0.0253164556962
/*
 * 米制正方向到电机正方向的映射，只允许 1.0 或 -1.0。
 * -1.0：rad 负 = 打开方向 = 米正方向。
 */
#define KFS_GRIP_APP_DIRECTION             (-1.0)
/* MIT 位置刚度 Kp。 */
#define KFS_GRIP_APP_KP                    15.0f
/* MIT 速度阻尼 Kd。 */
#define KFS_GRIP_APP_KD                     0.5f

void KfsGripApp_Init(void);
void KfsGripApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* KFS_GRIP_APP_H */
