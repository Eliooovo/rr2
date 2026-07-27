/**
 * @file    kfs_lift_app.h
 * @brief   KFS 单 RobStride 电机抬升 App 配置与周期接口。
 */

#ifndef KFS_LIFT_APP_H
#define KFS_LIFT_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 用户可调参数。
 *
 * 上位机命令和反馈使用 m；App 内部转换为 RobStride 连续多圈位置 rad。
 * 实车联调时优先只调整 METERS_PER_MOTOR_RAD 和 DIRECTION。
 */
#define KFS_LIFT_APP_CONTROL_PERIOD_MS             10U
#define KFS_LIFT_APP_OFFLINE_TIMEOUT_MS            100U
/* 电机连续位置转动 1 rad 对应的抬升位移，单位 m/rad。 电机转一圈，直线移动4mm*/
#define KFS_LIFT_APP_METERS_PER_MOTOR_RAD          0.000636619772 
/* 米制正方向到电机正方向的映射，只允许 1.0 或 -1.0。 */
#define KFS_LIFT_APP_DIRECTION                     (1.0)
/* 连续多圈位置外环的速度、加速度和 P 控制参数，单位见宏名。 */
#define KFS_LIFT_APP_MAX_SPEED_RAD_S               25.0f
#define KFS_LIFT_APP_ACCELERATION_RAD_S2           800.0f
#define KFS_LIFT_APP_CURRENT_LIMIT_A               2.0f
#define KFS_LIFT_APP_POSITION_KP_S_1               3.0f
#define KFS_LIFT_APP_POSITION_TOLERANCE_RAD        0.0f

void KfsLiftApp_Init(void);
void KfsLiftApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* KFS_LIFT_APP_H */
