/**
 * @file    kfs_rotate_app.h
 * @brief   KFS 旋转关节位置控制 App（根部 PP + 末端 CSP）配置与周期接口
 */

#ifndef KFS_ROTATE_APP_H
#define KFS_ROTATE_APP_H

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
/* 位置控制命令检查周期，单位 ms。 */
#define KFS_ROTATE_APP_CTRL_PERIOD_MS    10U
/* 根部/末端反馈超时，单位 ms；末端实测反馈间隔峰值为 111 ms。 */
#define KFS_ROTATE_APP_ROOT_OFFLINE_TIMEOUT_MS 100U
#define KFS_ROTATE_APP_TIP_OFFLINE_TIMEOUT_MS  200U
/* PP/CSP 速度上限，0.785 rad/s ≈ 45 °/s。 */
#define KFS_ROTATE_APP_MAX_SPEED_RAD_S   0.785f
/* 根部 PP 模式加速度，低于手册默认值以抑制超调。 */
#define KFS_ROTATE_APP_ROOT_PP_ACCELERATION_RAD_S2 1.0f
/* RS03 驱动器额定相电流峰值，用于根部 PP 带载起动。 */
#define KFS_ROTATE_APP_ROOT_PP_CURRENT_LIMIT_A     12.0f

void KfsRotateApp_Init(void);
void KfsRotateApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* KFS_ROTATE_APP_H */
