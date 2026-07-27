/**
 * @file    lift_app.h
 * @brief   四电机升降 App 配置与周期接口
 */

#ifndef LIFT_APP_H
#define LIFT_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 用户可配置项。 */
#define LIFT_APP_CONTROL_PERIOD_MS             1U
#define LIFT_APP_OFFLINE_TIMEOUT_MS            100U
#define LIFT_APP_POSITION_INTEGRAL_LIMIT       0.0f
#define LIFT_APP_SPEED_INTEGRAL_LIMIT          30000.0f
/* 同组双电机同步 PD。 */
#define LIFT_APP_SYNC_KP_RPM_PER_DEG            1.0f
#define LIFT_APP_SYNC_KD_RPM_S_PER_DEG          0.0f
#define LIFT_APP_SYNC_MAX_CORRECTION_RPM        100.0f
/* 前后组平均位置同步 PD，以及四机模式下的单电机最终修正限幅。 */
#define LIFT_APP_FOUR_SYNC_KP_RPM_PER_DEG       12.0f
#define LIFT_APP_FOUR_SYNC_KD_RPM_S_PER_DEG     0.0f
#define LIFT_APP_FOUR_SYNC_MAX_CORRECTION_RPM   200.0f
/* 减速器输出轴转动 1 rad 对应抬升直线位移 0.01242 m。 */
#define LIFT_APP_METERS_PER_OUTPUT_RAD           0.0175
#define LIFT_APP_MOTOR_REDUCTION_RATIO           19.0

typedef struct {
    uint8_t motor_id;          /**< C620 电调 ID，范围 1..8。 */
    int8_t direction;          /**< 位置正方向，只能为 1 或 -1。 */
    float position_kp;
    float position_ki;
    float position_kd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float max_speed_rpm;
    float current_limit;
} lift_app_motor_config_t;

/*
 * 电机顺序固定为 FRONT_A、FRONT_B、REAR_A、REAR_B。
 * direction 为 1 表示不反向，为 -1 表示反向；当前两组均为 A 反向、B 不反向。
 */
#define LIFT_APP_MOTOR_CONFIG_INIT                                     \
    {                                                                  \
        {1U, -1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 1800.0f, 5000.0f}, \
        {2U, 1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 1800.0f, 5000.0f}, \
        {3U, -1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 1800.0f, 5000.0f}, \
        {4U, 1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 1800.0f, 5000.0f}, \
    }

void LiftApp_Init(void);
void LiftApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* LIFT_APP_H */
