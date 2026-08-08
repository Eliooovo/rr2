/**
 * @file    chassis_app.h
 * @brief   四麦轮底盘 App 配置与周期接口
 */

#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 用户可配置项。 */
#define CHASSIS_APP_CONTROL_PERIOD_MS       1U
#define CHASSIS_APP_OFFLINE_TIMEOUT_MS      100U
#define CHASSIS_APP_CURRENT_LIMIT           12000.0f
#define CHASSIS_APP_SPEED_INTEGRAL_LIMIT    30000.0f

#define CHASSIS_APP_WHEEL_DIAMETER_M        0.14f
#define CHASSIS_APP_LENGTH_M                0.58f
#define CHASSIS_APP_WIDTH_M                 0.50f
#define CHASSIS_APP_MOTOR_REDUCTION_RATIO   19.0f

/* 上位机命令进入运动学解算前的执行倍率。 */
#define CHASSIS_APP_VX_SCALE                0.924f
#define CHASSIS_APP_VY_SCALE                1.15f
#define CHASSIS_APP_WZ_SCALE                0.9f

/* 坐标方向配置：只能使用 1.0f 或 -1.0f。 */
#define CHASSIS_APP_VX_DIRECTION            (-1.0f)
#define CHASSIS_APP_VY_DIRECTION            ( 1.0f)

typedef struct {
    uint8_t motor_id;    /**< C620 电调 ID，范围 1..8。 */
    int8_t direction;    /**< 电机安装方向，只能为 1 或 -1。 */
    float speed_kp;
    float speed_ki;
    float speed_kd;
} chassis_app_motor_config_t;

/**
 * 电机顺序固定为 RF、LF、LB、RB。
 *
 * 当前实车映射为 LF=1、LB=2、RB=3、RF=4。
 */
#define CHASSIS_APP_MOTOR_CONFIG_INIT                    \
    {                                                    \
        {4U,  1, 10.0f, 80.0f, 0.03f}, /* RF 右前 */    \
        {1U, -1, 10.0f, 80.0f, 0.03f}, /* LF 左前 */    \
        {2U, -1, 10.0f, 80.0f, 0.03f}, /* LB 左后 */    \
        {3U,  1, 10.0f, 80.0f, 0.03f}, /* RB 右后 */    \
    }

void ChassisApp_Init(void);
void ChassisApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_APP_H */
