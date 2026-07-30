/**
 * @file    comm_app.h
 * @brief   USB CDC 通讯 App 的公开邮箱与周期接口
 */

#ifndef COMM_APP_H
#define COMM_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 用户可配置项。协议帧布局等固定常量保留在 comm_app.c。 */
#define COMM_APP_FEEDBACK_PERIOD_MS 20U
#define COMM_APP_RX_BUFFER_SIZE     256U

/**
 * 上位机命令邮箱。
 *
 * CommApp 是唯一写入者；其他 App 只读。命令不会超时清除，sequence 在每
 * 收到一帧合法协议帧后递增。
 */
typedef struct {
    float chassis_vx_m_s;
    float chassis_vy_m_s;
    float chassis_wz_rad_s;
    float lift_front_position_m;
    float lift_rear_position_m;
    float kfs_lift_position_m;
    float kfs_root_rotate_rad;
    float kfs_tip_rotate_rad;
    float kfs_grip_position_m;
    float weapon_rotate_rad;
    float weapon_grip_position_m;
    uint32_t sequence;
    uint8_t valid;
} comm_app_command_t;

/**
 * 实际状态反馈邮箱。
 *
 * ChassisApp 只写 chassis_*；LiftApp 只写 lift_*；CommApp 只读并打包。
 */
typedef struct {
    float chassis_vx_m_s;
    float chassis_vy_m_s;
    float chassis_wz_rad_s;
    float lift_front_position_m;
    float lift_rear_position_m;
    float kfs_lift_position_m;
    float kfs_root_rotate_rad;
    float kfs_tip_rotate_rad;
    float kfs_grip_position_m;
    float weapon_rotate_rad;
    float weapon_grip_position_m;
    uint8_t chassis_valid;
    uint8_t lift_valid;
    uint8_t kfs_lift_valid;
    uint8_t kfs_root_rotate_valid;
    uint8_t kfs_tip_rotate_valid;
    uint8_t kfs_grip_valid;
} comm_app_feedback_t;

extern volatile comm_app_command_t g_comm_app_command;
extern volatile comm_app_feedback_t g_comm_app_feedback;

void CommApp_Init(void);
void CommApp_RunPeriodic(void);

/** USB CDC 接收回调调用；只把数据压入环形缓冲区。 */
void CommApp_OnUsbReceived(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* COMM_APP_H */
