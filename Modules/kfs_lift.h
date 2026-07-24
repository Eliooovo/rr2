/**
 * @file    kfs_lift.h
 * @brief   RS00 KFS 抬升模块
 *
 * KFS 高度目标使用米，电机位置目标使用弧度。模块不使用动态内存，
 * 调用方负责分配并保留一个 kfs_lift_t 对象。
 */

#ifndef KFS_LIFT_H
#define KFS_LIFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "rs_motor.h"

#define KFS_LIFT_CONFIRM_ZERO_COMMAND_M (-1.0f)

typedef enum {
    KFS_LIFT_STATUS_OK = 0,
    KFS_LIFT_STATUS_INVALID_ARGUMENT,
    KFS_LIFT_STATUS_INVALID_CONFIG,
    KFS_LIFT_STATUS_NOT_INITIALIZED,
    KFS_LIFT_STATUS_NOT_ZEROED,
    KFS_LIFT_STATUS_FEEDBACK_OFFLINE,
    KFS_LIFT_STATUS_MOTOR_ERROR
} kfs_lift_status_t;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t motor_id;                  /**< RS00 电机 ID，范围 1..0x7f。 */
    uint8_t master_id;                 /**< 主机 ID，当前约定为 0xfd。 */
    rs_motor_type_t motor_type;        /**< RS00 对应的 RobStride 参数表编号。 */
    uint32_t offline_timeout_ms;       /**< 反馈超时阈值，单位 ms。 */
    float max_height_m;                /**< 最大目标高度，单位 m。 */
    float meters_per_revolution_m;     /**< 电机轴转一圈对应的直线上升量，单位 m。 */
    float speed_rad_s;                 /**< PP 目标速度，单位 rad/s。 */
    float acceleration_rad_s2;         /**< PP 目标加速度，单位 rad/s^2。 */
    int8_t direction;                  /**< 电机正方向对应的高度方向，取 +1 或 -1。 */
    float position_command_limit_rad;  /**< PP 多圈位置目标绝对值上限，单位 rad。 */
} kfs_lift_config_t;

typedef struct {
    uint8_t initialized;
    uint8_t wake_command_sent;
    uint8_t zeroed;
    uint8_t ready;
    uint8_t feedback_valid;
    uint8_t target_pending;
    float target_height_m;
    float actual_height_m;
    uint32_t last_feedback_ms;
} kfs_lift_state_t;

typedef struct {
    uint8_t feedback_timeout;
    uint8_t can_tx_error;
    uint8_t motor_fault;
    uint8_t invalid_feedback;
} kfs_lift_fault_t;

typedef struct {
    rs_motor_t motor;
    float zero_angle_rad;
    float accumulated_angle_rad;
    float previous_feedback_angle_rad;
    float last_command_angle_rad;
    uint32_t processed_feedback_count;
    uint8_t feedback_unwrap_initialized;
    uint8_t command_initialized;
} kfs_lift_internal_t;

typedef struct kfs_lift {
    kfs_lift_config_t config;
    kfs_lift_state_t state;
    kfs_lift_fault_t fault;
    kfs_lift_internal_t internal;
} kfs_lift_t;

/** 初始化 RS00 驱动和 KFS 抬升状态，不会发送运动命令。 */
kfs_lift_status_t KfsLift_Init(kfs_lift_t *lift);

/** 设置绝对目标高度，单位 m，合法范围为 0.0 m 至 max_height_m。 */
kfs_lift_status_t KfsLift_SetTargetHeightM(kfs_lift_t *lift, float height_m);

/** 将当前反馈电机位置记录为高度 0.0 m，并使模块进入可运动状态。 */
kfs_lift_status_t KfsLift_ConfirmZero(kfs_lift_t *lift);

/**
 * 在主循环中周期调用：先发送一次使能唤醒，再处理反馈累计、超时保护和
 * 待发送的 PP 目标。调用前必须已启动 config.hfdcan 对应的 FDCAN 外设。
 */
void KfsLift_RunPeriodic(kfs_lift_t *lift);

/** 停止并失能 RS00 电机。 */
void KfsLift_Stop(kfs_lift_t *lift);

/** 获取相对零点的实际高度，单位 m；未归零或反馈失效时返回 0.0 m。 */
float KfsLift_GetActualHeightM(const kfs_lift_t *lift);

/** 返回模块是否已经归零且反馈在线。 */
uint8_t KfsLift_IsReady(const kfs_lift_t *lift);

#ifdef __cplusplus
}
#endif

#endif /* KFS_LIFT_H */
