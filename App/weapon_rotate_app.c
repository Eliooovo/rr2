/**
 * @file    weapon_rotate_app.c
 * @brief   端头旋转关节限流 CSP 位置控制（限速 ~45°/s）。
 *
 * 管理一个旋转关节：端头（RS05, id=5）。
 * 上位机通过 USB 协议下发目标角度（单位 rad），App 在收到首个有效命令
 * 后使能电机，并以不超过 WEAPON_ROTATE_APP_MAX_SPEED_RAD_S 的速度平滑到位。
 * 反馈为单圈角度（依赖机械零位），不使用多圈累计。
 */

#include "weapon_rotate_app.h"

#include <float.h>

#include "comm_app.h"
#include "fdcan.h"
#include "main.h"
#include "rs_motor.h"

/* ---- 旋转关节描述符 ---- */

typedef struct {
    rs_motor_t motor;                     /* 电机实例 */

    /* 上位机命令/反馈字段绑定（Init 时设置，之后只读/只写）。 */
    const volatile float *command_rad;    /* 目标角度 rad */
    volatile float *feedback_rad;         /* 当前角度 rad */
    volatile uint8_t *feedback_valid;     /* 反馈有效标志 */

    /* 运行时状态 */
    float    target_position_rad;         /* 当前目标位置 */
    uint8_t  target_received;             /* 是否收到过有效命令 */
    uint32_t last_command_sequence;       /* 上次处理的命令序列号 */
    uint32_t last_ctrl_ms;                /* 上次下发 CSP 的时间戳 */
} weapon_rotate_joint_t;

/* ---- 关节实例 ---- */

static weapon_rotate_joint_t s_joint;
static uint8_t s_initialized;

/* ---- 辅助函数 ---- */

static uint8_t WeaponRotateIsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static void WeaponRotateJoint_ClearFeedback(weapon_rotate_joint_t *joint)
{
    *joint->feedback_rad = 0.0f;
    *joint->feedback_valid = 0U;
}

static void WeaponRotateJoint_FailAndDisable(weapon_rotate_joint_t *joint)
{
    WeaponRotateJoint_ClearFeedback(joint);
    (void)rs_motor_disable(&joint->motor);
}

static uint8_t WeaponRotateJoint_FeedbackIsRecent(const weapon_rotate_joint_t *joint,
                                                   uint32_t now_ms)
{
    return (joint->motor.state.online != 0U &&
            (uint32_t)(now_ms - joint->motor.state.last_update_ms) <
                WEAPON_ROTATE_APP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

/* 检查上位机新命令：sequence 递增说明收到新帧，更新目标。 */
static void WeaponRotateJoint_UpdateTarget(weapon_rotate_joint_t *joint)
{
    uint32_t sequence = g_comm_app_command.sequence;
    float target_rad;

    if (g_comm_app_command.valid == 0U ||
        sequence == joint->last_command_sequence) {
        return;
    }
    joint->last_command_sequence = sequence;

    target_rad = *joint->command_rad;
    if (WeaponRotateIsFinite(target_rad) == 0U) {
        /* 非有限值不覆盖旧目标，保持上一次有效角度命令。 */
        return;
    }

    if (joint->target_received == 0U ||
        target_rad != joint->target_position_rad) {
        joint->target_position_rad = target_rad;
    }
    joint->target_received = 1U;
}

static void WeaponRotateJoint_UpdateFeedback(weapon_rotate_joint_t *joint,
                                              uint32_t now_ms)
{
    rs_motor_feedback_t fb;
    float angle_rad;

    if (WeaponRotateJoint_FeedbackIsRecent(joint, now_ms) == 0U ||
        rs_motor_get_feedback(&joint->motor, &fb) != RS_MOTOR_OK) {
        WeaponRotateJoint_ClearFeedback(joint);
        return;
    }

    angle_rad = fb.angle_rad;
    if (WeaponRotateIsFinite(angle_rad) == 0U) {
        WeaponRotateJoint_ClearFeedback(joint);
        return;
    }

    *joint->feedback_rad = angle_rad;
    *joint->feedback_valid = 1U;
}

static void WeaponRotateJoint_RunPeriodic(weapon_rotate_joint_t *joint,
                                           uint32_t now_ms)
{
    rs_motor_status_t status;

    /* 离线判断由驱动按反馈时间戳处理，不能按主循环次数判断。 */
    status = rs_motor_update(&joint->motor, now_ms);
    if (status != RS_MOTOR_OK) {
        if (status == RS_MOTOR_ERROR_FDCAN_TX) {
            return;
        }
        WeaponRotateJoint_FailAndDisable(joint);
        return;
    }

    /* 1. 更新反馈到上位机邮箱。 */
    WeaponRotateJoint_UpdateFeedback(joint, now_ms);

    /* 2. 按周期检查命令、下发 CSP。 */
    if ((uint32_t)(now_ms - joint->last_ctrl_ms) <
        WEAPON_ROTATE_APP_CTRL_PERIOD_MS) {
        return;
    }
    joint->last_ctrl_ms = now_ms;

    WeaponRotateJoint_UpdateTarget(joint);

    /*
     * 安全：未收到任何有效上位机命令前不使能电机，
     * 避免上电后在无目标的情况下自运动。
     */
    if (joint->target_received == 0U) {
        return;
    }

    /*
     * 限流 CSP 位置控制：
     *   电流限制只在首次配置、参数变化或离线恢复时写入；
     *   正常周期调用只重发位置目标，同时作为 CAN keepalive。
     *   顶住机械限位时，位置环输出受 current_limit 限制。
     */
    status = rs_motor_csp_position_control_limited(
        &joint->motor,
        WEAPON_ROTATE_APP_CSP_CURRENT_LIMIT_A,
        WEAPON_ROTATE_APP_MAX_SPEED_RAD_S,
        joint->target_position_rad);
    if (status != RS_MOTOR_OK) {
        if (status == RS_MOTOR_ERROR_FDCAN_TX) {
            return;
        }
        WeaponRotateJoint_FailAndDisable(joint);
        return;
    }
}

/* ---- 关节初始化 ---- */

static void WeaponRotateJoint_Init(weapon_rotate_joint_t *joint,
                                    uint8_t motor_id,
                                    rs_motor_type_t motor_type,
                                    const volatile float *command_rad,
                                    volatile float *feedback_rad,
                                    volatile uint8_t *feedback_valid)
{
    joint->command_rad = command_rad;
    joint->feedback_rad = feedback_rad;
    joint->feedback_valid = feedback_valid;

    WeaponRotateJoint_ClearFeedback(joint);

    /* 硬件通信配置由 CubeMX/FDCAN3 和 BSP 负责，这里只注册 RobStride 实例。 */
    joint->motor.config.hfdcan = &hfdcan3;
    joint->motor.config.motor_id = motor_id;
    joint->motor.config.master_id = 0xFDU;
    joint->motor.config.motor_type = motor_type;
    joint->motor.config.offline_timeout_ms = WEAPON_ROTATE_APP_OFFLINE_TIMEOUT_MS;

    if (rs_motor_init(&joint->motor) != RS_MOTOR_OK) {
        WeaponRotateJoint_FailAndDisable(joint);
        return;
    }

    joint->target_position_rad = 0.0f;
    joint->target_received = 0U;
    joint->last_command_sequence = 0U;
    joint->last_ctrl_ms = 0U;
}

/* ---- 公共接口 ---- */

void WeaponRotateApp_Init(void)
{
    if (s_initialized != 0U) {
        return;
    }

    /* 参数合法性检查（只做一次）。 */
    if (WeaponRotateIsFinite(WEAPON_ROTATE_APP_CSP_CURRENT_LIMIT_A) == 0U ||
        WeaponRotateIsFinite(WEAPON_ROTATE_APP_MAX_SPEED_RAD_S) == 0U ||
        WEAPON_ROTATE_APP_CSP_CURRENT_LIMIT_A <= 0.0f ||
        WEAPON_ROTATE_APP_MAX_SPEED_RAD_S <= 0.0f ||
        WEAPON_ROTATE_APP_CTRL_PERIOD_MS == 0U ||
        WEAPON_ROTATE_APP_CTRL_PERIOD_MS >=
            WEAPON_ROTATE_APP_OFFLINE_TIMEOUT_MS) {
        return;
    }

    /* 端头旋转：RS05, id=5, comm 字段 weapon_rotate。 */
    WeaponRotateJoint_Init(
        &s_joint,
        5U,
        RS_MOTOR_TYPE_5,
        &g_comm_app_command.weapon_rotate_rad,
        &g_comm_app_feedback.weapon_rotate_rad,
        &g_comm_app_feedback.weapon_rotate_valid);

    s_initialized = 1U;
}

void WeaponRotateApp_RunPeriodic(void)
{
    uint32_t now_ms;

    if (s_initialized == 0U) {
        g_comm_app_feedback.weapon_rotate_rad = 0.0f;
        g_comm_app_feedback.weapon_rotate_valid = 0U;
        return;
    }

    now_ms = HAL_GetTick();
    WeaponRotateJoint_RunPeriodic(&s_joint, now_ms);
}
