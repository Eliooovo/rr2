/**
 * @file    kfs_rotate_app.c
 * @brief   KFS RS03 根部旋转 CSP 位置控制（限速 ~45°/s）。
 *
 * 上位机通过 USB 协议下发目标角度（单位 rad），App 在收到首个有效命令
 * 后使能电机，并以不超过 KFS_ROTATE_APP_MAX_SPEED_RAD_S 的速度平滑到位。
 * 反馈为 RS03 单圈角度（依赖机械零位），不使用多圈累计。
 */

#include "kfs_rotate_app.h"

#include <float.h>

#include "comm_app.h"
#include "fdcan.h"
#include "main.h"
#include "rs_motor.h"

/* ---- 私有状态 ---- */
static rs_motor_t s_motor;
static uint8_t s_initialized;

static float s_target_position_rad;
static uint8_t s_target_received; /* 收到首个有效命令前置 0，避免上电自运动 */
static uint8_t s_target_applied;  /* 当前目标是否已下发到电机 */

static uint32_t s_last_command_sequence;
static uint32_t s_last_ctrl_ms;

/* ---- 辅助函数 ---- */

static uint8_t KfsRotateApp_IsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static void KfsRotateApp_ClearFeedback(void)
{
    g_comm_app_feedback.kfs_root_rotate_rad = 0.0f;
    g_comm_app_feedback.kfs_root_rotate_valid = 0U;
}

static void KfsRotateApp_FailAndDisable(void)
{
    KfsRotateApp_ClearFeedback();
    s_target_applied = 0U;
    (void)rs_motor_disable(&s_motor);
}

static uint8_t KfsRotateApp_FeedbackIsRecent(uint32_t now_ms)
{
    return (s_motor.state.online != 0U &&
            (uint32_t)(now_ms - s_motor.state.last_update_ms) <
                KFS_ROTATE_APP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

/* 检查上位机新命令：sequence 递增说明收到新帧，更新目标。 */
static void KfsRotateApp_UpdateTarget(void)
{
    uint32_t sequence = g_comm_app_command.sequence;
    float target_rad;

    if (g_comm_app_command.valid == 0U ||
        sequence == s_last_command_sequence) {
        return;
    }
    s_last_command_sequence = sequence;

    target_rad = g_comm_app_command.kfs_root_rotate_rad;
    if (KfsRotateApp_IsFinite(target_rad) == 0U) {
        /* 非有限值不覆盖旧目标，保持上一次有效角度命令。 */
        return;
    }

    if (s_target_received == 0U ||
        target_rad != s_target_position_rad) {
        s_target_position_rad = target_rad;
        s_target_applied = 0U;
    }
    s_target_received = 1U;
}

static void KfsRotateApp_UpdateFeedback(uint32_t now_ms)
{
    rs_motor_feedback_t fb;
    float angle_rad;

    if (KfsRotateApp_FeedbackIsRecent(now_ms) == 0U ||
        rs_motor_get_feedback(&s_motor, &fb) != RS_MOTOR_OK) {
        KfsRotateApp_ClearFeedback();
        return;
    }

    angle_rad = fb.angle_rad;
    if (KfsRotateApp_IsFinite(angle_rad) == 0U) {
        KfsRotateApp_ClearFeedback();
        return;
    }

    g_comm_app_feedback.kfs_root_rotate_rad = angle_rad;
    g_comm_app_feedback.kfs_root_rotate_valid = 1U;
}

/* ---- 公共接口 ---- */

void KfsRotateApp_Init(void)
{
    rs_motor_status_t status;

    if (s_initialized != 0U) {
        return;
    }

    KfsRotateApp_ClearFeedback();

    if (KfsRotateApp_IsFinite(KFS_ROTATE_APP_MAX_SPEED_RAD_S) == 0U ||
        KFS_ROTATE_APP_MAX_SPEED_RAD_S <= 0.0f ||
        KFS_ROTATE_APP_CTRL_PERIOD_MS == 0U ||
        KFS_ROTATE_APP_CTRL_PERIOD_MS >=
            KFS_ROTATE_APP_OFFLINE_TIMEOUT_MS) {
        KfsRotateApp_FailAndDisable();
        return;
    }

    /* 硬件通信配置由 CubeMX/FDCAN3 和 BSP 负责，这里只注册 RobStride 实例。 */
    s_motor.config.hfdcan = &hfdcan3;
    s_motor.config.motor_id = 2U;
    s_motor.config.master_id = 0xFDU;
    s_motor.config.motor_type = RS_MOTOR_TYPE_3;
    s_motor.config.offline_timeout_ms = KFS_ROTATE_APP_OFFLINE_TIMEOUT_MS;

    status = rs_motor_init(&s_motor);
    if (status != RS_MOTOR_OK) {
        KfsRotateApp_FailAndDisable();
        return;
    }

    s_target_position_rad = 0.0f;
    s_target_received = 0U;
    s_target_applied = 0U;
    s_last_command_sequence = 0U;
    s_last_ctrl_ms = 0U;
    s_initialized = 1U;
}

void KfsRotateApp_RunPeriodic(void)
{
    uint32_t now_ms;

    if (s_initialized == 0U) {
        KfsRotateApp_ClearFeedback();
        return;
    }

    now_ms = HAL_GetTick();

    /* 1. 更新反馈到上位机邮箱。 */
    KfsRotateApp_UpdateFeedback(now_ms);

    /* 2. 检测离线：feedback_count 不涨 → 重置状态等下次 CSP 自动重新使能。 */
    {
        static uint32_t last_fb_count = 0U;

        if (s_motor.state.feedback_count == last_fb_count) {
            s_motor.internal.mode_applied = 0U;
            s_motor.state.enabled = 0U;
        }
        last_fb_count = s_motor.state.feedback_count;
    }

    /* 3. 按周期检查命令、下发 CSP。 */
    if ((uint32_t)(now_ms - s_last_ctrl_ms) <
        KFS_ROTATE_APP_CTRL_PERIOD_MS) {
        return;
    }
    s_last_ctrl_ms = now_ms;

    KfsRotateApp_UpdateTarget();

    /*
     * 安全：未收到任何有效上位机命令前不使能电机，
     * 避免上电后在无目标的情况下自运动。
     */
    if (s_target_received == 0U) {
        return;
    }

    /*
     * 目标已下发且电机正常运行 → 跳过，减少 CAN 总线负载。
     * 例外：刚上电 (mode_applied=0) 或离线恢复时必须重新下发。
     */
    if (s_target_applied != 0U &&
        s_motor.state.enabled != 0U &&
        s_motor.internal.mode_applied != 0U) {
        return;
    }

    /*
     * CSP（Cyclic Synchronous Position）位置控制：
     *   电机内部以不超过 speed_limit 的速度平滑移动到目标位置，
     *   到位后自动保持。即使目标不变也下发，确保模式/使能状态正确。
     */
    if (rs_motor_csp_position_control(
            &s_motor,
            KFS_ROTATE_APP_MAX_SPEED_RAD_S,
            s_target_position_rad) != RS_MOTOR_OK) {
        KfsRotateApp_FailAndDisable();
        return;
    }
    s_target_applied = 1U;
}
