/**
 * @file    kfs_grip_app.c
 * @brief   KFS 夹爪开合命令换算、RobStride 限流 CSP 控制与米制反馈。
 *
 * 管理单个 RS05 夹爪电机（RS_MOTOR_TYPE_5, id=4），挂载在 FDCAN3。
 * 上位机通过 USB 协议下发目标位置（单位 m），App 内部转换为电机单圈
 * 角度 rad 后通过限流 CSP 位置控制执行，到位后持续保持。
 * 反馈为单圈角度 rad → 米制 m（依赖机械零位），不使用多圈累计。
 */

#include "kfs_grip_app.h"

#include <float.h>

#include "comm_app.h"
#include "fdcan.h"
#include "main.h"
#include "rs_motor.h"

/* ---- 私有变量 ---- */

static rs_motor_t s_motor;
static float s_target_position_rad;
static uint32_t s_last_command_sequence;
static uint32_t s_last_ctrl_ms;
static uint8_t s_initialized;
static uint8_t s_target_received;

/* ---- 辅助函数 ---- */

static uint8_t KfsGripApp_IsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static uint8_t KfsGripApp_IsFiniteDouble(double value)
{
    return (value >= -DBL_MAX && value <= DBL_MAX) ? 1U : 0U;
}

static uint8_t KfsGripApp_DirectionIsValid(double direction)
{
    return (direction == 1.0 || direction == -1.0) ? 1U : 0U;
}

static void KfsGripApp_ClearFeedback(void)
{
    g_comm_app_feedback.kfs_grip_position_m = 0.0f;
    g_comm_app_feedback.kfs_grip_valid = 0U;
}

static void KfsGripApp_FailAndDisable(void)
{
    KfsGripApp_ClearFeedback();
    (void)rs_motor_disable(&s_motor);
}

/*
 * 单位换算：上位机 m <-> 电机 rad。
 *   命令：meters → motor_rad = (meters * DIRECTION) / METERS_PER_MOTOR_RAD
 *   反馈：motor_rad → meters = rad * METERS_PER_MOTOR_RAD * DIRECTION
 */
static float KfsGripApp_MetersToMotorRad(float position_m)
{
    return (position_m * (float)KFS_GRIP_APP_DIRECTION) /
           (float)KFS_GRIP_APP_METERS_PER_MOTOR_RAD;
}

static float KfsGripApp_MotorRadToMeters(float position_rad)
{
    return position_rad *
           (float)KFS_GRIP_APP_METERS_PER_MOTOR_RAD *
           (float)KFS_GRIP_APP_DIRECTION;
}

static uint8_t KfsGripApp_FeedbackIsRecent(uint32_t now_ms)
{
    return (s_motor.state.online != 0U &&
            (uint32_t)(now_ms - s_motor.state.last_update_ms) <
                KFS_GRIP_APP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

/* 检查上位机新命令：sequence 递增说明收到新帧，更新目标。 */
static void KfsGripApp_UpdateTargetFromCommand(void)
{
    uint32_t sequence = g_comm_app_command.sequence;
    float target_m;

    if (g_comm_app_command.valid == 0U ||
        sequence == s_last_command_sequence) {
        return;
    }
    s_last_command_sequence = sequence;

    target_m = g_comm_app_command.kfs_grip_position_m;
    if (KfsGripApp_IsFinite(target_m) == 0U) {
        /* 非有限值不覆盖旧目标，保持上一次有效命令。 */
        return;
    }

    {
        float target_rad = KfsGripApp_MetersToMotorRad(target_m);

        if (KfsGripApp_IsFinite(target_rad) == 0U) {
            return;
        }

        if (s_target_received == 0U ||
            target_rad != s_target_position_rad) {
            s_target_position_rad = target_rad;
        }
        s_target_received = 1U;
    }
}

static void KfsGripApp_UpdateFeedback(uint32_t now_ms)
{
    rs_motor_feedback_t fb;
    float position_m;

    if (KfsGripApp_FeedbackIsRecent(now_ms) == 0U ||
        rs_motor_get_feedback(&s_motor, &fb) != RS_MOTOR_OK) {
        KfsGripApp_ClearFeedback();
        return;
    }

    position_m = KfsGripApp_MotorRadToMeters(fb.angle_rad);
    if (KfsGripApp_IsFinite(position_m) == 0U) {
        KfsGripApp_ClearFeedback();
        return;
    }

    g_comm_app_feedback.kfs_grip_position_m = position_m;
    g_comm_app_feedback.kfs_grip_valid = 1U;
}

/* ---- 公共接口 ---- */

void KfsGripApp_Init(void)
{
    rs_motor_status_t status;

    if (s_initialized != 0U) {
        return;
    }

    KfsGripApp_ClearFeedback();

    /* 参数合法性检查（只做一次）。 */
    if (KfsGripApp_IsFiniteDouble(KFS_GRIP_APP_METERS_PER_MOTOR_RAD) == 0U ||
        KfsGripApp_IsFiniteDouble(KFS_GRIP_APP_DIRECTION) == 0U ||
        KfsGripApp_IsFinite(KFS_GRIP_APP_CSP_CURRENT_LIMIT_A) == 0U ||
        KfsGripApp_IsFinite(KFS_GRIP_APP_CSP_SPEED_LIMIT_RAD_S) == 0U ||
        KfsGripApp_DirectionIsValid(KFS_GRIP_APP_DIRECTION) == 0U ||
        KFS_GRIP_APP_METERS_PER_MOTOR_RAD <= 0.0 ||
        KFS_GRIP_APP_CSP_CURRENT_LIMIT_A <= 0.0f ||
        KFS_GRIP_APP_CSP_SPEED_LIMIT_RAD_S <= 0.0f ||
        KFS_GRIP_APP_CTRL_PERIOD_MS == 0U ||
        KFS_GRIP_APP_CTRL_PERIOD_MS >=
            KFS_GRIP_APP_OFFLINE_TIMEOUT_MS) {
        KfsGripApp_FailAndDisable();
        return;
    }

    /* 硬件通信配置由 CubeMX/FDCAN3 和 BSP 负责，这里只注册 RobStride 实例。 */
    s_motor.config.hfdcan = &hfdcan3;
    s_motor.config.motor_id = 4U;
    s_motor.config.master_id = 0xFDU;
    s_motor.config.motor_type = RS_MOTOR_TYPE_5;
    s_motor.config.offline_timeout_ms = KFS_GRIP_APP_OFFLINE_TIMEOUT_MS;

    status = rs_motor_init(&s_motor);
    if (status != RS_MOTOR_OK) {
        KfsGripApp_FailAndDisable();
        return;
    }

    s_target_position_rad = 0.0f;
    s_last_command_sequence = 0U;
    s_last_ctrl_ms = 0U;
    s_target_received = 0U;
    s_initialized = 1U;
}

void KfsGripApp_RunPeriodic(void)
{
    uint32_t now_ms;

    if (s_initialized == 0U) {
        KfsGripApp_ClearFeedback();
        return;
    }

    now_ms = HAL_GetTick();

    /* 离线判断由驱动按反馈时间戳处理，不能按主循环次数判断。 */
    if (rs_motor_update(&s_motor, now_ms) != RS_MOTOR_OK) {
        KfsGripApp_FailAndDisable();
        return;
    }

    /* 1. 更新反馈到上位机邮箱。 */
    KfsGripApp_UpdateFeedback(now_ms);

    /* 2. 按周期检查命令、下发 CSP 位置目标。 */
    if ((uint32_t)(now_ms - s_last_ctrl_ms) <
        KFS_GRIP_APP_CTRL_PERIOD_MS) {
        return;
    }
    s_last_ctrl_ms = now_ms;

    KfsGripApp_UpdateTargetFromCommand();

    /*
     * 安全：未收到任何有效上位机命令前不使能电机，
     * 避免上电后在无目标的情况下自运动。
     */
    if (s_target_received == 0U) {
        return;
    }

    /*
     * CSP 限流位置控制：
     *   电机以受限速度移动到目标角度；被物体阻挡后，位置环输出受
     *   current_limit 限制，从而以可控夹持力持续保持。
     * 电流/速度只在首次配置、参数变化或离线恢复时写入；正常周期只
     * 重发位置目标，作为 CAN keepalive 并刷新电机 Type 2 反馈。
     */
    if (rs_motor_csp_position_control_limited(
            &s_motor,
            KFS_GRIP_APP_CSP_CURRENT_LIMIT_A,
            KFS_GRIP_APP_CSP_SPEED_LIMIT_RAD_S,
            s_target_position_rad) != RS_MOTOR_OK) {
        KfsGripApp_FailAndDisable();
        return;
    }
}
