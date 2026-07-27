/**
 * @file    kfs_lift_app.c
 * @brief   KFS 抬升命令换算、RobStride 控制与米制反馈。
 */

#include "kfs_lift_app.h"

#include <float.h>

#include "comm_app.h"
#include "fdcan.h"
#include "main.h"
#include "rs_motor.h"

static rs_motor_t s_motor;
static double s_target_position_rad;
static uint32_t s_last_command_sequence;
static uint32_t s_last_control_ms;
static uint8_t s_initialized;
static uint8_t s_control_timer_started;
/* 收到首个有限命令前不下发电机使能和位置目标，避免上电自运动。 */
static uint8_t s_target_received;
/* 新目标只配置一次，之后由 rs_motor_update() 周期运行多圈位置外环。 */
static uint8_t s_target_applied;

static uint8_t KfsLiftApp_IsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static uint8_t KfsLiftApp_IsFiniteDouble(double value)
{
    return (value >= -DBL_MAX && value <= DBL_MAX) ? 1U : 0U;
}

static uint8_t KfsLiftApp_DirectionIsValid(double direction)
{
    return (direction == 1.0 || direction == -1.0) ? 1U : 0U;
}

static void KfsLiftApp_ClearFeedback(void)
{
    g_comm_app_feedback.kfs_lift_position_m = 0.0f;
    g_comm_app_feedback.kfs_lift_valid = 0U;
}

static void KfsLiftApp_FailAndDisable(void)
{
    KfsLiftApp_ClearFeedback();
    s_target_applied = 0U;
    /* 失败时尽量让电机退出控制；未初始化时驱动会返回错误，这里忽略。 */
    (void)rs_motor_disable(&s_motor);
}

static double KfsLiftApp_MetersToMotorRad(float position_m)
{
    /* 命令：上位机米制位移 -> 电机连续多圈位置 rad。 */
    return ((double)position_m * KFS_LIFT_APP_DIRECTION) /
           KFS_LIFT_APP_METERS_PER_MOTOR_RAD;
}

static float KfsLiftApp_MotorRadToMeters(double position_rad)
{
    /* 反馈：电机连续多圈位置 rad -> 上位机米制位移。 */
    return (float)(position_rad *
                   KFS_LIFT_APP_METERS_PER_MOTOR_RAD *
                   KFS_LIFT_APP_DIRECTION);
}

static uint8_t KfsLiftApp_FeedbackIsRecent(uint32_t now_ms)
{
    return (s_motor.state.online != 0U &&
            (uint32_t)(now_ms - s_motor.state.last_update_ms) <
                KFS_LIFT_APP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static void KfsLiftApp_UpdateTargetFromCommand(void)
{
    uint32_t sequence = g_comm_app_command.sequence;
    float target_m;
    double target_rad;

    /* command.sequence 每收到一帧合法 USB 协议帧递增，用它避免重复处理。 */
    if (g_comm_app_command.valid == 0U ||
        sequence == s_last_command_sequence) {
        return;
    }
    s_last_command_sequence = sequence;

    target_m = g_comm_app_command.kfs_lift_position_m;
    if (KfsLiftApp_IsFinite(target_m) == 0U) {
        /* 非有限值不覆盖旧目标，保持上一次有效抬升命令。 */
        return;
    }

    target_rad = KfsLiftApp_MetersToMotorRad(target_m);
    if (KfsLiftApp_IsFiniteDouble(target_rad) == 0U) {
        return;
    }
    if (s_target_received == 0U ||
        target_rad != s_target_position_rad) {
        s_target_position_rad = target_rad;
        s_target_applied = 0U;
    }
    s_target_received = 1U;
}

static void KfsLiftApp_UpdateFeedback(uint32_t now_ms)
{
    rs_motor_feedback_t feedback;
    float position_m;

    /*
     * 只有电机在线、反馈未超时并且已建立 RobStride 多圈软件零点时，
     * 才向 comm_app 发布有效米制反馈。
     */
    if (KfsLiftApp_FeedbackIsRecent(now_ms) == 0U ||
        rs_motor_get_feedback(&s_motor, &feedback) != RS_MOTOR_OK ||
        feedback.multi_turn.valid == 0U) {
        KfsLiftApp_ClearFeedback();
        return;
    }

    position_m =
        KfsLiftApp_MotorRadToMeters(feedback.multi_turn.angle_rad);
    if (KfsLiftApp_IsFinite(position_m) == 0U) {
        KfsLiftApp_ClearFeedback();
        return;
    }

    g_comm_app_feedback.kfs_lift_position_m = position_m;
    g_comm_app_feedback.kfs_lift_valid = 1U;
}

void KfsLiftApp_Init(void)
{
    rs_motor_status_t status;

    if (s_initialized != 0U) {
        return;
    }

    KfsLiftApp_ClearFeedback();
    /* 本任务暂不控制这些协议字段，初始化为 0 保持反馈帧内容确定。 */
    g_comm_app_feedback.kfs_root_rotate_rad = 0.0f;
    g_comm_app_feedback.kfs_tip_rotate_rad = 0.0f;
    g_comm_app_feedback.kfs_grip_position_m = 0.0f;
    g_comm_app_feedback.weapon_rotate_rad = 0.0f;
    g_comm_app_feedback.weapon_grip_position_m = 0.0f;

    if (KfsLiftApp_IsFiniteDouble(KFS_LIFT_APP_METERS_PER_MOTOR_RAD) == 0U ||
        KfsLiftApp_IsFiniteDouble(KFS_LIFT_APP_DIRECTION) == 0U ||
        KfsLiftApp_IsFinite(KFS_LIFT_APP_MAX_SPEED_RAD_S) == 0U ||
        KfsLiftApp_IsFinite(KFS_LIFT_APP_ACCELERATION_RAD_S2) == 0U ||
        KfsLiftApp_IsFinite(KFS_LIFT_APP_CURRENT_LIMIT_A) == 0U ||
        KfsLiftApp_IsFinite(KFS_LIFT_APP_POSITION_KP_S_1) == 0U ||
        KfsLiftApp_IsFinite(KFS_LIFT_APP_POSITION_TOLERANCE_RAD) == 0U ||
        KfsLiftApp_DirectionIsValid(KFS_LIFT_APP_DIRECTION) == 0U ||
        KFS_LIFT_APP_METERS_PER_MOTOR_RAD <= 0.0 ||
        KFS_LIFT_APP_MAX_SPEED_RAD_S <= 0.0f ||
        KFS_LIFT_APP_ACCELERATION_RAD_S2 <= 0.0f ||
        KFS_LIFT_APP_CURRENT_LIMIT_A <= 0.0f ||
        KFS_LIFT_APP_POSITION_KP_S_1 <= 0.0f ||
        KFS_LIFT_APP_POSITION_TOLERANCE_RAD < 0.0f ||
        KFS_LIFT_APP_CONTROL_PERIOD_MS == 0U ||
        KFS_LIFT_APP_CONTROL_PERIOD_MS >=
            KFS_LIFT_APP_OFFLINE_TIMEOUT_MS) {
        KfsLiftApp_FailAndDisable();
        return;
    }

    /* 硬件通信配置由 CubeMX/FDCAN3 和 BSP 负责，这里只注册 RobStride 实例。 */
    s_motor.config.hfdcan = &hfdcan3;
    s_motor.config.motor_id = 1U;
    s_motor.config.master_id = 0xFDU;
    s_motor.config.motor_type = RS_MOTOR_TYPE_0;
    s_motor.config.offline_timeout_ms = KFS_LIFT_APP_OFFLINE_TIMEOUT_MS;
    s_motor.config.multi_turn.current_limit_a =
        KFS_LIFT_APP_CURRENT_LIMIT_A;
    s_motor.config.multi_turn.position_kp_s_1 =
        KFS_LIFT_APP_POSITION_KP_S_1;
    s_motor.config.multi_turn.position_tolerance_rad =
        KFS_LIFT_APP_POSITION_TOLERANCE_RAD;
    s_motor.config.multi_turn.control_period_ms =
        KFS_LIFT_APP_CONTROL_PERIOD_MS;

    status = rs_motor_init(&s_motor);
    if (status != RS_MOTOR_OK) {
        KfsLiftApp_FailAndDisable();
        return;
    }

    s_target_position_rad = 0.0;
    s_last_command_sequence = 0U;
    s_last_control_ms = 0U;
    s_control_timer_started = 0U;
    s_target_received = 0U;
    s_target_applied = 0U;
    s_initialized = 1U;
}

void KfsLiftApp_RunPeriodic(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;

    if (s_initialized == 0U) {
        KfsLiftApp_ClearFeedback();
        return;
    }

    now_ms = HAL_GetTick();
    if (s_control_timer_started == 0U) {
        s_last_control_ms = now_ms;
        s_control_timer_started = 1U;
        KfsLiftApp_UpdateFeedback(now_ms);
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - s_last_control_ms);
    if (elapsed_ms < KFS_LIFT_APP_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;

    KfsLiftApp_UpdateTargetFromCommand();
    /*
     * 第一次收到有效目标或目标变化时，配置 RobStride 连续多圈位置模式。
     * 没有命令时只更新在线状态和反馈，不主动给电机下发运动目标。
     */
    if (s_target_received != 0U &&
        s_target_applied == 0U &&
        rs_motor_multi_turn_position_control(
            &s_motor,
            KFS_LIFT_APP_MAX_SPEED_RAD_S,
            KFS_LIFT_APP_ACCELERATION_RAD_S2,
            s_target_position_rad) != RS_MOTOR_OK) {
        KfsLiftApp_FailAndDisable();
        return;
    }
    s_target_applied = s_target_received;

    if (rs_motor_update(&s_motor, now_ms) != RS_MOTOR_OK) {
        KfsLiftApp_FailAndDisable();
        return;
    }
    KfsLiftApp_UpdateFeedback(now_ms);
}
