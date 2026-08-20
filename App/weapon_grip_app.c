/**
 * @file    weapon_grip_app.c
 * @brief   端头开合命令换算、STS 舵机位置控制与米制反馈。
 *
 * 管理单个 STS3215 舵机（ID=6），挂载在 UART7。
 * 上位机通过 USB 协议下发目标位置（单位 m），App 内部直接换算为
 * 舵机 raw 位置值（0~4095，零点 CENTER_RAW=2048），
 * 通过 WritePosEx 执行，到位后自动保持。
 * 反馈为 raw → 米制 m。
 */

#include "weapon_grip_app.h"

#include <float.h>
#include <math.h>

#include "comm_app.h"
#include "main.h"
#include "sts_servo.h"
#include "usart.h"

/* ---- 私有变量 ---- */

static sts_servo_t s_servo;
static int16_t s_target_position_raw;
static uint32_t s_last_command_sequence;
static uint32_t s_last_ctrl_ms;
static uint32_t s_last_feedback_ms;
static uint32_t s_position_command_error_count;
static uint32_t s_position_recovery_count;
static uint32_t s_target_applied_ms;
static sts_servo_status_t s_last_position_command_status;
static uint8_t s_initialized;
static uint8_t s_target_received;
static uint8_t s_target_applied;
static uint8_t s_recovery_attempted;

/* ---- 辅助函数 ---- */

static uint8_t WeaponGripApp_IsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static void WeaponGripApp_ClearFeedback(void)
{
    g_comm_app_feedback.weapon_grip_position_m = 0.0f;
    g_comm_app_feedback.weapon_grip_valid = 0U;
}

static void WeaponGripApp_FailAndDisable(void)
{
    WeaponGripApp_ClearFeedback();
    s_target_applied = 0U;
    (void)sts_servo_disable(&s_servo);
}

/*
 * 单位换算：上位机 m <-> 舵机 raw。
 *
 * 往外（正米制）→ raw 减小（从 2048 往 1024 方向，即 -90°）。
 *   命令：raw = CENTER_RAW - position_m * RAW_PER_METER
 *   反馈：position_m = (CENTER_RAW - raw) / RAW_PER_METER
 */
static int16_t WeaponGripApp_MetersToServoRaw(float position_m)
{
    float raw_f = (float)WEAPON_GRIP_APP_CENTER_RAW -
                  position_m * WEAPON_GRIP_APP_RAW_PER_METER;

    if (raw_f < 0.0f) {
        raw_f = 0.0f;
    } else if (raw_f > 4095.0f) {
        raw_f = 4095.0f;
    }
    return (int16_t)raw_f;
}

static float WeaponGripApp_ServoRawToMeters(int16_t raw)
{
    return ((float)WEAPON_GRIP_APP_CENTER_RAW - (float)raw) /
           WEAPON_GRIP_APP_RAW_PER_METER;
}

static int16_t WeaponGripApp_FeedbackRaw(void)
{
    return (int16_t)(s_servo.feedback.pos_rad * 4096.0f /
                     (2.0f * M_PI));
}

/* 检查上位机新命令：sequence 递增说明收到新帧，更新目标。 */
static void WeaponGripApp_UpdateTargetFromCommand(void)
{
    uint32_t sequence = g_comm_app_command.sequence;
    float target_m;

    if (g_comm_app_command.valid == 0U ||
        sequence == s_last_command_sequence) {
        return;
    }
    s_last_command_sequence = sequence;

    target_m = g_comm_app_command.weapon_grip_position_m;
    if (WeaponGripApp_IsFinite(target_m) == 0U) {
        /* 非有限值不覆盖旧目标，保持上一次有效命令。 */
        return;
    }

    {
        int16_t target_raw = WeaponGripApp_MetersToServoRaw(target_m);

        if (s_target_received == 0U ||
            target_raw != s_target_position_raw) {
            s_target_position_raw = target_raw;
            s_target_applied = 0U;
            s_recovery_attempted = 0U;
        }
        s_target_received = 1U;
    }
}

static void WeaponGripApp_UpdateFeedback(void)
{
    sts_servo_feedback_t fb;
    float position_m;

    /* 离线时仍持续读取，使通信恢复后能自动重新上线。 */
    if (sts_servo_get_feedback(&s_servo, &fb) != STS_SERVO_OK) {
        /* 短暂丢包保留上一帧，超时由周期函数统一清除。 */
        return;
    }

    position_m = WeaponGripApp_ServoRawToMeters(
        (int16_t)(fb.pos_rad * 4096.0f / (2.0f * M_PI)));
    if (WeaponGripApp_IsFinite(position_m) == 0U) {
        WeaponGripApp_ClearFeedback();
        return;
    }

    g_comm_app_feedback.weapon_grip_position_m = position_m;
    g_comm_app_feedback.weapon_grip_valid = 1U;
}

/* ---- 公共接口 ---- */

void WeaponGripApp_Init(void)
{
    sts_servo_status_t status;

    if (s_initialized != 0U) {
        return;
    }

    WeaponGripApp_ClearFeedback();

    /* 参数合法性检查（只做一次）。 */
    if (WeaponGripApp_IsFinite(WEAPON_GRIP_APP_RAW_PER_METER) == 0U ||
        WEAPON_GRIP_APP_RAW_PER_METER <= 0.0f ||
        WEAPON_GRIP_APP_CENTER_RAW > 4095 ||
        WEAPON_GRIP_APP_CONTROL_PERIOD_MS == 0U ||
        WEAPON_GRIP_APP_CONTROL_PERIOD_MS >=
            WEAPON_GRIP_APP_OFFLINE_TIMEOUT_MS) {
        WeaponGripApp_FailAndDisable();
        return;
    }

    /* 等待舵机上电稳定。 */
    HAL_Delay(500);

    /* 硬件通信配置由 CubeMX/UART7 负责，这里只注册 STS 舵机实例。 */
    s_servo.config.huart = &huart7;
    s_servo.config.id = 6U;
    s_servo.config.offline_timeout_ms = WEAPON_GRIP_APP_OFFLINE_TIMEOUT_MS;

    status = sts_servo_init(&s_servo);
    if (status != STS_SERVO_OK) {
        WeaponGripApp_FailAndDisable();
        return;
    }

    status = sts_servo_enable(&s_servo);
    if (status != STS_SERVO_OK) {
        WeaponGripApp_FailAndDisable();
        return;
    }

    s_target_position_raw = (int16_t)WEAPON_GRIP_APP_CENTER_RAW;
    s_last_command_sequence = 0U;
    s_last_ctrl_ms = 0U;
    s_position_command_error_count = 0U;
    s_position_recovery_count = 0U;
    s_target_applied_ms = 0U;
    s_last_position_command_status = STS_SERVO_OK;
    s_target_received = 0U;
    s_target_applied = 0U;
    s_recovery_attempted = 0U;
    s_initialized = 1U;
}

void WeaponGripApp_RunPeriodic(void)
{
    uint32_t now_ms;

    if (s_initialized == 0U) {
        WeaponGripApp_ClearFeedback();
        return;
    }

    now_ms = HAL_GetTick();

    /* 1. 更新反馈到上位机邮箱（限速以减少 UART 阻塞）。 */
    if ((uint32_t)(now_ms - s_last_feedback_ms) >=
        WEAPON_GRIP_APP_FEEDBACK_PERIOD_MS) {
        WeaponGripApp_UpdateFeedback();
        s_last_feedback_ms = now_ms;
    }

    /* 2. 更新舵机在线状态。 */
    /* UART 读取可能跨 tick，在读取完成后重新取时间判定超时。 */
    sts_servo_update(&s_servo, HAL_GetTick());
    if (s_servo.state.online == 0U) {
        WeaponGripApp_ClearFeedback();
    }

    /* 3. 按周期检查命令、下发位置。 */
    if ((uint32_t)(now_ms - s_last_ctrl_ms) <
        WEAPON_GRIP_APP_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_ctrl_ms = now_ms;

    WeaponGripApp_UpdateTargetFromCommand();

    /*
     * 安全：未收到任何有效上位机命令前不下发位置目标，
     * 避免上电后在无目标的情况下自运动。
     */
    if (s_target_received == 0U) {
        return;
    }

    /*
     * 目标已下发且舵机正常运行 → 跳过，减少串口总线负载。
     * 例外：目标变化 (target_applied=0) 时必须重新下发。
     */
    if (s_target_applied != 0U && s_servo.state.online != 0U) {
        int32_t error_raw = (int32_t)s_target_position_raw -
                            (int32_t)WeaponGripApp_FeedbackRaw();

        if (error_raw < 0) {
            error_raw = -error_raw;
        }

        if (s_recovery_attempted == 0U &&
            s_servo.feedback.moving == 0U &&
            (uint32_t)error_raw > WEAPON_GRIP_APP_POSITION_TOLERANCE_RAW &&
            (uint32_t)(now_ms - s_target_applied_ms) >=
                WEAPON_GRIP_APP_RECOVERY_DELAY_MS) {
            /* 只重发一次以清除舵机内部堵转保护，避免持续顶机构。 */
            s_recovery_attempted = 1U;
            s_position_recovery_count++;
            s_target_applied = 0U;
        } else {
            return;
        }
    }

    /*
     * 舵机位置控制：直接写 raw 目标值。
     */
    s_last_position_command_status = sts_servo_set_position_raw(
        &s_servo,
        s_target_position_raw,
        WEAPON_GRIP_APP_MOVE_SPEED,
        WEAPON_GRIP_APP_MOVE_ACC);
    if (s_last_position_command_status != STS_SERVO_OK) {
        /*
         * 位置写入的应答丢失是可恢复的通信故障。保留扭矩使能和
         * 当前安全位置，下一个控制周期重试，避免因一次 ORE/超时永久失能。
         */
        s_position_command_error_count++;
        s_target_applied = 0U;
        return;
    }
    s_target_applied = 1U;
    s_target_applied_ms = HAL_GetTick();
}
