/**
 * @file    kfs_lift.c
 * @brief   RS00 KFS 抬升模块实现
 */

#include "kfs_lift.h"

#include <float.h>
#include <string.h>

#include "stm32h7xx_hal.h"

#define KFS_LIFT_DEFAULT_FEEDBACK_PERIOD_RAD \
    (RS_MOTOR_FEEDBACK_POSITION_MAX_RAD - RS_MOTOR_FEEDBACK_POSITION_MIN_RAD)
#define KFS_LIFT_TWO_PI                     6.28318530718f

/* 判断配置或反馈浮点数是否为有限值。 */
static uint8_t KfsLift_IsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

/* 校验 KFS 机械换算、运动限制和通信配置。 */
static uint8_t KfsLift_ConfigIsValid(const kfs_lift_config_t *config)
{
    if (config == 0 || config->hfdcan == 0 || config->motor_id == 0U ||
        config->motor_id > 0x7FU || config->offline_timeout_ms == 0U ||
        (uint32_t)config->motor_type >= (uint32_t)RS_MOTOR_TYPE_COUNT) {
        return 0U;
    }
    if (!KfsLift_IsFinite(config->max_height_m) || config->max_height_m <= 0.0f ||
        !KfsLift_IsFinite(config->meters_per_revolution_m) ||
        config->meters_per_revolution_m <= 0.0f ||
        !KfsLift_IsFinite(config->speed_rad_s) || config->speed_rad_s <= 0.0f ||
        !KfsLift_IsFinite(config->acceleration_rad_s2) ||
        config->acceleration_rad_s2 <= 0.0f ||
        (config->direction != 1 && config->direction != -1) ||
        !KfsLift_IsFinite(config->position_command_limit_rad) ||
        config->position_command_limit_rad <= 0.0f ||
        config->max_height_m / config->meters_per_revolution_m * KFS_LIFT_TWO_PI >
            config->position_command_limit_rad) {
        return 0U;
    }
    return 1U;
}

/* 将绝对高度目标换算为带方向的电机轴角度目标，单位 rad。 */
static float KfsLift_HeightToAngleRad(const kfs_lift_t *lift, float height_m)
{
    return height_m / lift->config.meters_per_revolution_m * KFS_LIFT_TWO_PI *
           (float)lift->config.direction;
}

/* 消费新的 RS00 反馈，并按协议位置范围处理跨圈跳变。 */
static void KfsLift_UpdateFeedback(kfs_lift_t *lift)
{
    uint32_t feedback_count = lift->internal.motor.state.feedback_count;
    float feedback_angle_rad;
    float delta_rad;
    float feedback_period_rad = KFS_LIFT_DEFAULT_FEEDBACK_PERIOD_RAD;

    if (feedback_count == lift->internal.processed_feedback_count) {
        return;
    }

    feedback_angle_rad = lift->internal.motor.feedback.angle_rad;
    if (!KfsLift_IsFinite(feedback_angle_rad)) {
        lift->fault.invalid_feedback = 1U;
        lift->state.feedback_valid = 0U;
        lift->internal.processed_feedback_count = feedback_count;
        return;
    }

    if (lift->internal.feedback_unwrap_initialized == 0U) {
        lift->internal.accumulated_angle_rad = feedback_angle_rad;
        lift->internal.previous_feedback_angle_rad = feedback_angle_rad;
        lift->internal.feedback_unwrap_initialized = 1U;
    } else {
        delta_rad = feedback_angle_rad - lift->internal.previous_feedback_angle_rad;
        while (delta_rad > feedback_period_rad * 0.5f) {
            delta_rad -= feedback_period_rad;
        }
        while (delta_rad < -feedback_period_rad * 0.5f) {
            delta_rad += feedback_period_rad;
        }
        lift->internal.accumulated_angle_rad += delta_rad;
        lift->internal.previous_feedback_angle_rad = feedback_angle_rad;
    }

    lift->state.last_feedback_ms = lift->internal.motor.state.last_update_ms;
    lift->state.feedback_valid = 1U;
    lift->fault.motor_fault =
        (lift->internal.motor.feedback.fault.uncalibrated != 0U ||
         lift->internal.motor.feedback.fault.stall_overload != 0U ||
         lift->internal.motor.feedback.fault.magnetic_encoder != 0U ||
         lift->internal.motor.feedback.fault.over_temperature != 0U ||
         lift->internal.motor.feedback.fault.drive != 0U ||
         lift->internal.motor.feedback.fault.under_voltage != 0U) ? 1U : 0U;
    lift->state.actual_height_m =
        (lift->internal.accumulated_angle_rad - lift->internal.zero_angle_rad) /
        KFS_LIFT_TWO_PI * lift->config.meters_per_revolution_m *
        (float)lift->config.direction;
    lift->internal.processed_feedback_count = feedback_count;
}

/* 根据归零和反馈状态刷新对外的 ready 标志。 */
static void KfsLift_UpdateReadyState(kfs_lift_t *lift)
{
    lift->state.ready = (lift->state.zeroed != 0U &&
                         lift->state.feedback_valid != 0U &&
                         lift->internal.motor.state.online != 0U) ? 1U : 0U;
}

/** 初始化 RS00 驱动和 KFS 抬升状态，不会发送运动命令。 */
kfs_lift_status_t KfsLift_Init(kfs_lift_t *lift)
{
    rs_motor_status_t motor_status;

    if (lift == 0) {
        return KFS_LIFT_STATUS_INVALID_ARGUMENT;
    }
    if (lift->state.initialized != 0U) {
        return KFS_LIFT_STATUS_INVALID_ARGUMENT;
    }
    if (KfsLift_ConfigIsValid(&lift->config) == 0U) {
        return KFS_LIFT_STATUS_INVALID_CONFIG;
    }

    (void)memset(&lift->state, 0, sizeof(lift->state));
    (void)memset(&lift->fault, 0, sizeof(lift->fault));
    (void)memset(&lift->internal, 0, sizeof(lift->internal));

    lift->internal.motor.config.hfdcan = lift->config.hfdcan;
    lift->internal.motor.config.motor_id = lift->config.motor_id;
    lift->internal.motor.config.master_id = lift->config.master_id;
    lift->internal.motor.config.motor_type = lift->config.motor_type;
    lift->internal.motor.config.offline_timeout_ms = lift->config.offline_timeout_ms;
    lift->internal.motor.config.pp_position_limit_rad =
        lift->config.position_command_limit_rad;

    motor_status = rs_motor_init(&lift->internal.motor);
    if (motor_status != RS_MOTOR_STATUS_OK) {
        return (motor_status == RS_MOTOR_STATUS_INVALID_CONFIG) ?
               KFS_LIFT_STATUS_INVALID_CONFIG : KFS_LIFT_STATUS_MOTOR_ERROR;
    }

    lift->state.initialized = 1U;
    return KFS_LIFT_STATUS_OK;
}

/** 设置绝对目标高度，单位 m，合法范围为 0.0 m 至 max_height_m。 */
kfs_lift_status_t KfsLift_SetTargetHeightM(kfs_lift_t *lift, float height_m)
{
    float target_angle_rad;

    if (lift == 0 || !KfsLift_IsFinite(height_m)) {
        return KFS_LIFT_STATUS_INVALID_ARGUMENT;
    }
    if (lift->state.initialized == 0U) {
        return KFS_LIFT_STATUS_NOT_INITIALIZED;
    }
    if (lift->state.zeroed == 0U) {
        return KFS_LIFT_STATUS_NOT_ZEROED;
    }
    if (height_m < 0.0f || height_m > lift->config.max_height_m) {
        return KFS_LIFT_STATUS_INVALID_ARGUMENT;
    }
    if (lift->state.feedback_valid == 0U ||
        lift->internal.motor.state.online == 0U) {
        return KFS_LIFT_STATUS_FEEDBACK_OFFLINE;
    }

    target_angle_rad = lift->internal.zero_angle_rad +
                       KfsLift_HeightToAngleRad(lift, height_m);
    if (lift->internal.command_initialized == 0U ||
        target_angle_rad != lift->internal.last_command_angle_rad) {
        lift->internal.last_command_angle_rad = target_angle_rad;
        lift->internal.command_initialized = 1U;
        lift->state.target_pending = 1U;
    }
    lift->state.target_height_m = height_m;
    lift->fault.can_tx_error = 0U;
    return KFS_LIFT_STATUS_OK;
}

/** 将当前反馈电机位置记录为高度 0.0 m，并使模块进入可运动状态。 */
kfs_lift_status_t KfsLift_ConfirmZero(kfs_lift_t *lift)
{
    if (lift == 0) {
        return KFS_LIFT_STATUS_INVALID_ARGUMENT;
    }
    if (lift->state.initialized == 0U) {
        return KFS_LIFT_STATUS_NOT_INITIALIZED;
    }
    KfsLift_UpdateFeedback(lift);
    if (lift->state.feedback_valid == 0U ||
        lift->internal.motor.state.online == 0U) {
        return KFS_LIFT_STATUS_FEEDBACK_OFFLINE;
    }

    KfsLift_Stop(lift);
    lift->internal.zero_angle_rad = lift->internal.accumulated_angle_rad;
    lift->state.zeroed = 1U;
    lift->state.target_height_m = 0.0f;
    lift->state.target_pending = 0U;
    lift->state.actual_height_m = 0.0f;
    lift->fault.feedback_timeout = 0U;
    lift->fault.invalid_feedback = 0U;
    KfsLift_UpdateReadyState(lift);
    return KFS_LIFT_STATUS_OK;
}

/** 停止并失能 RS00 电机。 */
void KfsLift_Stop(kfs_lift_t *lift)
{
    if (lift == 0 || lift->state.initialized == 0U) {
        return;
    }
    lift->state.target_pending = 0U;
    lift->internal.command_initialized = 0U;
    if (lift->internal.motor.state.enabled != 0U) {
        if (rs_motor_disable(&lift->internal.motor) != RS_MOTOR_STATUS_OK) {
            lift->fault.can_tx_error = 1U;
        }
    }
}

/** 在主循环中周期调用，处理反馈累计、超时保护和待发送的 PP 目标。 */
void KfsLift_RunPeriodic(kfs_lift_t *lift)
{
    uint32_t now_ms;
    rs_motor_status_t motor_status;

    if (lift == 0 || lift->state.initialized == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    (void)rs_motor_update(&lift->internal.motor, now_ms);
    KfsLift_UpdateFeedback(lift);

    /* FDCAN 启动后先唤醒电机；未成功发送使能前不得进入位置控制。 */
    if (lift->state.wake_command_sent == 0U) {
        motor_status = rs_motor_enable(&lift->internal.motor);
        if (motor_status != RS_MOTOR_STATUS_OK) {
            lift->fault.can_tx_error = 1U;
            return;
        }
        lift->state.wake_command_sent = 1U;
    }

    if (lift->internal.motor.state.feedback_count != 0U &&
        lift->internal.motor.state.online == 0U) {
        lift->state.feedback_valid = 0U;
        lift->fault.feedback_timeout = 1U;
        KfsLift_UpdateReadyState(lift);
        KfsLift_Stop(lift);
        return;
    }

    KfsLift_UpdateReadyState(lift);
    if (lift->state.ready == 0U || lift->state.target_pending == 0U) {
        return;
    }

    motor_status = rs_motor_pp_position_control(&lift->internal.motor,
                                                lift->config.speed_rad_s,
                                                lift->config.acceleration_rad_s2,
                                                lift->internal.last_command_angle_rad);
    if (motor_status != RS_MOTOR_STATUS_OK) {
        lift->fault.can_tx_error = 1U;
        KfsLift_Stop(lift);
        return;
    }
    lift->state.target_pending = 0U;
}

/** 获取相对零点的实际高度，单位 m；未归零或反馈失效时返回 0.0 m。 */
float KfsLift_GetActualHeightM(const kfs_lift_t *lift)
{
    if (lift == 0 || lift->state.initialized == 0U ||
        lift->state.zeroed == 0U || lift->state.feedback_valid == 0U ||
        lift->internal.motor.state.online == 0U) {
        return 0.0f;
    }
    return lift->state.actual_height_m;
}

/** 返回模块是否已经归零且反馈在线。 */
uint8_t KfsLift_IsReady(const kfs_lift_t *lift)
{
    if (lift == 0) {
        return 0U;
    }
    return (lift->state.ready != 0U) ? 1U : 0U;
}
