/**
 * @file    lift_app.c
 * @brief   四电机升降命令读取、连续位置控制与反馈
 */

#include "lift_app.h"

#include <float.h>

#include "comm_app.h"
#include "dji_motor.h"
#include "dji_motor_group.h"
#include "fdcan.h"
#include "main.h"

#define LIFT_APP_RAD_TO_DEG_D 57.2957795130823208768
#define LIFT_APP_DEG_TO_RAD_D 0.01745329251994329577
#define LIFT_APP_METERS_PER_MOTOR_RAD_D \
    (LIFT_APP_METERS_PER_OUTPUT_RAD / LIFT_APP_MOTOR_REDUCTION_RATIO)

enum {
    LIFT_MOTOR_LF = 0,
    LIFT_MOTOR_RF,
    LIFT_MOTOR_RB,
    LIFT_MOTOR_LB,
    LIFT_MOTOR_COUNT
};

enum {
    LIFT_GROUP_CAPACITY = 2
};

enum {
    LIFT_SYNC_PAIR_FRONT = 0,
    LIFT_SYNC_PAIR_REAR,
    LIFT_SYNC_PAIR_COUNT
};

typedef struct {
    double last_error_deg;
    uint8_t derivative_started;
} lift_app_sync_controller_t;

static const lift_app_motor_config_t s_motor_config[LIFT_MOTOR_COUNT] =
    LIFT_APP_MOTOR_CONFIG_INIT;
static const uint8_t
    s_sync_motor_indexes[LIFT_SYNC_PAIR_COUNT][2] = {
        {LIFT_MOTOR_LF, LIFT_MOTOR_RF},
        {LIFT_MOTOR_LB, LIFT_MOTOR_RB},
    };
static dji_motor_t s_motors[LIFT_MOTOR_COUNT];
static dji_motor_group_t s_groups[LIFT_GROUP_CAPACITY];
static lift_app_sync_controller_t
    s_sync_controllers[LIFT_SYNC_PAIR_COUNT];
static lift_app_sync_controller_t s_four_motor_sync_controller;
static double s_target_position_rad[LIFT_MOTOR_COUNT];
static uint8_t s_group_count;
static uint8_t s_initialized;
static uint8_t s_control_timer_started;
static uint32_t s_last_control_ms;

static uint8_t LiftApp_IsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static uint8_t LiftApp_IsFiniteDouble(double value)
{
    return (value >= -DBL_MAX && value <= DBL_MAX) ? 1U : 0U;
}

static double LiftApp_ClampDouble(double value,
                                 double minimum,
                                 double maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static uint8_t LiftApp_FeedbackIsRecent(uint8_t motor_index,
                                        uint32_t now_ms)
{
    const dji_motor_t *motor = &s_motors[motor_index];

    return (motor->state.online != 0U &&
            (uint32_t)(now_ms - motor->state.last_update_ms) <
                LIFT_APP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static void LiftApp_ResetSyncController(
    lift_app_sync_controller_t *controller)
{
    controller->last_error_deg = 0.0;
    controller->derivative_started = 0U;
}

static double LiftApp_RunSyncController(
    lift_app_sync_controller_t *controller,
    double error_deg,
    uint32_t elapsed_ms,
    float kp_rpm_per_deg,
    float kd_rpm_s_per_deg,
    float output_limit_rpm)
{
    double output_rpm =
        (double)kp_rpm_per_deg * error_deg;

    if (controller->derivative_started != 0U &&
        elapsed_ms > 0U &&
        kd_rpm_s_per_deg > 0.0f) {
        double dt_s = (double)elapsed_ms * 0.001;
        double derivative_deg_s =
            (error_deg - controller->last_error_deg) / dt_s;

        output_rpm +=
            (double)kd_rpm_s_per_deg * derivative_deg_s;
    }
    controller->last_error_deg = error_deg;
    controller->derivative_started = 1U;

    return LiftApp_ClampDouble(
        output_rpm,
        -(double)output_limit_rpm,
        (double)output_limit_rpm);
}

static void LiftApp_ClearSyncPair(uint8_t pair_index)
{
    uint8_t motor_a_index = s_sync_motor_indexes[pair_index][0];
    uint8_t motor_b_index = s_sync_motor_indexes[pair_index][1];

    LiftApp_ResetSyncController(&s_sync_controllers[pair_index]);
    (void)dji_motor_set_speed_correction(&s_motors[motor_a_index], 0.0f);
    (void)dji_motor_set_speed_correction(&s_motors[motor_b_index], 0.0f);
}

static void LiftApp_UpdateSyncPair(uint8_t pair_index,
                                   uint32_t now_ms,
                                   uint32_t elapsed_ms)
{
    uint8_t motor_a_index = s_sync_motor_indexes[pair_index][0];
    uint8_t motor_b_index = s_sync_motor_indexes[pair_index][1];
    lift_app_sync_controller_t *controller =
        &s_sync_controllers[pair_index];
    dji_motor_feedback_t feedback_a;
    dji_motor_feedback_t feedback_b;
    double position_a_deg;
    double position_b_deg;
    double error_deg;
    double output_rpm;
    float correction_a_rpm;
    float correction_b_rpm;
    dji_motor_status_t status_a;
    dji_motor_status_t status_b;

    if (LiftApp_FeedbackIsRecent(motor_a_index, now_ms) == 0U ||
        LiftApp_FeedbackIsRecent(motor_b_index, now_ms) == 0U ||
        dji_motor_get_feedback(&s_motors[motor_a_index], &feedback_a) !=
            DJI_MOTOR_STATUS_OK ||
        dji_motor_get_feedback(&s_motors[motor_b_index], &feedback_b) !=
            DJI_MOTOR_STATUS_OK ||
        feedback_a.multi_turn.valid == 0U ||
        feedback_b.multi_turn.valid == 0U) {
        LiftApp_ClearSyncPair(pair_index);
        return;
    }

    position_a_deg =
        feedback_a.multi_turn.angle_deg *
        (double)s_motor_config[motor_a_index].direction;
    position_b_deg =
        feedback_b.multi_turn.angle_deg *
        (double)s_motor_config[motor_b_index].direction;
    error_deg = position_a_deg - position_b_deg;
    if (LiftApp_IsFiniteDouble(error_deg) == 0U) {
        LiftApp_ClearSyncPair(pair_index);
        return;
    }

    output_rpm = LiftApp_RunSyncController(
        controller,
        error_deg,
        elapsed_ms,
        LIFT_APP_SYNC_KP_RPM_PER_DEG,
        LIFT_APP_SYNC_KD_RPM_S_PER_DEG,
        LIFT_APP_SYNC_MAX_CORRECTION_RPM);
    correction_a_rpm =
        (float)(-(double)s_motor_config[motor_a_index].direction *
                output_rpm);
    correction_b_rpm =
        (float)((double)s_motor_config[motor_b_index].direction *
                output_rpm);

    status_a = dji_motor_set_speed_correction(
        &s_motors[motor_a_index], correction_a_rpm);
    status_b = dji_motor_set_speed_correction(
        &s_motors[motor_b_index], correction_b_rpm);
    if (status_a != DJI_MOTOR_STATUS_OK ||
        status_b != DJI_MOTOR_STATUS_OK) {
        LiftApp_ClearSyncPair(pair_index);
    }
}

static void LiftApp_UpdateFourMotorSync(uint32_t now_ms,
                                        uint32_t elapsed_ms)
{
    dji_motor_feedback_t feedback[LIFT_MOTOR_COUNT];
    double position_deg[LIFT_MOTOR_COUNT];
    double physical_correction_rpm[LIFT_MOTOR_COUNT];
    float base_correction_rpm[LIFT_MOTOR_COUNT];
    float combined_correction_rpm[LIFT_MOTOR_COUNT];
    double front_average_deg;
    double rear_average_deg;
    double error_deg;
    double output_rpm;
    double maximum_correction_rpm = 0.0;
    double scale = 1.0;

    if (s_target_position_rad[LIFT_MOTOR_LF] !=
        s_target_position_rad[LIFT_MOTOR_LB]) {
        LiftApp_ResetSyncController(&s_four_motor_sync_controller);
        return;
    }

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        if (LiftApp_FeedbackIsRecent(i, now_ms) == 0U ||
            dji_motor_get_feedback(&s_motors[i], &feedback[i]) !=
                DJI_MOTOR_STATUS_OK ||
            feedback[i].multi_turn.valid == 0U) {
            LiftApp_ResetSyncController(&s_four_motor_sync_controller);
            return;
        }
        position_deg[i] =
            feedback[i].multi_turn.angle_deg *
            (double)s_motor_config[i].direction;
        if (LiftApp_IsFiniteDouble(position_deg[i]) == 0U) {
            LiftApp_ResetSyncController(&s_four_motor_sync_controller);
            return;
        }
    }

    front_average_deg =
        (position_deg[LIFT_MOTOR_LF] +
         position_deg[LIFT_MOTOR_RF]) * 0.5;
    rear_average_deg =
        (position_deg[LIFT_MOTOR_LB] +
         position_deg[LIFT_MOTOR_RB]) * 0.5;
    error_deg = front_average_deg - rear_average_deg;
    if (LiftApp_IsFiniteDouble(error_deg) == 0U) {
        LiftApp_ResetSyncController(&s_four_motor_sync_controller);
        return;
    }

    output_rpm = LiftApp_RunSyncController(
        &s_four_motor_sync_controller,
        error_deg,
        elapsed_ms,
        LIFT_APP_FOUR_SYNC_KP_RPM_PER_DEG,
        LIFT_APP_FOUR_SYNC_KD_RPM_S_PER_DEG,
        LIFT_APP_FOUR_SYNC_MAX_CORRECTION_RPM);
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        double cross_correction_rpm =
            (i == LIFT_MOTOR_LF || i == LIFT_MOTOR_RF) ?
            -output_rpm : output_rpm;
        double magnitude;

        base_correction_rpm[i] =
            s_motors[i].state.speed_correction_rpm;
        physical_correction_rpm[i] =
            (double)base_correction_rpm[i] *
            (double)s_motor_config[i].direction +
            cross_correction_rpm;
        magnitude = (physical_correction_rpm[i] >= 0.0) ?
                    physical_correction_rpm[i] :
                    -physical_correction_rpm[i];
        if (magnitude > maximum_correction_rpm) {
            maximum_correction_rpm = magnitude;
        }
    }

    if (maximum_correction_rpm >
        (double)LIFT_APP_FOUR_SYNC_MAX_CORRECTION_RPM) {
        scale =
            (double)LIFT_APP_FOUR_SYNC_MAX_CORRECTION_RPM /
            maximum_correction_rpm;
    }
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        combined_correction_rpm[i] =
            (float)(physical_correction_rpm[i] * scale *
                    (double)s_motor_config[i].direction);
    }

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        if (dji_motor_set_speed_correction(
                &s_motors[i], combined_correction_rpm[i]) !=
            DJI_MOTOR_STATUS_OK) {
            for (uint8_t j = 0U; j < LIFT_MOTOR_COUNT; ++j) {
                (void)dji_motor_set_speed_correction(
                    &s_motors[j], base_correction_rpm[j]);
            }
            LiftApp_ResetSyncController(&s_four_motor_sync_controller);
            return;
        }
    }
}

static dji_motor_status_t LiftApp_InitGroups(void)
{
    uint8_t group_index_by_bank[2] = {0xFFU, 0xFFU};

    s_group_count = 0U;
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        uint8_t bank = (s_motor_config[i].motor_id <= 4U) ? 0U : 1U;
        uint8_t group_index = group_index_by_bank[bank];

        if (group_index == 0xFFU) {
            if (s_group_count >= LIFT_GROUP_CAPACITY) {
                return DJI_MOTOR_STATUS_INVALID_CONFIG;
            }
            group_index = s_group_count;
            s_group_count++;
            group_index_by_bank[bank] = group_index;
            s_groups[group_index].config.hfdcan = &hfdcan2;
            s_groups[group_index].config.command_id =
                (bank == 0U) ? DJI_MOTOR_CMD_ID_1_TO_4 :
                               DJI_MOTOR_CMD_ID_5_TO_8;
            s_groups[group_index].config.motor_count = 0U;
        }

        s_groups[group_index].config.motors[
            s_groups[group_index].config.motor_count] = &s_motors[i];
        s_groups[group_index].config.motor_count++;
    }

    for (uint8_t i = 0U; i < s_group_count; ++i) {
        dji_motor_status_t status =
            dji_motor_group_init(&s_groups[i]);

        if (status != DJI_MOTOR_STATUS_OK) {
            return status;
        }
    }
    return DJI_MOTOR_STATUS_OK;
}

static void LiftApp_UpdateTargetsFromCommand(void)
{
    float front_position_m;
    float rear_position_m;

    if (g_comm_app_command.valid == 0U) {
        return;
    }

    front_position_m =
        g_comm_app_command.lift_front_position_m;
    rear_position_m =
        g_comm_app_command.lift_rear_position_m;

    if (LiftApp_IsFinite(front_position_m) != 0U) {
        double front_position_rad =
            (double)front_position_m /
            LIFT_APP_METERS_PER_MOTOR_RAD_D;

        s_target_position_rad[LIFT_MOTOR_LF] =
            front_position_rad;
        s_target_position_rad[LIFT_MOTOR_RF] =
            front_position_rad;
    }
    if (LiftApp_IsFinite(rear_position_m) != 0U) {
        double rear_position_rad =
            (double)rear_position_m /
            LIFT_APP_METERS_PER_MOTOR_RAD_D;

        s_target_position_rad[LIFT_MOTOR_LB] =
            rear_position_rad;
        s_target_position_rad[LIFT_MOTOR_RB] =
            rear_position_rad;
    }
}

static void LiftApp_UpdateFeedback(uint32_t now_ms)
{
    double position_m[LIFT_MOTOR_COUNT];

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        dji_motor_feedback_t feedback;

        if (LiftApp_FeedbackIsRecent(i, now_ms) == 0U ||
            dji_motor_get_feedback(&s_motors[i], &feedback) !=
                DJI_MOTOR_STATUS_OK ||
            feedback.multi_turn.valid == 0U) {
            g_comm_app_feedback.lift_front_position_m = 0.0f;
            g_comm_app_feedback.lift_rear_position_m = 0.0f;
            g_comm_app_feedback.lift_valid = 0U;
            return;
        }
        position_m[i] =
            feedback.multi_turn.angle_deg *
            LIFT_APP_DEG_TO_RAD_D *
            LIFT_APP_METERS_PER_MOTOR_RAD_D *
            (double)s_motor_config[i].direction;
    }

    g_comm_app_feedback.lift_front_position_m =
        (float)((position_m[LIFT_MOTOR_LF] +
                 position_m[LIFT_MOTOR_RF]) * 0.5);
    g_comm_app_feedback.lift_rear_position_m =
        (float)((position_m[LIFT_MOTOR_LB] +
                 position_m[LIFT_MOTOR_RB]) * 0.5);
    g_comm_app_feedback.lift_valid = 1U;
}

void LiftApp_Init(void)
{
    dji_motor_status_t status = DJI_MOTOR_STATUS_OK;

    if (s_initialized != 0U) {
        return;
    }

    g_comm_app_feedback.lift_front_position_m = 0.0f;
    g_comm_app_feedback.lift_rear_position_m = 0.0f;
    g_comm_app_feedback.lift_valid = 0U;

    if (LiftApp_IsFiniteDouble(LIFT_APP_METERS_PER_OUTPUT_RAD) == 0U ||
        LiftApp_IsFiniteDouble(LIFT_APP_MOTOR_REDUCTION_RATIO) == 0U ||
        LiftApp_IsFinite(LIFT_APP_SYNC_KP_RPM_PER_DEG) == 0U ||
        LiftApp_IsFinite(LIFT_APP_SYNC_KD_RPM_S_PER_DEG) == 0U ||
        LiftApp_IsFinite(LIFT_APP_SYNC_MAX_CORRECTION_RPM) == 0U ||
        LiftApp_IsFinite(LIFT_APP_FOUR_SYNC_KP_RPM_PER_DEG) == 0U ||
        LiftApp_IsFinite(LIFT_APP_FOUR_SYNC_KD_RPM_S_PER_DEG) == 0U ||
        LiftApp_IsFinite(LIFT_APP_FOUR_SYNC_MAX_CORRECTION_RPM) == 0U ||
        LIFT_APP_METERS_PER_OUTPUT_RAD <= 0.0 ||
        LIFT_APP_MOTOR_REDUCTION_RATIO <= 0.0 ||
        LIFT_APP_SYNC_KP_RPM_PER_DEG < 0.0f ||
        LIFT_APP_SYNC_KD_RPM_S_PER_DEG < 0.0f ||
        LIFT_APP_SYNC_MAX_CORRECTION_RPM <= 0.0f ||
        LIFT_APP_FOUR_SYNC_KP_RPM_PER_DEG < 0.0f ||
        LIFT_APP_FOUR_SYNC_KD_RPM_S_PER_DEG < 0.0f ||
        LIFT_APP_FOUR_SYNC_MAX_CORRECTION_RPM <= 0.0f) {
        return;
    }

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        if (s_motor_config[i].direction != 1 &&
            s_motor_config[i].direction != -1) {
            status = DJI_MOTOR_STATUS_INVALID_CONFIG;
            break;
        }

        s_motors[i].config.motor_id = s_motor_config[i].motor_id;
        s_motors[i].config.offline_timeout_ms =
            LIFT_APP_OFFLINE_TIMEOUT_MS;
        s_motors[i].config.control_period_ms =
            LIFT_APP_CONTROL_PERIOD_MS;
        s_motors[i].config.current_limit =
            s_motor_config[i].current_limit;
        s_motors[i].config.max_speed_rpm =
            s_motor_config[i].max_speed_rpm;
        s_motors[i].config.position_pid.kp =
            s_motor_config[i].position_kp;
        s_motors[i].config.position_pid.ki =
            s_motor_config[i].position_ki;
        s_motors[i].config.position_pid.kd =
            s_motor_config[i].position_kd;
        s_motors[i].config.position_pid.integral_limit =
            LIFT_APP_POSITION_INTEGRAL_LIMIT;
        s_motors[i].config.speed_pid.kp =
            s_motor_config[i].speed_kp;
        s_motors[i].config.speed_pid.ki =
            s_motor_config[i].speed_ki;
        s_motors[i].config.speed_pid.kd =
            s_motor_config[i].speed_kd;
        s_motors[i].config.speed_pid.integral_limit =
            LIFT_APP_SPEED_INTEGRAL_LIMIT;

        status = dji_motor_init(&s_motors[i]);
        if (status != DJI_MOTOR_STATUS_OK) {
            break;
        }
        s_target_position_rad[i] = 0.0;
    }

    if (status == DJI_MOTOR_STATUS_OK) {
        status = LiftApp_InitGroups();
    }
    if (status == DJI_MOTOR_STATUS_OK) {
        for (uint8_t i = 0U; i < LIFT_SYNC_PAIR_COUNT; ++i) {
            LiftApp_ClearSyncPair(i);
        }
        LiftApp_ResetSyncController(&s_four_motor_sync_controller);
        s_control_timer_started = 0U;
        s_last_control_ms = 0U;
        s_initialized = 1U;
    }
}

void LiftApp_RunPeriodic(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;

    if (s_initialized == 0U) {
        g_comm_app_feedback.lift_valid = 0U;
        return;
    }

    now_ms = HAL_GetTick();
    if (s_control_timer_started == 0U) {
        s_last_control_ms = now_ms;
        s_control_timer_started = 1U;
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - s_last_control_ms);
    if (elapsed_ms < LIFT_APP_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;

    LiftApp_UpdateTargetsFromCommand();
    for (uint8_t i = 0U; i < LIFT_SYNC_PAIR_COUNT; ++i) {
        LiftApp_UpdateSyncPair(i, now_ms, elapsed_ms);
    }
    LiftApp_UpdateFourMotorSync(now_ms, elapsed_ms);
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        (void)dji_motor_position_control(
            &s_motors[i],
            s_target_position_rad[i] *
                LIFT_APP_RAD_TO_DEG_D *
                (double)s_motor_config[i].direction);
    }

    for (uint8_t i = 0U; i < s_group_count; ++i) {
        (void)dji_motor_group_update(&s_groups[i], now_ms);
    }
    LiftApp_UpdateFeedback(now_ms);
}
