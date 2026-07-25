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
    LIFT_MOTOR_FRONT_A = 0,
    LIFT_MOTOR_FRONT_B,
    LIFT_MOTOR_REAR_A,
    LIFT_MOTOR_REAR_B,
    LIFT_MOTOR_COUNT
};

enum {
    LIFT_GROUP_CAPACITY = 2
};

static const lift_app_motor_config_t s_motor_config[LIFT_MOTOR_COUNT] =
    LIFT_APP_MOTOR_CONFIG_INIT;
static dji_motor_t s_motors[LIFT_MOTOR_COUNT];
static dji_motor_group_t s_groups[LIFT_GROUP_CAPACITY];
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

static uint8_t LiftApp_FeedbackIsRecent(uint8_t motor_index,
                                        uint32_t now_ms)
{
    const dji_motor_t *motor = &s_motors[motor_index];

    return (motor->state.online != 0U &&
            (uint32_t)(now_ms - motor->state.last_update_ms) <
                LIFT_APP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
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

        s_target_position_rad[LIFT_MOTOR_FRONT_A] =
            front_position_rad;
        s_target_position_rad[LIFT_MOTOR_FRONT_B] =
            front_position_rad;
    }
    if (LiftApp_IsFinite(rear_position_m) != 0U) {
        double rear_position_rad =
            (double)rear_position_m /
            LIFT_APP_METERS_PER_MOTOR_RAD_D;

        s_target_position_rad[LIFT_MOTOR_REAR_A] =
            rear_position_rad;
        s_target_position_rad[LIFT_MOTOR_REAR_B] =
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
        (float)((position_m[LIFT_MOTOR_FRONT_A] +
                 position_m[LIFT_MOTOR_FRONT_B]) * 0.5);
    g_comm_app_feedback.lift_rear_position_m =
        (float)((position_m[LIFT_MOTOR_REAR_A] +
                 position_m[LIFT_MOTOR_REAR_B]) * 0.5);
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
        LIFT_APP_METERS_PER_OUTPUT_RAD <= 0.0 ||
        LIFT_APP_MOTOR_REDUCTION_RATIO <= 0.0) {
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
