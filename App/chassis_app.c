/**
 * @file    chassis_app.c
 * @brief   四麦轮底盘命令读取、运动学、DJI 控制与实际速度反馈
 */

#include "chassis_app.h"

#include <float.h>

#include "comm_app.h"
#include "dji_motor.h"
#include "dji_motor_group.h"
#include "fdcan.h"
#include "main.h"

#define CHASSIS_APP_PI_F 3.14159265358979323846f

enum {
    CHASSIS_WHEEL_RF = 0,
    CHASSIS_WHEEL_LF,
    CHASSIS_WHEEL_LB,
    CHASSIS_WHEEL_RB,
    CHASSIS_MOTOR_COUNT
};

enum {
    CHASSIS_GROUP_CAPACITY = 2
};

static const chassis_app_motor_config_t
    s_motor_config[CHASSIS_MOTOR_COUNT] =
        CHASSIS_APP_MOTOR_CONFIG_INIT;
static dji_motor_t s_motors[CHASSIS_MOTOR_COUNT];
static dji_motor_group_t s_groups[CHASSIS_GROUP_CAPACITY];
static float s_target_motor_rpm[CHASSIS_MOTOR_COUNT];
static uint8_t s_group_count;
static uint8_t s_initialized;
static uint8_t s_control_timer_started;
static uint32_t s_last_control_ms;

static uint8_t ChassisApp_IsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static uint8_t ChassisApp_DirectionIsValid(float direction)
{
    return (direction == 1.0f || direction == -1.0f) ? 1U : 0U;
}

static dji_motor_status_t ChassisApp_InitGroups(void)
{
    uint8_t group_index_by_bank[2] = {0xFFU, 0xFFU};

    s_group_count = 0U;
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        uint8_t bank = (s_motor_config[i].motor_id <= 4U) ? 0U : 1U;
        uint8_t group_index = group_index_by_bank[bank];

        if (group_index == 0xFFU) {
            if (s_group_count >= CHASSIS_GROUP_CAPACITY) {
                return DJI_MOTOR_STATUS_INVALID_CONFIG;
            }
            group_index = s_group_count;
            s_group_count++;
            group_index_by_bank[bank] = group_index;
            s_groups[group_index].config.hfdcan = &hfdcan1;
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

static void ChassisApp_UpdateTargetFromCommand(void)
{
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
    float rotation_radius_m;
    float motor_rpm_per_m_s;

    if (g_comm_app_command.valid == 0U) {
        vx_m_s = 0.0f;
        vy_m_s = 0.0f;
        wz_rad_s = 0.0f;
    } else {
        vx_m_s = g_comm_app_command.chassis_vx_m_s;
        vy_m_s = g_comm_app_command.chassis_vy_m_s;
        wz_rad_s = g_comm_app_command.chassis_wz_rad_s;

        if (ChassisApp_IsFinite(vx_m_s) == 0U ||
            ChassisApp_IsFinite(vy_m_s) == 0U ||
            ChassisApp_IsFinite(wz_rad_s) == 0U) {
            vx_m_s = 0.0f;
            vy_m_s = 0.0f;
            wz_rad_s = 0.0f;
        }
    }

    vx_m_s *= CHASSIS_APP_VX_DIRECTION * CHASSIS_APP_VX_SCALE;
    vy_m_s *= CHASSIS_APP_VY_DIRECTION * CHASSIS_APP_VY_SCALE;
    wz_rad_s *= CHASSIS_APP_WZ_SCALE;

    rotation_radius_m =
        (CHASSIS_APP_LENGTH_M + CHASSIS_APP_WIDTH_M) * 0.5f;
    motor_rpm_per_m_s =
        (60.0f * CHASSIS_APP_MOTOR_REDUCTION_RATIO) /
        (CHASSIS_APP_PI_F * CHASSIS_APP_WHEEL_DIAMETER_M);

    s_target_motor_rpm[CHASSIS_WHEEL_RF] =
        (vx_m_s - vy_m_s - rotation_radius_m * wz_rad_s) *
        motor_rpm_per_m_s;
    s_target_motor_rpm[CHASSIS_WHEEL_LF] =
        (vx_m_s + vy_m_s + rotation_radius_m * wz_rad_s) *
        motor_rpm_per_m_s;
    s_target_motor_rpm[CHASSIS_WHEEL_LB] =
        (vx_m_s - vy_m_s + rotation_radius_m * wz_rad_s) *
        motor_rpm_per_m_s;
    s_target_motor_rpm[CHASSIS_WHEEL_RB] =
        (vx_m_s + vy_m_s - rotation_radius_m * wz_rad_s) *
        motor_rpm_per_m_s;
}

static uint8_t ChassisApp_ReadWheelLinearSpeeds(
    uint32_t now_ms,
    float wheel_m_s[CHASSIS_MOTOR_COUNT])
{
    const float motor_rpm_to_wheel_m_s =
        (CHASSIS_APP_PI_F * CHASSIS_APP_WHEEL_DIAMETER_M) /
        (60.0f * CHASSIS_APP_MOTOR_REDUCTION_RATIO);

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        dji_motor_feedback_t feedback;

        if (s_motors[i].state.online == 0U ||
            (uint32_t)(now_ms - s_motors[i].state.last_update_ms) >=
                CHASSIS_APP_OFFLINE_TIMEOUT_MS ||
            dji_motor_get_feedback(&s_motors[i], &feedback) !=
                DJI_MOTOR_STATUS_OK) {
            return 0U;
        }

        wheel_m_s[i] =
            (float)feedback.speed_rpm *
            (float)s_motor_config[i].direction *
            motor_rpm_to_wheel_m_s;
    }
    return 1U;
}

static void ChassisApp_UpdateFeedback(uint32_t now_ms)
{
    float wheel_m_s[CHASSIS_MOTOR_COUNT];
    float rotation_radius_m;
    float vx_internal_m_s;
    float vy_internal_m_s;
    float wz_rad_s;

    if (ChassisApp_ReadWheelLinearSpeeds(now_ms, wheel_m_s) == 0U) {
        g_comm_app_feedback.chassis_vx_m_s = 0.0f;
        g_comm_app_feedback.chassis_vy_m_s = 0.0f;
        g_comm_app_feedback.chassis_wz_rad_s = 0.0f;
        g_comm_app_feedback.chassis_valid = 0U;
        return;
    }

    rotation_radius_m =
        (CHASSIS_APP_LENGTH_M + CHASSIS_APP_WIDTH_M) * 0.5f;
    vx_internal_m_s =
        (wheel_m_s[CHASSIS_WHEEL_RF] +
         wheel_m_s[CHASSIS_WHEEL_LF] +
         wheel_m_s[CHASSIS_WHEEL_LB] +
         wheel_m_s[CHASSIS_WHEEL_RB]) * 0.25f;
    vy_internal_m_s =
        (-wheel_m_s[CHASSIS_WHEEL_RF] +
          wheel_m_s[CHASSIS_WHEEL_LF] -
          wheel_m_s[CHASSIS_WHEEL_LB] +
          wheel_m_s[CHASSIS_WHEEL_RB]) * 0.25f;
    wz_rad_s =
        (-wheel_m_s[CHASSIS_WHEEL_RF] +
          wheel_m_s[CHASSIS_WHEEL_LF] +
          wheel_m_s[CHASSIS_WHEEL_LB] -
          wheel_m_s[CHASSIS_WHEEL_RB]) /
        (4.0f * rotation_radius_m);

    /*
     * 返回上位机坐标方向；不除以 SCALE，反馈表示底盘实际执行速度。
     * 方向系数为 +/-1，其逆变换等于自身。
     */
    g_comm_app_feedback.chassis_vx_m_s =
        vx_internal_m_s * CHASSIS_APP_VX_DIRECTION;
    g_comm_app_feedback.chassis_vy_m_s =
        vy_internal_m_s * CHASSIS_APP_VY_DIRECTION;
    g_comm_app_feedback.chassis_wz_rad_s = wz_rad_s;
    g_comm_app_feedback.chassis_valid = 1U;
}

void ChassisApp_Init(void)
{
    dji_motor_status_t status = DJI_MOTOR_STATUS_OK;

    if (s_initialized != 0U) {
        return;
    }

    g_comm_app_feedback.chassis_vx_m_s = 0.0f;
    g_comm_app_feedback.chassis_vy_m_s = 0.0f;
    g_comm_app_feedback.chassis_wz_rad_s = 0.0f;
    g_comm_app_feedback.chassis_valid = 0U;

    if (ChassisApp_DirectionIsValid(CHASSIS_APP_VX_DIRECTION) == 0U ||
        ChassisApp_DirectionIsValid(CHASSIS_APP_VY_DIRECTION) == 0U ||
        ChassisApp_IsFinite(CHASSIS_APP_VX_SCALE) == 0U ||
        ChassisApp_IsFinite(CHASSIS_APP_VY_SCALE) == 0U ||
        ChassisApp_IsFinite(CHASSIS_APP_WZ_SCALE) == 0U ||
        ChassisApp_IsFinite(CHASSIS_APP_WHEEL_DIAMETER_M) == 0U ||
        ChassisApp_IsFinite(CHASSIS_APP_LENGTH_M) == 0U ||
        ChassisApp_IsFinite(CHASSIS_APP_WIDTH_M) == 0U ||
        ChassisApp_IsFinite(CHASSIS_APP_MOTOR_REDUCTION_RATIO) == 0U ||
        CHASSIS_APP_WHEEL_DIAMETER_M <= 0.0f ||
        CHASSIS_APP_LENGTH_M + CHASSIS_APP_WIDTH_M <= 0.0f ||
        CHASSIS_APP_MOTOR_REDUCTION_RATIO <= 0.0f) {
        return;
    }

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        if (s_motor_config[i].direction != 1 &&
            s_motor_config[i].direction != -1) {
            status = DJI_MOTOR_STATUS_INVALID_CONFIG;
            break;
        }

        s_motors[i].config.motor_id = s_motor_config[i].motor_id;
        s_motors[i].config.offline_timeout_ms =
            CHASSIS_APP_OFFLINE_TIMEOUT_MS;
        s_motors[i].config.control_period_ms =
            CHASSIS_APP_CONTROL_PERIOD_MS;
        s_motors[i].config.current_limit = CHASSIS_APP_CURRENT_LIMIT;
        s_motors[i].config.max_speed_rpm = 0.0f;
        s_motors[i].config.speed_pid.kp = s_motor_config[i].speed_kp;
        s_motors[i].config.speed_pid.ki = s_motor_config[i].speed_ki;
        s_motors[i].config.speed_pid.kd = s_motor_config[i].speed_kd;
        s_motors[i].config.speed_pid.integral_limit =
            CHASSIS_APP_SPEED_INTEGRAL_LIMIT;
        s_motors[i].config.position_pid.kp = 0.0f;
        s_motors[i].config.position_pid.ki = 0.0f;
        s_motors[i].config.position_pid.kd = 0.0f;
        s_motors[i].config.position_pid.integral_limit = 0.0f;

        status = dji_motor_init(&s_motors[i]);
        if (status != DJI_MOTOR_STATUS_OK) {
            break;
        }
        s_target_motor_rpm[i] = 0.0f;
    }

    if (status == DJI_MOTOR_STATUS_OK) {
        status = ChassisApp_InitGroups();
    }
    if (status == DJI_MOTOR_STATUS_OK) {
        s_control_timer_started = 0U;
        s_last_control_ms = 0U;
        s_initialized = 1U;
    }
}

void ChassisApp_RunPeriodic(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;

    if (s_initialized == 0U) {
        g_comm_app_feedback.chassis_valid = 0U;
        return;
    }

    now_ms = HAL_GetTick();
    if (s_control_timer_started == 0U) {
        s_last_control_ms = now_ms;
        s_control_timer_started = 1U;
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - s_last_control_ms);
    if (elapsed_ms < CHASSIS_APP_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;

    ChassisApp_UpdateTargetFromCommand();
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        (void)dji_motor_speed_control(
            &s_motors[i],
            s_target_motor_rpm[i] *
                (float)s_motor_config[i].direction);
    }
    for (uint8_t i = 0U; i < s_group_count; ++i) {
        (void)dji_motor_group_update(&s_groups[i], now_ms);
    }

    ChassisApp_UpdateFeedback(now_ms);
}
