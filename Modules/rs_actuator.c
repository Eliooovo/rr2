#include "rs_actuator.h"

#include <stddef.h>

#include "main.h"
#include "rs_motor.h"
#include "rs_motor_instances.h"

#define RS_ACTUATOR_CONTROL_PERIOD_MS      20U
#define RS_ACTUATOR_OFFLINE_TIMEOUT_MS     RS_MOTOR_DEFAULT_OFFLINE_MS
#define RS_ACTUATOR_DEFAULT_SPEED_DEG_S    30.0f

typedef struct {
    RsMotorInstanceId motor_instance;
    int8_t direction;
    float min_deg;
    float max_deg;
    float default_speed_deg_s;
} RsActuatorConfig;

typedef struct {
    float target_deg;
    float speed_deg_s;
    uint8_t enabled;
} RsActuatorState;

/* User-facing position actuators. The lower RS motor layer only sees generic
 * motor objects and never needs to know the mechanism where they are mounted. */
static const RsActuatorConfig s_actuator_config[RS_ACTUATOR_COUNT] = {
    [RS_ACTUATOR_0] = {
        .motor_instance = RS_MOTOR_INSTANCE_RS00_ID3,
        .direction = 1,
        .min_deg = -720.0f,
        .max_deg = 720.0f,
        .default_speed_deg_s = RS_ACTUATOR_DEFAULT_SPEED_DEG_S,
    },
};

static RsActuatorState s_actuator_state[RS_ACTUATOR_COUNT];
static uint32_t s_last_control_ms;

volatile RsActuatorBootTestPhase g_rs_actuator_boot_test_phase =
    RS_ACTUATOR_BOOT_TEST_PHASE_DISABLED;
volatile uint32_t g_rs_actuator_csp_success_count;
volatile uint32_t g_rs_actuator_csp_error_count;
volatile uint8_t g_rs_actuator_last_csp_status;
volatile float g_rs_actuator_last_csp_target_rad;
volatile float g_rs_actuator_last_csp_speed_rad_s;

#if RS_ACTUATOR_BOOT_TEST_ENABLE
static uint8_t s_boot_test_active;
static uint8_t s_boot_test_started;
static uint8_t s_boot_test_online_seen;
static uint8_t s_boot_test_wakeup_sent;
static uint32_t s_boot_test_online_since_ms;
static uint32_t s_boot_test_start_ms;
#endif

static uint8_t RsActuator_IsValid(RsActuatorId actuator)
{
    return ((uint32_t)actuator < (uint32_t)RS_ACTUATOR_COUNT) ? 1U : 0U;
}

static uint8_t RsActuator_IsFinite(float value)
{
    return (value == value && value < 1000000.0f && value > -1000000.0f) ? 1U : 0U;
}

static float RsActuator_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static RsMotor *RsActuator_GetMotor(RsActuatorId actuator)
{
    if (RsActuator_IsValid(actuator) == 0U) {
        return NULL;
    }
    return RsMotorInstances_Get(s_actuator_config[actuator].motor_instance);
}

#if RS_ACTUATOR_BOOT_TEST_ENABLE
/* 上电测试只允许在所有 RS 执行器都持续收到反馈后开始。 */
static uint8_t RsActuator_AllMotorsRecentlyOnline(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < (uint8_t)RS_ACTUATOR_COUNT; ++i) {
        if (RsActuator_IsOnline((RsActuatorId)i, now_ms) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static void RsActuator_RunBootTest(uint32_t now_ms)
{
    RsMotor *motor;

    if (s_boot_test_active == 0U) {
        return;
    }

    if (s_boot_test_started == 0U) {
        if (RsActuator_AllMotorsRecentlyOnline(now_ms) == 0U) {
            g_rs_actuator_boot_test_phase =
                RS_ACTUATOR_BOOT_TEST_PHASE_WAIT_FEEDBACK;
            /* 某些固件在失能状态不会主动上报反馈。先发一次无运动
             * 的使能帧唤醒电机，随后再等待反馈，避免“等在线”与
             * “必须先使能才在线”的死锁。 */
            motor = RsActuator_GetMotor(RS_ACTUATOR_0);
            if (motor != NULL && s_boot_test_wakeup_sent == 0U) {
                if (rs_motor_enable(motor) == RS_MOTOR_STATUS_OK) {
                    s_boot_test_wakeup_sent = 1U;
                }
            }

            /* 反馈中断后重新等待一段连续在线时间。 */
            s_boot_test_online_seen = 0U;
            return;
        }

        if (s_boot_test_online_seen == 0U) {
            s_boot_test_online_seen = 1U;
            s_boot_test_online_since_ms = now_ms;
            g_rs_actuator_boot_test_phase =
                RS_ACTUATOR_BOOT_TEST_PHASE_WAIT_STABLE;
            return;
        }

        if ((uint32_t)(now_ms - s_boot_test_online_since_ms) <
            RS_ACTUATOR_BOOT_TEST_DELAY_MS) {
            return;
        }

        for (uint8_t i = 0U; i < (uint8_t)RS_ACTUATOR_COUNT; ++i) {
            RsActuator_SetZeroToCurrent((RsActuatorId)i);
            (void)RsActuator_SetTargetDeg((RsActuatorId)i,
                                          RS_ACTUATOR_BOOT_TEST_TARGET_DEG);
            s_actuator_state[i].speed_deg_s = RS_ACTUATOR_BOOT_TEST_SPEED_DEG_S;
        }

        s_boot_test_start_ms = now_ms;
        s_boot_test_started = 1U;
        g_rs_actuator_boot_test_phase = RS_ACTUATOR_BOOT_TEST_PHASE_RUNNING;
        return;
    }

    if ((uint32_t)(now_ms - s_boot_test_start_ms) >=
        RS_ACTUATOR_BOOT_TEST_DURATION_MS) {
        for (uint8_t i = 0U; i < (uint8_t)RS_ACTUATOR_COUNT; ++i) {
            RsActuator_Disable((RsActuatorId)i);
        }
        s_boot_test_active = 0U;
        g_rs_actuator_boot_test_phase = RS_ACTUATOR_BOOT_TEST_PHASE_DONE;
    }
}
#endif

void RsActuator_Init(void)
{
    for (uint8_t i = 0U; i < (uint8_t)RS_ACTUATOR_COUNT; ++i) {
        s_actuator_state[i].target_deg = 0.0f;
        s_actuator_state[i].speed_deg_s = s_actuator_config[i].default_speed_deg_s;
        s_actuator_state[i].enabled = 0U;
    }
    s_last_control_ms = 0U;
    g_rs_actuator_boot_test_phase =
        (RS_ACTUATOR_BOOT_TEST_ENABLE != 0U) ?
        RS_ACTUATOR_BOOT_TEST_PHASE_WAIT_FEEDBACK :
        RS_ACTUATOR_BOOT_TEST_PHASE_DISABLED;
    g_rs_actuator_csp_success_count = 0U;
    g_rs_actuator_csp_error_count = 0U;
    g_rs_actuator_last_csp_status = RS_ACTUATOR_CSP_STATUS_NOT_CALLED;
    g_rs_actuator_last_csp_target_rad = 0.0f;
    g_rs_actuator_last_csp_speed_rad_s = 0.0f;
#if RS_ACTUATOR_BOOT_TEST_ENABLE
    s_boot_test_active = 1U;
    s_boot_test_started = 0U;
    s_boot_test_online_seen = 0U;
    s_boot_test_wakeup_sent = 0U;
    s_boot_test_online_since_ms = 0U;
    s_boot_test_start_ms = 0U;
#endif
}

void RsActuator_RunPeriodic(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (s_last_control_ms == 0U) {
        s_last_control_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - s_last_control_ms) < RS_ACTUATOR_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;

    for (uint8_t i = 0U; i < (uint8_t)RS_ACTUATOR_COUNT; ++i) {
        RsMotor *motor = RsActuator_GetMotor((RsActuatorId)i);

        if (motor == NULL) {
            continue;
        }
        (void)rs_motor_update(motor, now_ms);
    }

#if RS_ACTUATOR_BOOT_TEST_ENABLE
    RsActuator_RunBootTest(now_ms);
#endif

    for (uint8_t i = 0U; i < (uint8_t)RS_ACTUATOR_COUNT; ++i) {
        RsMotor *motor = RsActuator_GetMotor((RsActuatorId)i);

        if (motor == NULL) {
            continue;
        }
        if (s_actuator_state[i].enabled == 0U) {
            continue;
        }
        if (RsActuator_IsOnline((RsActuatorId)i, now_ms) == 0U) {
            continue;
        }

        /* CSP mode uses the motor's internal position loop. The MCU only sends
         * target position and speed limit periodically. target_deg is relative
         * to the software zero, so convert it back to the protocol coordinate
         * by adding zero_offset_rad before transmission. */
        float target_rad = motor->internal.zero_offset_rad +
                           s_actuator_state[i].target_deg *
                               (float)s_actuator_config[i].direction *
                               RS_MOTOR_RAD_PER_DEG;
        float speed_rad_s = s_actuator_state[i].speed_deg_s *
                            RS_MOTOR_RAD_PER_DEG;
        RsMotorStatus status = rs_motor_csp_position_control(
            motor,
            speed_rad_s,
            target_rad);

        g_rs_actuator_last_csp_status = (uint8_t)status;
        g_rs_actuator_last_csp_target_rad = target_rad;
        g_rs_actuator_last_csp_speed_rad_s = speed_rad_s;
        if (status == RS_MOTOR_STATUS_OK) {
            g_rs_actuator_csp_success_count++;
        } else {
            g_rs_actuator_csp_error_count++;
        }
    }
}

uint8_t RsActuator_SetTargetDeg(RsActuatorId actuator, float position_deg)
{
    const RsActuatorConfig *config;
    RsActuatorState *state;

    if (RsActuator_IsValid(actuator) == 0U ||
        RsActuator_IsFinite(position_deg) == 0U) {
        return 0U;
    }

    config = &s_actuator_config[actuator];
    state = &s_actuator_state[actuator];
    state->target_deg = RsActuator_Clamp(position_deg, config->min_deg, config->max_deg);
    state->speed_deg_s = config->default_speed_deg_s;
    state->enabled = 1U;
    return 1U;
}

void RsActuator_SetZeroToCurrent(RsActuatorId actuator)
{
    RsMotor *motor = RsActuator_GetMotor(actuator);

    if (motor == NULL) {
        return;
    }
    motor->internal.zero_offset_rad = motor->feedback.total_position_rad;
    motor->feedback.position_rad = 0.0f;
    motor->feedback.angle_deg = 0.0f;
    if (RsActuator_IsValid(actuator) != 0U) {
        s_actuator_state[actuator].target_deg = 0.0f;
    }
}

void RsActuator_Disable(RsActuatorId actuator)
{
    RsMotor *motor = RsActuator_GetMotor(actuator);

    if (RsActuator_IsValid(actuator) == 0U) {
        return;
    }
    s_actuator_state[actuator].enabled = 0U;
    if (motor != NULL) {
        (void)rs_motor_disable(motor);
    }
}

float RsActuator_GetTargetDeg(RsActuatorId actuator)
{
    if (RsActuator_IsValid(actuator) == 0U) {
        return 0.0f;
    }
    return s_actuator_state[actuator].target_deg;
}

float RsActuator_GetPositionDeg(RsActuatorId actuator)
{
    RsMotor *motor = RsActuator_GetMotor(actuator);

    if (motor == NULL) {
        return 0.0f;
    }
    return motor->feedback.angle_deg * (float)s_actuator_config[actuator].direction;
}

uint8_t RsActuator_IsOnline(RsActuatorId actuator, uint32_t now_ms)
{
    RsMotor *motor = RsActuator_GetMotor(actuator);

    if (motor == NULL) {
        return 0U;
    }
    return (motor->state.online != 0U &&
            (uint32_t)(now_ms - motor->state.last_update_ms) <=
                RS_ACTUATOR_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}
