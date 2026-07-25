#include "dji_motor.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#include "main.h"

static uint8_t dji_motor_is_finite_float(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static uint8_t dji_motor_is_finite_double(double value)
{
    return (value >= -DBL_MAX && value <= DBL_MAX) ? 1U : 0U;
}

static double dji_motor_clamp_double(double value,
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

static uint8_t dji_motor_pid_config_is_valid(
    const dji_motor_pid_config_t *config)
{
    return (dji_motor_is_finite_float(config->kp) != 0U &&
            dji_motor_is_finite_float(config->ki) != 0U &&
            dji_motor_is_finite_float(config->kd) != 0U &&
            dji_motor_is_finite_float(config->integral_limit) != 0U &&
            config->integral_limit >= 0.0f) ? 1U : 0U;
}

static uint8_t dji_motor_config_is_valid(const dji_motor_config_t *config)
{
    if (config->motor_id == 0U || config->motor_id > 8U ||
        config->control_period_ms == 0U ||
        config->offline_timeout_ms <= config->control_period_ms) {
        return 0U;
    }
    if (dji_motor_is_finite_float(config->current_limit) == 0U ||
        config->current_limit <= 0.0f ||
        config->current_limit > DJI_MOTOR_CURRENT_MAX ||
        dji_motor_is_finite_float(config->max_speed_rpm) == 0U ||
        config->max_speed_rpm < 0.0f) {
        return 0U;
    }
    if (dji_motor_pid_config_is_valid(&config->speed_pid) == 0U ||
        dji_motor_pid_config_is_valid(&config->position_pid) == 0U) {
        return 0U;
    }
    return 1U;
}

static dji_motor_status_t dji_motor_require_initialized(
    const dji_motor_t *motor)
{
    if (motor == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (motor->internal.initialized == 0U) {
        return DJI_MOTOR_STATUS_NOT_INITIALIZED;
    }
    return DJI_MOTOR_STATUS_OK;
}

static void dji_motor_reset_pid(dji_motor_pid_state_t *pid)
{
    pid->integral = 0.0;
    pid->last_error = 0.0;
}

static void dji_motor_reset_all_pid(dji_motor_t *motor)
{
    dji_motor_reset_pid(&motor->internal.position_pid);
    dji_motor_reset_pid(&motor->internal.speed_pid);
}

static void dji_motor_apply_mode(dji_motor_t *motor,
                                 dji_motor_control_mode_t mode)
{
    if (motor->state.control_mode != mode) {
        dji_motor_reset_all_pid(motor);
        motor->internal.control_timer_started = 0U;
    }
    motor->state.control_mode = mode;
    motor->state.enabled = 1U;
}

static double dji_motor_pid_update(
    const dji_motor_pid_config_t *config,
    dji_motor_pid_state_t *state,
    double error,
    double dt_s)
{
    double integral_limit = (double)config->integral_limit;
    double derivative;

    state->integral += error * dt_s;
    state->integral = dji_motor_clamp_double(state->integral,
                                             -integral_limit,
                                             integral_limit);
    derivative = (error - state->last_error) / dt_s;
    state->last_error = error;

    return (double)config->kp * error +
           (double)config->ki * state->integral +
           (double)config->kd * derivative;
}

static int16_t dji_motor_float_to_current(float current, float limit)
{
    double clamped = dji_motor_clamp_double((double)current,
                                            -(double)limit,
                                            (double)limit);

    if (clamped >= 0.0) {
        return (int16_t)(clamped + 0.5);
    }
    return (int16_t)(clamped - 0.5);
}

static float dji_motor_run_speed_pid(dji_motor_t *motor,
                                     float target_speed_rpm,
                                     int16_t feedback_speed_rpm,
                                     double dt_s)
{
    double speed_error = (double)target_speed_rpm -
                         (double)feedback_speed_rpm;

    return (float)dji_motor_pid_update(&motor->config.speed_pid,
                                       &motor->internal.speed_pid,
                                       speed_error,
                                       dt_s);
}

dji_motor_status_t dji_motor_init(dji_motor_t *motor)
{
    if (motor == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (motor->internal.initialized != 0U) {
        return DJI_MOTOR_STATUS_DUPLICATE_INSTANCE;
    }
    if (dji_motor_config_is_valid(&motor->config) == 0U) {
        return DJI_MOTOR_STATUS_INVALID_CONFIG;
    }

    memset(&motor->feedback, 0, sizeof(motor->feedback));
    memset(&motor->state, 0, sizeof(motor->state));
    memset(&motor->internal, 0, sizeof(motor->internal));
    motor->state.control_mode = DJI_MOTOR_CONTROL_MODE_NONE;
    motor->internal.initialized = 1U;
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_deinit(dji_motor_t *motor)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (motor->internal.group != NULL) {
        return DJI_MOTOR_STATUS_ALREADY_GROUPED;
    }

    memset(&motor->feedback, 0, sizeof(motor->feedback));
    memset(&motor->state, 0, sizeof(motor->state));
    memset(&motor->internal, 0, sizeof(motor->internal));
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_disable(dji_motor_t *motor)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    motor->state.enabled = 0U;
    motor->state.control_mode = DJI_MOTOR_CONTROL_MODE_NONE;
    motor->state.target_current = 0.0f;
    motor->state.speed_correction_rpm = 0.0f;
    motor->state.current_correction = 0.0f;
    motor->internal.current_command = 0;
    motor->internal.control_timer_started = 0U;
    dji_motor_reset_all_pid(motor);
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_set_speed_correction(
    dji_motor_t *motor,
    float speed_correction_rpm)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (dji_motor_is_finite_float(speed_correction_rpm) == 0U) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    motor->state.speed_correction_rpm = speed_correction_rpm;
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_set_current_correction(
    dji_motor_t *motor,
    float current_correction)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (dji_motor_is_finite_float(current_correction) == 0U) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    motor->state.current_correction = current_correction;
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_current_control(dji_motor_t *motor,
                                             float current)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (dji_motor_is_finite_float(current) == 0U) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    dji_motor_apply_mode(motor, DJI_MOTOR_CONTROL_MODE_CURRENT);
    motor->state.target_current = current;
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_speed_control(dji_motor_t *motor,
                                           float speed_rpm)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (dji_motor_is_finite_float(speed_rpm) == 0U) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    dji_motor_apply_mode(motor, DJI_MOTOR_CONTROL_MODE_SPEED);
    motor->state.target_speed_rpm = speed_rpm;
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_position_control(dji_motor_t *motor,
                                              double position_deg)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (dji_motor_is_finite_double(position_deg) == 0U) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (motor->config.max_speed_rpm <= 0.0f) {
        return DJI_MOTOR_STATUS_INVALID_CONFIG;
    }

    dji_motor_apply_mode(motor, DJI_MOTOR_CONTROL_MODE_POSITION);
    motor->state.target_position_deg = position_deg;
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_set_zero_to_current(dji_motor_t *motor)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);
    uint32_t interrupt_mask;

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (motor->feedback.multi_turn.valid == 0U) {
        return DJI_MOTOR_STATUS_NOT_INITIALIZED;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    motor->internal.previous_encoder = motor->feedback.encoder;
    motor->feedback.multi_turn.encoder_count = 0;
    motor->feedback.multi_turn.angle_deg = 0.0;
    __DMB();
    if (interrupt_mask == 0U) {
        __enable_irq();
    }

    motor->state.target_position_deg = 0.0;
    motor->state.position_error_deg = 0.0;
    motor->internal.control_timer_started = 0U;
    dji_motor_reset_all_pid(motor);
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_get_feedback(const dji_motor_t *motor,
                                          dji_motor_feedback_t *feedback)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);
    uint32_t interrupt_mask;

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (feedback == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    *feedback = motor->feedback;
    __DMB();
    if (interrupt_mask == 0U) {
        __enable_irq();
    }
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_update(dji_motor_t *motor, uint32_t now_ms)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);
    uint32_t elapsed_ms;
    double dt_s;
    float current = 0.0f;
    dji_motor_feedback_t feedback;

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }

    if (motor->state.feedback_count == 0U ||
        (uint32_t)(now_ms - motor->state.last_update_ms) >=
            motor->config.offline_timeout_ms) {
        if (motor->state.online != 0U) {
            motor->state.online = 0U;
            dji_motor_reset_all_pid(motor);
            motor->internal.control_timer_started = 0U;
        }
        motor->internal.current_command = 0;
        return DJI_MOTOR_STATUS_OK;
    }

    if (motor->state.enabled == 0U ||
        motor->state.control_mode == DJI_MOTOR_CONTROL_MODE_NONE) {
        motor->internal.current_command = 0;
        return DJI_MOTOR_STATUS_OK;
    }

    if (motor->internal.control_timer_started == 0U) {
        elapsed_ms = motor->config.control_period_ms;
        motor->internal.control_timer_started = 1U;
    } else {
        elapsed_ms = (uint32_t)(now_ms - motor->internal.last_control_ms);
        if (elapsed_ms < motor->config.control_period_ms) {
            return DJI_MOTOR_STATUS_OK;
        }
    }
    motor->internal.last_control_ms = now_ms;
    dt_s = (double)elapsed_ms * 0.001;

    if (motor->state.control_mode == DJI_MOTOR_CONTROL_MODE_CURRENT) {
        current = motor->state.target_current;
    } else {
        status = dji_motor_get_feedback(motor, &feedback);
        if (status != DJI_MOTOR_STATUS_OK) {
            motor->internal.current_command = 0;
            return status;
        }
    }

    if (motor->state.control_mode == DJI_MOTOR_CONTROL_MODE_SPEED) {
        current = dji_motor_run_speed_pid(motor,
                                          motor->state.target_speed_rpm,
                                          feedback.speed_rpm,
                                          dt_s) +
                  motor->state.current_correction;
    } else if (motor->state.control_mode == DJI_MOTOR_CONTROL_MODE_POSITION) {
        double target_speed_rpm;

        if (feedback.multi_turn.valid == 0U) {
            motor->internal.current_command = 0;
            return DJI_MOTOR_STATUS_OK;
        }
        motor->state.position_error_deg =
            motor->state.target_position_deg -
            feedback.multi_turn.angle_deg;
        target_speed_rpm = dji_motor_pid_update(
            &motor->config.position_pid,
            &motor->internal.position_pid,
            motor->state.position_error_deg,
            dt_s);
        target_speed_rpm += (double)motor->state.speed_correction_rpm;
        target_speed_rpm = dji_motor_clamp_double(
            target_speed_rpm,
            -(double)motor->config.max_speed_rpm,
            (double)motor->config.max_speed_rpm);
        motor->state.target_speed_rpm = (float)target_speed_rpm;
        current = dji_motor_run_speed_pid(motor,
                                          motor->state.target_speed_rpm,
                                          feedback.speed_rpm,
                                          dt_s) +
                  motor->state.current_correction;
    }

    motor->internal.current_command =
        dji_motor_float_to_current(current, motor->config.current_limit);
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_handle_feedback(dji_motor_t *motor,
                                             const uint8_t data[8],
                                             uint32_t now_ms)
{
    dji_motor_status_t status = dji_motor_require_initialized(motor);
    uint16_t encoder;

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (data == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    encoder = (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
    if (encoder >= DJI_MOTOR_ENCODER_RANGE) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    if (motor->feedback.multi_turn.valid == 0U) {
        motor->internal.previous_encoder = encoder;
        motor->feedback.multi_turn.encoder_count = 0;
        motor->feedback.multi_turn.angle_deg = 0.0;
        motor->feedback.multi_turn.valid = 1U;
    } else if (motor->state.online == 0U) {
        /*
         * 离线期间转过的圈数无法由单圈编码器恢复。重新上线时保留最后已知
         * 连续位置并重新锚定，避免把未知位移误判为一次跨零。
         */
        motor->internal.previous_encoder = encoder;
    } else {
        int32_t delta = (int32_t)encoder -
                        (int32_t)motor->internal.previous_encoder;

        if (delta > DJI_MOTOR_ENCODER_HALF) {
            delta -= DJI_MOTOR_ENCODER_RANGE;
        } else if (delta < -DJI_MOTOR_ENCODER_HALF) {
            delta += DJI_MOTOR_ENCODER_RANGE;
        }
        motor->feedback.multi_turn.encoder_count += (int64_t)delta;
        motor->feedback.multi_turn.angle_deg =
            (double)motor->feedback.multi_turn.encoder_count * 360.0 /
            (double)DJI_MOTOR_ENCODER_RANGE;
        motor->internal.previous_encoder = encoder;
    }

    motor->feedback.encoder = encoder;
    motor->feedback.speed_rpm =
        (int16_t)(((uint16_t)data[2] << 8U) | (uint16_t)data[3]);
    motor->feedback.given_current =
        (int16_t)(((uint16_t)data[4] << 8U) | (uint16_t)data[5]);
    motor->feedback.temperature_c = data[6];
    motor->state.online = 1U;
    motor->state.feedback_count++;
    motor->state.last_update_ms = now_ms;
    return DJI_MOTOR_STATUS_OK;
}
