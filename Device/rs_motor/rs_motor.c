#include "rs_motor.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define RS_COMM_TYPE_MOTION_CONTROL   0x01U
#define RS_COMM_TYPE_FEEDBACK         0x02U
#define RS_COMM_TYPE_ENABLE           0x03U
#define RS_COMM_TYPE_DISABLE          0x04U
#define RS_COMM_TYPE_PARAMETER_WRITE  0x12U

#define RS_PARAM_CONTROL_MODE         0x7005U
#define RS_PARAM_CURRENT_TARGET       0x7006U
#define RS_PARAM_SPEED_TARGET         0x700AU
#define RS_PARAM_POSITION_TARGET      0x7016U
#define RS_PARAM_CSP_SPEED_LIMIT      0x7017U
#define RS_PARAM_CURRENT_LIMIT        0x7018U
#define RS_PARAM_SPEED_ACCELERATION   0x7022U
#define RS_PARAM_PP_SPEED             0x7024U
#define RS_PARAM_PP_ACCELERATION      0x7025U

#define RS_EXT_ID_MAX                 0x1FFFFFFFU
#define RS_EXT_ID_TYPE_SHIFT          24U
#define RS_EXT_ID_DATA_SHIFT          8U
#define RS_EXT_ID_TYPE_MASK           0x1FU
#define RS_EXT_ID_BYTE_MASK           0xFFU
#define RS_DEFAULT_PP_POSITION_LIMIT_RAD 12.57f

typedef struct {
    float speed_min;
    float speed_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float torque_min;
    float torque_max;
} rs_motor_range_t;

static const rs_motor_range_t s_ranges[RS_MOTOR_TYPE_COUNT] = {
    {-33.0f, 33.0f, 0.0f, 500.0f, 0.0f, 5.0f, -14.0f, 14.0f},
    {-44.0f, 44.0f, 0.0f, 500.0f, 0.0f, 5.0f, -17.0f, 17.0f},
    {-44.0f, 44.0f, 0.0f, 500.0f, 0.0f, 5.0f, -17.0f, 17.0f},
    {-20.0f, 20.0f, 0.0f, 5000.0f, 0.0f, 100.0f, -60.0f, 60.0f},
    {-15.0f, 15.0f, 0.0f, 5000.0f, 0.0f, 100.0f, -120.0f, 120.0f},
    {-50.0f, 50.0f, 0.0f, 500.0f, 0.0f, 5.0f, -5.5f, 5.5f},
    {-50.0f, 50.0f, 0.0f, 5000.0f, 0.0f, 100.0f, -36.0f, 36.0f},
};

static rs_motor_t *s_motor_list;

_Static_assert(sizeof(float) == 4U, "RobStride parameter encoding requires 32-bit float");

static uint8_t rs_motor_is_finite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static float rs_motor_clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static uint16_t rs_motor_float_to_u16(float value, float minimum, float maximum)
{
    float clamped = rs_motor_clamp(value, minimum, maximum);

    if (clamped <= minimum) {
        return 0U;
    }
    if (clamped >= maximum) {
        return UINT16_MAX;
    }
    return (uint16_t)((clamped - minimum) * 65535.0f / (maximum - minimum));
}

static float rs_motor_u16_to_float(uint16_t value, float minimum, float maximum)
{
    return ((float)value / 65535.0f) * (maximum - minimum) + minimum;
}

static void rs_motor_put_be_u16(uint8_t data[2], uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

static uint16_t rs_motor_get_be_u16(const uint8_t data[2])
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

static void rs_motor_put_le_float(uint8_t data[4], float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8U);
    data[2] = (uint8_t)(raw >> 16U);
    data[3] = (uint8_t)(raw >> 24U);
}

static uint32_t rs_motor_make_id(uint8_t communication_type,
                                 uint16_t communication_data,
                                 uint8_t target_id)
{
    return ((uint32_t)(communication_type & RS_EXT_ID_TYPE_MASK) << RS_EXT_ID_TYPE_SHIFT) |
           ((uint32_t)communication_data << RS_EXT_ID_DATA_SHIFT) |
           (uint32_t)target_id;
}

static rs_motor_t *rs_motor_find_pointer(const rs_motor_t *motor)
{
    rs_motor_t *current = s_motor_list;

    while (current != NULL) {
        if (current == motor) {
            return current;
        }
        current = current->internal.next;
    }
    return NULL;
}

static uint8_t rs_motor_config_is_valid(const rs_motor_config_t *config)
{
    if (config->hfdcan == NULL || config->motor_id == 0U || config->motor_id > 0x7FU) {
        return 0U;
    }
    if ((uint32_t)config->motor_type >= (uint32_t)RS_MOTOR_TYPE_COUNT) {
        return 0U;
    }
    if (config->offline_timeout_ms == 0U) {
        return 0U;
    }
    if (rs_motor_is_finite(config->pp_position_limit_rad) == 0U ||
        config->pp_position_limit_rad < 0.0f ||
        config->pp_position_limit_rad > 100000.0f) {
        return 0U;
    }
    return 1U;
}

static rs_motor_status_t rs_motor_require_initialized(rs_motor_t *motor)
{
    if (motor == NULL) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (motor->internal.initialized == 0U || rs_motor_find_pointer(motor) == NULL) {
        return RS_MOTOR_STATUS_NOT_INITIALIZED;
    }
    return RS_MOTOR_STATUS_OK;
}

static rs_motor_status_t rs_motor_send(rs_motor_t *motor,
                                       uint8_t communication_type,
                                       uint16_t communication_data,
                                       const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};

    header.Identifier = rs_motor_make_id(communication_type,
                                         communication_data,
                                         motor->config.motor_id);
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(motor->config.hfdcan,
                                      &header,
                                      (uint8_t *)data) != HAL_OK) {
        return RS_MOTOR_STATUS_FDCAN_TX_ERROR;
    }
    return RS_MOTOR_STATUS_OK;
}

static rs_motor_status_t rs_motor_send_enable(rs_motor_t *motor)
{
    static const uint8_t data[8] = {0};

    return rs_motor_send(motor, RS_COMM_TYPE_ENABLE, motor->config.master_id, data);
}

static rs_motor_status_t rs_motor_send_disable(rs_motor_t *motor)
{
    static const uint8_t data[8] = {0};

    return rs_motor_send(motor, RS_COMM_TYPE_DISABLE, motor->config.master_id, data);
}

static rs_motor_status_t rs_motor_write_mode(rs_motor_t *motor,
                                             rs_motor_control_mode_t mode)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)RS_PARAM_CONTROL_MODE;
    data[1] = (uint8_t)(RS_PARAM_CONTROL_MODE >> 8U);
    data[4] = (uint8_t)mode;
    return rs_motor_send(motor,
                         RS_COMM_TYPE_PARAMETER_WRITE,
                         motor->config.master_id,
                         data);
}

static rs_motor_status_t rs_motor_write_float(rs_motor_t *motor,
                                              uint16_t index,
                                              float value)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8U);
    rs_motor_put_le_float(&data[4], value);
    return rs_motor_send(motor,
                         RS_COMM_TYPE_PARAMETER_WRITE,
                         motor->config.master_id,
                         data);
}

static rs_motor_status_t rs_motor_apply_mode(rs_motor_t *motor,
                                             rs_motor_control_mode_t mode)
{
    rs_motor_status_t status;
    uint8_t mode_changed = (motor->internal.mode_applied == 0U ||
                            motor->internal.applied_control_mode != mode) ? 1U : 0U;

    if (mode_changed != 0U) {
        if (motor->state.enabled != 0U) {
            status = rs_motor_send_disable(motor);
            if (status != RS_MOTOR_STATUS_OK) {
                return status;
            }
            motor->state.enabled = 0U;
        }

        status = rs_motor_write_mode(motor, mode);
        if (status != RS_MOTOR_STATUS_OK) {
            return status;
        }

        status = rs_motor_send_enable(motor);
        if (status != RS_MOTOR_STATUS_OK) {
            return status;
        }

        motor->state.enabled = 1U;
        motor->state.control_mode = mode;
        motor->internal.applied_control_mode = mode;
        motor->internal.mode_applied = 1U;
        return RS_MOTOR_STATUS_OK;
    }

    if (motor->state.enabled == 0U) {
        status = rs_motor_send_enable(motor);
        if (status != RS_MOTOR_STATUS_OK) {
            return status;
        }
        motor->state.enabled = 1U;
    }
    return RS_MOTOR_STATUS_OK;
}

rs_motor_status_t rs_motor_init(rs_motor_t *motor)
{
    rs_motor_t *current;
    const rs_motor_range_t *range;

    if (motor == NULL) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    /* 不清除已注册对象，否则会破坏实例链表。 */
    if (rs_motor_find_pointer(motor) != NULL) {
        return RS_MOTOR_STATUS_DUPLICATE_INSTANCE;
    }

    memset(&motor->feedback, 0, sizeof(motor->feedback));
    memset(&motor->state, 0, sizeof(motor->state));
    memset(&motor->internal, 0, sizeof(motor->internal));
    motor->state.control_mode = RS_MOTOR_CONTROL_MODE_NONE;
    motor->internal.applied_control_mode = RS_MOTOR_CONTROL_MODE_NONE;

    if (rs_motor_config_is_valid(&motor->config) == 0U) {
        return RS_MOTOR_STATUS_INVALID_CONFIG;
    }

    current = s_motor_list;
    while (current != NULL) {
        if (current->config.hfdcan == motor->config.hfdcan &&
            current->config.master_id == motor->config.master_id &&
            current->config.motor_id == motor->config.motor_id) {
            return RS_MOTOR_STATUS_DUPLICATE_INSTANCE;
        }
        current = current->internal.next;
    }

    range = &s_ranges[motor->config.motor_type];
    motor->internal.position_min = RS_MOTOR_FEEDBACK_POSITION_MIN_RAD;
    motor->internal.position_max = RS_MOTOR_FEEDBACK_POSITION_MAX_RAD;
    motor->internal.feedback_position_min = RS_MOTOR_FEEDBACK_POSITION_MIN_RAD;
    motor->internal.feedback_position_max = RS_MOTOR_FEEDBACK_POSITION_MAX_RAD;
    if (motor->config.pp_position_limit_rad > 0.0f) {
        motor->internal.pp_position_min = -motor->config.pp_position_limit_rad;
        motor->internal.pp_position_max = motor->config.pp_position_limit_rad;
    } else {
        motor->internal.pp_position_min = -RS_DEFAULT_PP_POSITION_LIMIT_RAD;
        motor->internal.pp_position_max = RS_DEFAULT_PP_POSITION_LIMIT_RAD;
    }
    motor->internal.speed_min = range->speed_min;
    motor->internal.speed_max = range->speed_max;
    motor->internal.kp_min = range->kp_min;
    motor->internal.kp_max = range->kp_max;
    motor->internal.kd_min = range->kd_min;
    motor->internal.kd_max = range->kd_max;
    motor->internal.torque_min = range->torque_min;
    motor->internal.torque_max = range->torque_max;
    motor->internal.initialized = 1U;

    motor->internal.next = s_motor_list;
    s_motor_list = motor;
    return RS_MOTOR_STATUS_OK;
}

rs_motor_status_t rs_motor_deinit(rs_motor_t *motor)
{
    rs_motor_t **link;

    if (motor == NULL) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    link = &s_motor_list;
    while (*link != NULL && *link != motor) {
        link = &(*link)->internal.next;
    }
    if (*link == NULL || motor->internal.initialized == 0U) {
        return RS_MOTOR_STATUS_NOT_INITIALIZED;
    }

    *link = motor->internal.next;
    memset(&motor->feedback, 0, sizeof(motor->feedback));
    memset(&motor->state, 0, sizeof(motor->state));
    memset(&motor->internal, 0, sizeof(motor->internal));
    motor->state.control_mode = RS_MOTOR_CONTROL_MODE_NONE;
    motor->internal.applied_control_mode = RS_MOTOR_CONTROL_MODE_NONE;
    return RS_MOTOR_STATUS_OK;
}

rs_motor_status_t rs_motor_enable(rs_motor_t *motor)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    status = rs_motor_send_enable(motor);
    if (status == RS_MOTOR_STATUS_OK) {
        motor->state.enabled = 1U;
    }
    return status;
}

rs_motor_status_t rs_motor_disable(rs_motor_t *motor)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    status = rs_motor_send_disable(motor);
    if (status == RS_MOTOR_STATUS_OK) {
        motor->state.enabled = 0U;
    }
    return status;
}

rs_motor_status_t rs_motor_motion_control(rs_motor_t *motor,
                                          float torque_nm,
                                          float position_rad,
                                          float speed_rad_s,
                                          float kp,
                                          float kd)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);
    uint8_t data[8];
    uint16_t torque_raw;

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    if (rs_motor_is_finite(torque_nm) == 0U ||
        rs_motor_is_finite(position_rad) == 0U ||
        rs_motor_is_finite(speed_rad_s) == 0U ||
        rs_motor_is_finite(kp) == 0U ||
        rs_motor_is_finite(kd) == 0U) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    torque_raw = rs_motor_float_to_u16(torque_nm,
                                       motor->internal.torque_min,
                                       motor->internal.torque_max);
    rs_motor_put_be_u16(&data[0],
                        rs_motor_float_to_u16(position_rad,
                                              motor->internal.position_min,
                                              motor->internal.position_max));
    rs_motor_put_be_u16(&data[2],
                        rs_motor_float_to_u16(speed_rad_s,
                                              motor->internal.speed_min,
                                              motor->internal.speed_max));
    rs_motor_put_be_u16(&data[4],
                        rs_motor_float_to_u16(kp,
                                              motor->internal.kp_min,
                                              motor->internal.kp_max));
    rs_motor_put_be_u16(&data[6],
                        rs_motor_float_to_u16(kd,
                                              motor->internal.kd_min,
                                              motor->internal.kd_max));

    status = rs_motor_apply_mode(motor, RS_MOTOR_CONTROL_MODE_MOTION);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    return rs_motor_send(motor, RS_COMM_TYPE_MOTION_CONTROL, torque_raw, data);
}

rs_motor_status_t rs_motor_pp_position_control(rs_motor_t *motor,
                                               float speed_rad_s,
                                               float acceleration_rad_s2,
                                               float position_rad)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    if (rs_motor_is_finite(speed_rad_s) == 0U ||
        rs_motor_is_finite(acceleration_rad_s2) == 0U ||
        rs_motor_is_finite(position_rad) == 0U) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    speed_rad_s = rs_motor_clamp(speed_rad_s,
                                 motor->internal.speed_min,
                                 motor->internal.speed_max);
    position_rad = rs_motor_clamp(position_rad,
                                  motor->internal.pp_position_min,
                                  motor->internal.pp_position_max);

    status = rs_motor_apply_mode(motor, RS_MOTOR_CONTROL_MODE_PP_POSITION);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    status = rs_motor_write_float(motor, RS_PARAM_PP_SPEED, speed_rad_s);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    status = rs_motor_write_float(motor, RS_PARAM_PP_ACCELERATION, acceleration_rad_s2);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    return rs_motor_write_float(motor, RS_PARAM_POSITION_TARGET, position_rad);
}

rs_motor_status_t rs_motor_csp_position_control(rs_motor_t *motor,
                                                float speed_limit_rad_s,
                                                float position_rad)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    if (rs_motor_is_finite(speed_limit_rad_s) == 0U ||
        rs_motor_is_finite(position_rad) == 0U) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    speed_limit_rad_s = rs_motor_clamp(speed_limit_rad_s,
                                       motor->internal.speed_min,
                                       motor->internal.speed_max);
    position_rad = rs_motor_clamp(position_rad,
                                  motor->internal.position_min,
                                  motor->internal.position_max);

    status = rs_motor_apply_mode(motor, RS_MOTOR_CONTROL_MODE_CSP_POSITION);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    status = rs_motor_write_float(motor, RS_PARAM_CSP_SPEED_LIMIT, speed_limit_rad_s);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    return rs_motor_write_float(motor, RS_PARAM_POSITION_TARGET, position_rad);
}

rs_motor_status_t rs_motor_speed_control(rs_motor_t *motor,
                                         float current_limit_a,
                                         float acceleration_rad_s2,
                                         float speed_rad_s)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    if (rs_motor_is_finite(current_limit_a) == 0U ||
        rs_motor_is_finite(acceleration_rad_s2) == 0U ||
        rs_motor_is_finite(speed_rad_s) == 0U) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    speed_rad_s = rs_motor_clamp(speed_rad_s,
                                 motor->internal.speed_min,
                                 motor->internal.speed_max);

    status = rs_motor_apply_mode(motor, RS_MOTOR_CONTROL_MODE_SPEED);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    status = rs_motor_write_float(motor, RS_PARAM_CURRENT_LIMIT, current_limit_a);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    status = rs_motor_write_float(motor, RS_PARAM_SPEED_ACCELERATION, acceleration_rad_s2);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    return rs_motor_write_float(motor, RS_PARAM_SPEED_TARGET, speed_rad_s);
}

rs_motor_status_t rs_motor_current_control(rs_motor_t *motor, float current_a)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    if (rs_motor_is_finite(current_a) == 0U) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    status = rs_motor_apply_mode(motor, RS_MOTOR_CONTROL_MODE_CURRENT);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    return rs_motor_write_float(motor, RS_PARAM_CURRENT_TARGET, current_a);
}

uint8_t rs_motor_handle_rx(FDCAN_HandleTypeDef *hfdcan,
                           const FDCAN_RxHeaderTypeDef *header,
                           const uint8_t data[8],
                           uint32_t now_ms)
{
    uint32_t identifier;
    uint16_t communication_data;
    uint8_t communication_type;
    uint8_t master_id;
    uint8_t motor_id;
    uint8_t run_state;
    rs_motor_t *motor;

    if (hfdcan == NULL || header == NULL || data == NULL) {
        return 0U;
    }
    if (header->IdType != FDCAN_EXTENDED_ID ||
        header->RxFrameType != FDCAN_DATA_FRAME ||
        header->DataLength != FDCAN_DLC_BYTES_8 ||
        header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->Identifier > RS_EXT_ID_MAX) {
        return 0U;
    }

    identifier = header->Identifier;
    communication_type = (uint8_t)((identifier >> RS_EXT_ID_TYPE_SHIFT) &
                                   RS_EXT_ID_TYPE_MASK);
    if (communication_type != RS_COMM_TYPE_FEEDBACK) {
        return 0U;
    }

    master_id = (uint8_t)(identifier & RS_EXT_ID_BYTE_MASK);
    communication_data = (uint16_t)(identifier >> RS_EXT_ID_DATA_SHIFT);
    motor_id = (uint8_t)(communication_data & RS_EXT_ID_BYTE_MASK);

    motor = s_motor_list;
    while (motor != NULL) {
        if (motor->config.hfdcan == hfdcan &&
            motor->config.master_id == master_id &&
            motor->config.motor_id == motor_id) {
            break;
        }
        motor = motor->internal.next;
    }
    if (motor == NULL) {
        return 0U;
    }

    motor->feedback.fault.under_voltage = (uint8_t)((communication_data >> 8U) & 1U);
    motor->feedback.fault.drive = (uint8_t)((communication_data >> 9U) & 1U);
    motor->feedback.fault.over_temperature = (uint8_t)((communication_data >> 10U) & 1U);
    motor->feedback.fault.magnetic_encoder = (uint8_t)((communication_data >> 11U) & 1U);
    motor->feedback.fault.stall_overload = (uint8_t)((communication_data >> 12U) & 1U);
    motor->feedback.fault.uncalibrated = (uint8_t)((communication_data >> 13U) & 1U);

    run_state = (uint8_t)((communication_data >> 14U) & 0x03U);
    if (run_state > (uint8_t)RS_MOTOR_RUN_STATE_RUNNING) {
        run_state = (uint8_t)RS_MOTOR_RUN_STATE_REST;
    }
    motor->feedback.run_state = (rs_motor_run_state_t)run_state;

    motor->feedback.angle_rad = rs_motor_u16_to_float(rs_motor_get_be_u16(&data[0]),
                                                       motor->internal.feedback_position_min,
                                                       motor->internal.feedback_position_max);
    motor->feedback.speed_rad_s = rs_motor_u16_to_float(rs_motor_get_be_u16(&data[2]),
                                                         motor->internal.speed_min,
                                                         motor->internal.speed_max);
    motor->feedback.torque_nm = rs_motor_u16_to_float(rs_motor_get_be_u16(&data[4]),
                                                       motor->internal.torque_min,
                                                       motor->internal.torque_max);
    motor->feedback.temperature_c = (float)rs_motor_get_be_u16(&data[6]) / 10.0f;

    motor->state.online = 1U;
    motor->state.feedback_count++;
    motor->state.last_update_ms = now_ms;
    return 1U;
}

rs_motor_status_t rs_motor_update(rs_motor_t *motor, uint32_t now_ms)
{
    rs_motor_status_t status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    if (motor->state.feedback_count != 0U &&
        (uint32_t)(now_ms - motor->state.last_update_ms) >=
            motor->config.offline_timeout_ms) {
        motor->state.online = 0U;
    }
    return RS_MOTOR_STATUS_OK;
}
