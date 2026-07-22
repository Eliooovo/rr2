#include "rs_motor.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#include "bsp_fdcan.h"

#define RS_COMM_TYPE_MOTION_CONTROL   0x01U
#define RS_COMM_TYPE_FEEDBACK         0x02U
#define RS_COMM_TYPE_ENABLE           0x03U
#define RS_COMM_TYPE_DISABLE          0x04U
#define RS_COMM_TYPE_PARAMETER_WRITE  0x12U
#define RS_COMM_TYPE_PROACTIVE_REPORT 0x18U

#define RS_PARAM_CONTROL_MODE         0x7005U
#define RS_PARAM_POSITION_TARGET      0x7016U
#define RS_PARAM_CSP_SPEED_LIMIT      0x7017U

#define RS_EXT_ID_MAX                 0x1FFFFFFFU
#define RS_EXT_ID_TYPE_SHIFT          24U
#define RS_EXT_ID_DATA_SHIFT          8U
#define RS_EXT_ID_TYPE_MASK           0x1FU
#define RS_EXT_ID_BYTE_MASK           0xFFU

#define RS_STANDARD_MODE_POSITION     0x1U

RsMotorState g_rs_motors[RS_MOTOR_COUNT];

static RsMotor *s_motor_list;
static uint8_t s_host_id = RS_MOTOR_DEFAULT_HOST_ID;

_Static_assert(sizeof(float) == 4U, "RobStride private protocol requires 32-bit float");

/* ==========================================================================
 * Model parameters
 * ========================================================================== */

static const RsMotorModelParams s_rs00_params = {
    RS_MOTOR_MODEL_RS00,
    -12.57f, 12.57f,
    -33.0f, 33.0f,
    -14.0f, 14.0f
};

/* RS03 is kept as a configured entry, but its exact range should be checked
 * against the RS03 private-protocol manual before hardware validation. */
static const RsMotorModelParams s_rs03_placeholder_params = {
    RS_MOTOR_MODEL_RS03,
    -12.57f, 12.57f,
    -20.0f, 20.0f,
    -60.0f, 60.0f
};

static const RsMotorModelParams s_rs05_params = {
    RS_MOTOR_MODEL_RS05,
    -12.57f, 12.57f,
    -50.0f, 50.0f,
    -5.5f, 5.5f
};

static uint8_t RsMotor_IsValidMotorId(uint8_t motor_id)
{
    return (motor_id >= 1U && motor_id <= RS_MOTOR_COUNT) ? 1U : 0U;
}

static const RsMotorModelParams *RsMotor_GetDefaultParams(RsMotorModel model)
{
    if (model == RS_MOTOR_MODEL_RS00) {
        return &s_rs00_params;
    }
    if (model == RS_MOTOR_MODEL_RS03) {
        return &s_rs03_placeholder_params;
    }
    if (model == RS_MOTOR_MODEL_RS05) {
        return &s_rs05_params;
    }
    return NULL;
}

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

static float rs_motor_u16_to_float(uint16_t value, float minimum, float maximum)
{
    return ((float)value / 65535.0f) * (maximum - minimum) + minimum;
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

static void rs_motor_fill(uint8_t data[8], uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        data[i] = value;
    }
}

static uint32_t rs_motor_make_ext_id(uint8_t communication_type,
                                     uint16_t communication_data,
                                     uint8_t target_id)
{
    return ((uint32_t)(communication_type & RS_EXT_ID_TYPE_MASK) << RS_EXT_ID_TYPE_SHIFT) |
           ((uint32_t)communication_data << RS_EXT_ID_DATA_SHIFT) |
           (uint32_t)target_id;
}

static RsMotor *rs_motor_find_pointer(const RsMotor *motor)
{
    RsMotor *current = s_motor_list;

    while (current != NULL) {
        if (current == motor) {
            return current;
        }
        current = current->internal.next;
    }
    return NULL;
}

static RsMotor *rs_motor_find_by_id(FDCAN_HandleTypeDef *hfdcan,
                                    uint8_t host_id,
                                    uint8_t motor_id)
{
    RsMotor *current = s_motor_list;

    while (current != NULL) {
        if (current->config.hfdcan == hfdcan &&
            current->config.host_id == host_id &&
            current->config.motor_id == motor_id) {
            return current;
        }
        current = current->internal.next;
    }
    return NULL;
}

static uint8_t rs_motor_config_is_valid(const RsMotorConfig *config)
{
    if (config == NULL ||
        config->hfdcan == NULL ||
        config->motor_id == 0U ||
        config->motor_id > 0x7FU ||
        config->host_id == 0U ||
        config->protocol != RS_MOTOR_PROTOCOL_PRIVATE ||
        config->offline_timeout_ms == 0U) {
        return 0U;
    }
    if (config->params.position_max_rad <= config->params.position_min_rad ||
        config->params.speed_max_rad_s <= config->params.speed_min_rad_s ||
        config->params.torque_max_nm <= config->params.torque_min_nm) {
        return 0U;
    }
    return 1U;
}

static RsMotorStatus rs_motor_require_initialized(RsMotor *motor)
{
    if (motor == NULL) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (motor->internal.initialized == 0U || rs_motor_find_pointer(motor) == NULL) {
        return RS_MOTOR_STATUS_NOT_INITIALIZED;
    }
    return RS_MOTOR_STATUS_OK;
}

static void rs_motor_sync_legacy_state(RsMotor *motor)
{
    RsMotorState *legacy;

    if (motor == NULL || !RsMotor_IsValidMotorId(motor->config.motor_id)) {
        return;
    }

    legacy = &g_rs_motors[motor->config.motor_id - 1U];
    legacy->id = motor->config.motor_id;
    legacy->online = motor->state.online;
    legacy->model = motor->config.model;
    legacy->raw_position_rad = motor->feedback.raw_position_rad;
    legacy->last_raw_position_rad = motor->internal.last_raw_position_rad;
    legacy->wrap_count = motor->internal.wrap_count;
    legacy->total_position_rad = motor->feedback.total_position_rad;
    legacy->total_angle_deg = motor->feedback.total_position_rad * RS_MOTOR_DEG_PER_RAD;
    legacy->zero_offset_rad = motor->internal.zero_offset_rad;
    legacy->position_rad = motor->feedback.position_rad;
    legacy->angle_deg = motor->feedback.angle_deg;
    legacy->speed_rad_s = motor->feedback.speed_rad_s;
    legacy->speed_deg_s = motor->feedback.speed_deg_s;
    legacy->torque_nm = motor->feedback.torque_nm;
    legacy->temperature_c = motor->feedback.temperature_c;
    legacy->mode_state = (uint8_t)motor->feedback.run_state;
    legacy->fault = (motor->fault.uncalibrated |
                     motor->fault.stall_overload |
                     motor->fault.magnetic_encoder |
                     motor->fault.over_temperature |
                     motor->fault.drive |
                     motor->fault.under_voltage) ? 1U : 0U;
    legacy->warning = 0U;
    legacy->target_position_rad = motor->internal.target_position_rad;
    legacy->target_speed_rad_s = motor->internal.target_speed_rad_s;
    legacy->update_count = motor->state.feedback_count;
    legacy->last_update_ms = motor->state.last_update_ms;
}

static RsMotorStatus rs_motor_send(RsMotor *motor,
                                   uint8_t communication_type,
                                   uint16_t communication_data,
                                   const uint8_t data[8])
{
    uint32_t ext_id;

    if (motor == NULL || data == NULL) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    ext_id = rs_motor_make_ext_id(communication_type,
                                  communication_data,
                                  motor->config.motor_id);

    return (fdcanx_send_ext_data(motor->config.hfdcan,
                                 ext_id,
                                 (uint8_t *)data,
                                 8U) == 0U) ? RS_MOTOR_STATUS_OK
                                            : RS_MOTOR_STATUS_TX_ERROR;
}

static RsMotorStatus rs_motor_send_enable(RsMotor *motor)
{
    static const uint8_t report_data[8] = {
        0x01U, 0x02U, 0x03U, 0x04U,
        0x05U, 0x06U, 0x01U, 0x08U,
    };
    static const uint8_t enable_data[8] = {0};
    RsMotorStatus status;

    /* Private protocol type 0x18 enables proactive feedback. Some motor
     * firmware sends only one response frame until this is enabled. */
    status = rs_motor_send(motor,
                           RS_COMM_TYPE_PROACTIVE_REPORT,
                           motor->config.host_id,
                           report_data);
    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }

    return rs_motor_send(motor,
                         RS_COMM_TYPE_ENABLE,
                         motor->config.host_id,
                         enable_data);
}

static RsMotorStatus rs_motor_send_disable(RsMotor *motor)
{
    static const uint8_t data[8] = {0};

    return rs_motor_send(motor,
                         RS_COMM_TYPE_DISABLE,
                         motor->config.host_id,
                         data);
}

static RsMotorStatus rs_motor_write_mode(RsMotor *motor, RsMotorControlMode mode)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)RS_PARAM_CONTROL_MODE;
    data[1] = (uint8_t)(RS_PARAM_CONTROL_MODE >> 8U);
    data[4] = (uint8_t)mode;
    return rs_motor_send(motor,
                         RS_COMM_TYPE_PARAMETER_WRITE,
                         motor->config.host_id,
                         data);
}

static RsMotorStatus rs_motor_write_float(RsMotor *motor, uint16_t index, float value)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8U);
    rs_motor_put_le_float(&data[4], value);
    return rs_motor_send(motor,
                         RS_COMM_TYPE_PARAMETER_WRITE,
                         motor->config.host_id,
                         data);
}

static RsMotorStatus rs_motor_apply_mode(RsMotor *motor, RsMotorControlMode mode)
{
    RsMotorStatus status;
    uint8_t mode_changed;

    mode_changed = (motor->internal.mode_applied == 0U ||
                    motor->internal.applied_control_mode != mode) ? 1U : 0U;

    /* RobStride private protocol expects the control mode parameter to be
     * written before normal CSP target updates. Rewriting only on mode change
     * keeps the periodic position path small and predictable. */
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
        rs_motor_sync_legacy_state(motor);
        return RS_MOTOR_STATUS_OK;
    }

    if (motor->state.enabled == 0U) {
        status = rs_motor_send_enable(motor);
        if (status != RS_MOTOR_STATUS_OK) {
            return status;
        }
        motor->state.enabled = 1U;
        rs_motor_sync_legacy_state(motor);
    }

    return RS_MOTOR_STATUS_OK;
}

static void rs_motor_update_position_feedback(RsMotor *motor, float raw_position_rad)
{
    float span_rad;
    float half_span_rad;

    span_rad = motor->config.params.position_max_rad -
               motor->config.params.position_min_rad;

    if (motor->state.online != 0U) {
        float diff = raw_position_rad - motor->feedback.raw_position_rad;
        half_span_rad = span_rad * 0.5f;

        /* Multi-turn expansion: the protocol position is finite-span. When the
         * raw value jumps across the span boundary, record one wrap so feedback
         * can continue past targets like 361 degrees. */
        if (diff > half_span_rad) {
            motor->internal.wrap_count--;
        } else if (diff < -half_span_rad) {
            motor->internal.wrap_count++;
        }
    }

    motor->internal.last_raw_position_rad = motor->feedback.raw_position_rad;
    motor->feedback.raw_position_rad = raw_position_rad;
    motor->feedback.total_position_rad =
        raw_position_rad + (float)motor->internal.wrap_count * span_rad;
    motor->feedback.position_rad =
        motor->feedback.total_position_rad - motor->internal.zero_offset_rad;
    motor->feedback.angle_deg = motor->feedback.position_rad * RS_MOTOR_DEG_PER_RAD;
}

static void rs_motor_reset_runtime(RsMotor *motor)
{
    RsMotorConfig config = motor->config;
    RsMotor *next = motor->internal.next;

    memset(motor, 0, sizeof(*motor));
    motor->config = config;
    motor->internal.next = next;
    motor->state.control_mode = RS_MOTOR_CONTROL_MODE_NONE;
    motor->internal.applied_control_mode = RS_MOTOR_CONTROL_MODE_NONE;
}

/* ==========================================================================
 * Object API
 * ========================================================================== */

RsMotorStatus rs_motor_init(RsMotor *motor)
{
    RsMotor *current;

    if (motor == NULL) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (rs_motor_find_pointer(motor) != NULL) {
        return RS_MOTOR_STATUS_DUPLICATE_INSTANCE;
    }

    if (rs_motor_config_is_valid(&motor->config) == 0U) {
        return RS_MOTOR_STATUS_INVALID_CONFIG;
    }

    current = s_motor_list;
    while (current != NULL) {
        if (current->config.hfdcan == motor->config.hfdcan &&
            current->config.host_id == motor->config.host_id &&
            current->config.motor_id == motor->config.motor_id) {
            return RS_MOTOR_STATUS_DUPLICATE_INSTANCE;
        }
        current = current->internal.next;
    }

    rs_motor_reset_runtime(motor);
    motor->internal.initialized = 1U;
    motor->internal.next = s_motor_list;
    s_motor_list = motor;
    rs_motor_sync_legacy_state(motor);
    return RS_MOTOR_STATUS_OK;
}

RsMotorStatus rs_motor_deinit(RsMotor *motor)
{
    RsMotor **link;

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
    memset(&motor->internal, 0, sizeof(motor->internal));
    memset(&motor->feedback, 0, sizeof(motor->feedback));
    memset(&motor->state, 0, sizeof(motor->state));
    memset(&motor->fault, 0, sizeof(motor->fault));
    motor->state.control_mode = RS_MOTOR_CONTROL_MODE_NONE;
    motor->internal.applied_control_mode = RS_MOTOR_CONTROL_MODE_NONE;
    return RS_MOTOR_STATUS_OK;
}

RsMotorStatus rs_motor_enable(RsMotor *motor)
{
    RsMotorStatus status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }

    status = rs_motor_send_enable(motor);
    if (status == RS_MOTOR_STATUS_OK) {
        motor->state.enabled = 1U;
        rs_motor_sync_legacy_state(motor);
    }
    return status;
}

RsMotorStatus rs_motor_disable(RsMotor *motor)
{
    RsMotorStatus status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }

    status = rs_motor_send_disable(motor);
    if (status == RS_MOTOR_STATUS_OK) {
        motor->state.enabled = 0U;
        motor->internal.mode_applied = 0U;
        motor->internal.applied_control_mode = RS_MOTOR_CONTROL_MODE_NONE;
        motor->state.control_mode = RS_MOTOR_CONTROL_MODE_NONE;
        rs_motor_sync_legacy_state(motor);
    }
    return status;
}

RsMotorStatus rs_motor_csp_position_control(RsMotor *motor,
                                            float speed_limit_rad_s,
                                            float position_rad)
{
    RsMotorStatus status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }
    if (rs_motor_is_finite(speed_limit_rad_s) == 0U ||
        rs_motor_is_finite(position_rad) == 0U) {
        return RS_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    speed_limit_rad_s = rs_motor_clamp(speed_limit_rad_s,
                                       motor->config.params.speed_min_rad_s,
                                       motor->config.params.speed_max_rad_s);

    motor->internal.target_position_rad = position_rad;
    motor->internal.target_speed_rad_s = speed_limit_rad_s;
    rs_motor_sync_legacy_state(motor);

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

uint8_t rs_motor_handle_rx(FDCAN_HandleTypeDef *hfdcan,
                           const FDCAN_RxHeaderTypeDef *header,
                           const uint8_t data[8],
                           uint32_t now_ms)
{
    uint32_t identifier;
    uint16_t communication_data;
    uint8_t communication_type;
    uint8_t host_id;
    uint8_t motor_id;
    uint8_t run_state;
    RsMotor *motor;

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
    communication_type =
        (uint8_t)((identifier >> RS_EXT_ID_TYPE_SHIFT) & RS_EXT_ID_TYPE_MASK);
    if (communication_type != RS_COMM_TYPE_FEEDBACK) {
        return 0U;
    }

    host_id = (uint8_t)(identifier & RS_EXT_ID_BYTE_MASK);
    communication_data = (uint16_t)(identifier >> RS_EXT_ID_DATA_SHIFT);
    motor_id = (uint8_t)(communication_data & RS_EXT_ID_BYTE_MASK);
    motor = rs_motor_find_by_id(hfdcan, host_id, motor_id);
    if (motor == NULL) {
        return 0U;
    }

    motor->fault.under_voltage = (uint8_t)((communication_data >> 8U) & 1U);
    motor->fault.drive = (uint8_t)((communication_data >> 9U) & 1U);
    motor->fault.over_temperature = (uint8_t)((communication_data >> 10U) & 1U);
    motor->fault.magnetic_encoder = (uint8_t)((communication_data >> 11U) & 1U);
    motor->fault.stall_overload = (uint8_t)((communication_data >> 12U) & 1U);
    motor->fault.uncalibrated = (uint8_t)((communication_data >> 13U) & 1U);

    run_state = (uint8_t)((communication_data >> 14U) & 0x03U);
    if (run_state > (uint8_t)RS_MOTOR_RUN_STATE_RUNNING) {
        run_state = (uint8_t)RS_MOTOR_RUN_STATE_REST;
    }
    motor->feedback.run_state = (RsMotorRunState)run_state;

    rs_motor_update_position_feedback(
        motor,
        rs_motor_u16_to_float(rs_motor_get_be_u16(&data[0]),
                              motor->config.params.position_min_rad,
                              motor->config.params.position_max_rad));
    motor->feedback.speed_rad_s =
        rs_motor_u16_to_float(rs_motor_get_be_u16(&data[2]),
                              motor->config.params.speed_min_rad_s,
                              motor->config.params.speed_max_rad_s);
    motor->feedback.speed_deg_s = motor->feedback.speed_rad_s * RS_MOTOR_DEG_PER_RAD;
    motor->feedback.torque_nm =
        rs_motor_u16_to_float(rs_motor_get_be_u16(&data[4]),
                              motor->config.params.torque_min_nm,
                              motor->config.params.torque_max_nm);
    motor->feedback.temperature_c = (float)rs_motor_get_be_u16(&data[6]) * 0.1f;

    motor->state.online = 1U;
    motor->state.feedback_count++;
    motor->state.last_update_ms = now_ms;
    rs_motor_sync_legacy_state(motor);
    return 1U;
}

RsMotorStatus rs_motor_update(RsMotor *motor, uint32_t now_ms)
{
    RsMotorStatus status = rs_motor_require_initialized(motor);

    if (status != RS_MOTOR_STATUS_OK) {
        return status;
    }

    if (motor->state.feedback_count != 0U &&
        (uint32_t)(now_ms - motor->state.last_update_ms) >=
            motor->config.offline_timeout_ms) {
        motor->state.online = 0U;
        rs_motor_sync_legacy_state(motor);
    }
    return RS_MOTOR_STATUS_OK;
}

/* ==========================================================================
 * Default instances and legacy API
 * ========================================================================== */

void RsMotor_Init(void)
{
    s_motor_list = NULL;
    s_host_id = RS_MOTOR_DEFAULT_HOST_ID;
    memset(g_rs_motors, 0, sizeof(g_rs_motors));
}

RsMotor *RsMotor_GetObject(uint8_t motor_id)
{
    RsMotor *current = s_motor_list;

    while (current != NULL) {
        if (current->config.motor_id == motor_id) {
            return current;
        }
        current = current->internal.next;
    }
    return NULL;
}

uint8_t RsMotor_ConfigureMotor(uint8_t motor_id, RsMotorModel model)
{
    const RsMotorModelParams *params;
    RsMotor *motor = RsMotor_GetObject(motor_id);

    if (motor == NULL) {
        return 0U;
    }
    params = RsMotor_GetDefaultParams(model);
    if (params == NULL) {
        return 0U;
    }

    motor->config.model = model;
    motor->config.params = *params;
    motor->config.host_id = s_host_id;
    rs_motor_reset_runtime(motor);
    motor->internal.initialized = 1U;
    rs_motor_sync_legacy_state(motor);
    return 1U;
}

uint8_t RsMotor_SetCustomModelParams(uint8_t motor_id, const RsMotorModelParams *params)
{
    RsMotor *motor = RsMotor_GetObject(motor_id);

    if (motor == NULL || params == NULL) {
        return 0U;
    }
    if (params->position_max_rad <= params->position_min_rad ||
        params->speed_max_rad_s <= params->speed_min_rad_s ||
        params->torque_max_nm <= params->torque_min_nm) {
        return 0U;
    }

    motor->config.model = RS_MOTOR_MODEL_CUSTOM;
    motor->config.params = *params;
    rs_motor_reset_runtime(motor);
    motor->internal.initialized = 1U;
    rs_motor_sync_legacy_state(motor);
    return 1U;
}

RsMotorState *RsMotor_GetState(uint8_t motor_id)
{
    if (!RsMotor_IsValidMotorId(motor_id)) {
        return NULL;
    }
    return &g_rs_motors[motor_id - 1U];
}

uint8_t RsMotor_SetHostId(uint8_t host_id)
{
    if (host_id == 0U) {
        return 0U;
    }

    s_host_id = host_id;
    for (RsMotor *motor = s_motor_list; motor != NULL; motor = motor->internal.next) {
        motor->config.host_id = host_id;
        rs_motor_sync_legacy_state(motor);
    }
    return 1U;
}

uint8_t RsMotor_GetHostId(void)
{
    return s_host_id;
}

uint8_t RsMotor_IsFeedbackId(uint16_t std_id)
{
    return (std_id == (uint16_t)s_host_id) ? 1U : 0U;
}

uint8_t RsMotor_IsPrivateFeedbackId(uint32_t ext_id)
{
    uint8_t communication_type =
        (uint8_t)((ext_id >> RS_EXT_ID_TYPE_SHIFT) & RS_EXT_ID_TYPE_MASK);
    uint8_t host_id = (uint8_t)(ext_id & RS_EXT_ID_BYTE_MASK);

    return (communication_type == RS_COMM_TYPE_FEEDBACK &&
            host_id == s_host_id) ? 1U : 0U;
}

uint8_t RsMotor_HandlePrivateFeedback(uint32_t ext_id,
                                      const uint8_t data[8],
                                      uint32_t now_ms)
{
    uint16_t communication_data;
    uint8_t communication_type;
    uint8_t host_id;
    uint8_t motor_id;
    uint8_t run_state;
    RsMotor *motor;

    if (data == NULL || ext_id > RS_EXT_ID_MAX) {
        return 0U;
    }

    communication_type =
        (uint8_t)((ext_id >> RS_EXT_ID_TYPE_SHIFT) & RS_EXT_ID_TYPE_MASK);
    if (communication_type != RS_COMM_TYPE_FEEDBACK) {
        return 0U;
    }

    host_id = (uint8_t)(ext_id & RS_EXT_ID_BYTE_MASK);
    communication_data = (uint16_t)(ext_id >> RS_EXT_ID_DATA_SHIFT);
    motor_id = (uint8_t)(communication_data & RS_EXT_ID_BYTE_MASK);
    motor = rs_motor_find_by_id(&hfdcan3, host_id, motor_id);
    if (motor == NULL) {
        return 0U;
    }

    motor->fault.under_voltage = (uint8_t)((communication_data >> 8U) & 1U);
    motor->fault.drive = (uint8_t)((communication_data >> 9U) & 1U);
    motor->fault.over_temperature = (uint8_t)((communication_data >> 10U) & 1U);
    motor->fault.magnetic_encoder = (uint8_t)((communication_data >> 11U) & 1U);
    motor->fault.stall_overload = (uint8_t)((communication_data >> 12U) & 1U);
    motor->fault.uncalibrated = (uint8_t)((communication_data >> 13U) & 1U);

    run_state = (uint8_t)((communication_data >> 14U) & 0x03U);
    if (run_state > (uint8_t)RS_MOTOR_RUN_STATE_RUNNING) {
        run_state = (uint8_t)RS_MOTOR_RUN_STATE_REST;
    }
    motor->feedback.run_state = (RsMotorRunState)run_state;

    rs_motor_update_position_feedback(
        motor,
        rs_motor_u16_to_float(rs_motor_get_be_u16(&data[0]),
                              motor->config.params.position_min_rad,
                              motor->config.params.position_max_rad));
    motor->feedback.speed_rad_s =
        rs_motor_u16_to_float(rs_motor_get_be_u16(&data[2]),
                              motor->config.params.speed_min_rad_s,
                              motor->config.params.speed_max_rad_s);
    motor->feedback.speed_deg_s = motor->feedback.speed_rad_s * RS_MOTOR_DEG_PER_RAD;
    motor->feedback.torque_nm =
        rs_motor_u16_to_float(rs_motor_get_be_u16(&data[4]),
                              motor->config.params.torque_min_nm,
                              motor->config.params.torque_max_nm);
    motor->feedback.temperature_c = (float)rs_motor_get_be_u16(&data[6]) * 0.1f;

    motor->state.online = 1U;
    motor->state.feedback_count++;
    motor->state.last_update_ms = now_ms;
    rs_motor_sync_legacy_state(motor);
    return motor_id;
}

uint8_t RsMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms)
{
    (void)std_id;
    (void)data;
    (void)now_ms;
    return 0U;
}

void RsMotor_SetZeroToCurrent(uint8_t motor_id)
{
    RsMotor *motor = RsMotor_GetObject(motor_id);

    if (motor == NULL) {
        return;
    }
    motor->internal.zero_offset_rad = motor->feedback.total_position_rad;
    motor->feedback.position_rad = 0.0f;
    motor->feedback.angle_deg = 0.0f;
    rs_motor_sync_legacy_state(motor);
}

uint8_t RsMotor_SetTargetPositionDeg(uint8_t motor_id,
                                     float position_deg,
                                     float max_speed_deg_s)
{
    RsMotor *motor = RsMotor_GetObject(motor_id);

    if (motor == NULL ||
        rs_motor_is_finite(position_deg) == 0U ||
        rs_motor_is_finite(max_speed_deg_s) == 0U) {
        return 0U;
    }

    motor->internal.target_position_rad = position_deg * RS_MOTOR_RAD_PER_DEG;
    motor->internal.target_speed_rad_s = max_speed_deg_s * RS_MOTOR_RAD_PER_DEG;
    rs_motor_sync_legacy_state(motor);
    return 1U;
}

uint8_t RsMotor_CspPositionControl(uint8_t motor_id,
                                   float position_rad,
                                   float speed_limit_rad_s)
{
    return (rs_motor_csp_position_control(RsMotor_GetObject(motor_id),
                                          speed_limit_rad_s,
                                          position_rad) == RS_MOTOR_STATUS_OK) ? 1U : 0U;
}

/* ==========================================================================
 * Private-protocol frame builders
 * ========================================================================== */

uint8_t RsMotor_BuildPrivateEnableFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == NULL || data == NULL) {
        return 0U;
    }

    *ext_id = rs_motor_make_ext_id(RS_COMM_TYPE_ENABLE, s_host_id, motor_id);
    rs_motor_fill(data, 0U);
    return 1U;
}

uint8_t RsMotor_BuildPrivateStopFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == NULL || data == NULL) {
        return 0U;
    }

    *ext_id = rs_motor_make_ext_id(RS_COMM_TYPE_DISABLE, s_host_id, motor_id);
    rs_motor_fill(data, 0U);
    return 1U;
}

uint8_t RsMotor_BuildPrivateRunModeFrame(uint8_t motor_id,
                                         uint8_t run_mode,
                                         uint32_t *ext_id,
                                         uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == NULL || data == NULL) {
        return 0U;
    }

    *ext_id = rs_motor_make_ext_id(RS_COMM_TYPE_PARAMETER_WRITE, s_host_id, motor_id);
    rs_motor_fill(data, 0U);
    data[0] = (uint8_t)RS_PARAM_CONTROL_MODE;
    data[1] = (uint8_t)(RS_PARAM_CONTROL_MODE >> 8U);
    data[4] = run_mode;
    return 1U;
}

uint8_t RsMotor_BuildPrivateParamWriteFrame(uint8_t motor_id,
                                            uint16_t index,
                                            float value,
                                            uint32_t *ext_id,
                                            uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == NULL || data == NULL) {
        return 0U;
    }

    *ext_id = rs_motor_make_ext_id(RS_COMM_TYPE_PARAMETER_WRITE, s_host_id, motor_id);
    rs_motor_fill(data, 0U);
    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8U);
    rs_motor_put_le_float(&data[4], value);
    return 1U;
}

uint8_t RsMotor_BuildPrivatePositionFrame(uint8_t motor_id,
                                          float position_rad,
                                          float limit_spd_rad_s,
                                          uint32_t *ext_id,
                                          uint8_t data[8])
{
    RsMotor *motor = RsMotor_GetObject(motor_id);

    if (motor == NULL || ext_id == NULL || data == NULL) {
        return 0U;
    }

    motor->internal.target_position_rad = position_rad;
    motor->internal.target_speed_rad_s = limit_spd_rad_s;
    rs_motor_sync_legacy_state(motor);
    return RsMotor_BuildPrivateParamWriteFrame(motor_id,
                                               RS_PARAM_POSITION_TARGET,
                                               position_rad,
                                               ext_id,
                                               data);
}

/* ==========================================================================
 * Legacy standard-frame helpers
 * ========================================================================== */

static uint16_t rs_motor_make_standard_mode_id(uint8_t mode, uint8_t motor_id)
{
    return (uint16_t)((((uint16_t)mode & 0x7U) << 8U) | (uint16_t)motor_id);
}

static uint8_t rs_motor_prepare_standard_frame(uint8_t motor_id,
                                               uint16_t *std_id,
                                               uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || std_id == NULL || data == NULL) {
        return 0U;
    }

    *std_id = motor_id;
    rs_motor_fill(data, 0xFFU);
    return 1U;
}

uint8_t RsMotor_BuildEnableFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (rs_motor_prepare_standard_frame(motor_id, std_id, data) == 0U) {
        return 0U;
    }
    data[7] = 0xFCU;
    return 1U;
}

uint8_t RsMotor_BuildStopFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (rs_motor_prepare_standard_frame(motor_id, std_id, data) == 0U) {
        return 0U;
    }
    data[7] = 0xFDU;
    return 1U;
}

uint8_t RsMotor_BuildClearFaultFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (rs_motor_prepare_standard_frame(motor_id, std_id, data) == 0U) {
        return 0U;
    }
    data[6] = 0xFFU;
    data[7] = 0xFBU;
    return 1U;
}

uint8_t RsMotor_BuildSetZeroFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (rs_motor_prepare_standard_frame(motor_id, std_id, data) == 0U) {
        return 0U;
    }
    data[7] = 0xFEU;
    return 1U;
}

uint8_t RsMotor_BuildSetRunModeFrame(uint8_t motor_id,
                                     RsMotorRunMode mode,
                                     uint16_t *std_id,
                                     uint8_t data[8])
{
    if (rs_motor_prepare_standard_frame(motor_id, std_id, data) == 0U) {
        return 0U;
    }
    data[6] = (uint8_t)mode;
    data[7] = 0xFCU;
    return 1U;
}

uint8_t RsMotor_BuildSetProtocolFrame(uint8_t motor_id,
                                      RsMotorProtocol protocol,
                                      uint16_t *std_id,
                                      uint8_t data[8])
{
    if (rs_motor_prepare_standard_frame(motor_id, std_id, data) == 0U) {
        return 0U;
    }
    data[6] = (uint8_t)protocol;
    data[7] = 0xFDU;
    return 1U;
}

uint8_t RsMotor_BuildPositionFrame(uint8_t motor_id,
                                   float position_deg,
                                   float max_speed_deg_s,
                                   uint16_t *std_id,
                                   uint8_t data[8])
{
    RsMotor *motor = RsMotor_GetObject(motor_id);

    if (motor == NULL || std_id == NULL || data == NULL) {
        return 0U;
    }

    motor->internal.target_position_rad = position_deg * RS_MOTOR_RAD_PER_DEG;
    motor->internal.target_speed_rad_s = max_speed_deg_s * RS_MOTOR_RAD_PER_DEG;
    *std_id = rs_motor_make_standard_mode_id(RS_STANDARD_MODE_POSITION, motor_id);
    rs_motor_put_le_float(&data[0], motor->internal.target_position_rad);
    rs_motor_put_le_float(&data[4], motor->internal.target_speed_rad_s);
    rs_motor_sync_legacy_state(motor);
    return 1U;
}

uint8_t RsMotor_BuildPositionFrameFromTarget(uint8_t motor_id,
                                             uint16_t *std_id,
                                             uint8_t data[8])
{
    RsMotor *motor = RsMotor_GetObject(motor_id);

    if (motor == NULL || std_id == NULL || data == NULL) {
        return 0U;
    }

    *std_id = rs_motor_make_standard_mode_id(RS_STANDARD_MODE_POSITION, motor_id);
    rs_motor_put_le_float(&data[0], motor->internal.target_position_rad);
    rs_motor_put_le_float(&data[4], motor->internal.target_speed_rad_s);
    return 1U;
}
