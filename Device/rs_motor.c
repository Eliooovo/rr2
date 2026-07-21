/**
 * @file    rs_motor.c
 * @brief   灵足 RobStride 电机驱动 - MIT 标准帧协议层
 *
 * 本驱动优先支持 MIT 标准帧协议，因为它能直接满足:
 *   1. CAN 通信
 *   2. 获取位置/速度/力矩/温度反馈
 *   3. 使用电机内部位置模式 PID
 *   4. 位置指令使用 float rad，可发送 361deg 这类多圈目标
 *
 * 重要前提:
 *   电机需要处于 MIT 协议。若当前是私有协议，需要先用上位机或协议切换命令切换，
 *   并重新上电生效。RsMotor_BuildSetProtocolFrame() 只负责按 MIT 手册打包该命令。
 */

#include "rs_motor.h"

RsMotorState g_rs_motors[RS_MOTOR_COUNT] = {0};

static RsMotorModelParams s_model_params[RS_MOTOR_COUNT];
static uint8_t s_host_id = RS_MOTOR_DEFAULT_HOST_ID;

/* ==========================================================================
 * 型号参数
 * ========================================================================== */

static const RsMotorModelParams s_rs00_params = {
    RS_MOTOR_MODEL_RS00,
    -12.57f, 12.57f,
    -33.0f, 33.0f,
    -14.0f, 14.0f,
    1U
};

/* RS03 暂未在主目录找到手册。这里先保留入口，量程按 RS00 近似占位。
 * 真正接 RS03 前应按 RS03 手册修正 speed/torque 范围。 */
static const RsMotorModelParams s_rs03_placeholder_params = {
    RS_MOTOR_MODEL_RS03,
    -12.57f, 12.57f,
    -33.0f, 33.0f,
    -3.0f, 3.0f,
    1U
};

static const RsMotorModelParams s_rs05_params = {
    RS_MOTOR_MODEL_RS05,
    -12.57f, 12.57f,
    -50.0f, 50.0f,
    -5.5f, 5.5f,
    0U
};

/* ==========================================================================
 * 工具函数
 * ========================================================================== */

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
    return 0;
}

static float RsMotor_UintToFloat(uint32_t value, float min_value, float max_value, uint8_t bits)
{
    uint32_t max_int = (1UL << bits) - 1UL;
    return ((float)value * (max_value - min_value) / (float)max_int) + min_value;
}

static void RsMotor_WriteFloatLe(float value, uint8_t data[4])
{
    union {
        float f;
        uint32_t u;
    } v;

    v.f = value;
    data[0] = (uint8_t)(v.u & 0xFFU);
    data[1] = (uint8_t)((v.u >> 8U) & 0xFFU);
    data[2] = (uint8_t)((v.u >> 16U) & 0xFFU);
    data[3] = (uint8_t)((v.u >> 24U) & 0xFFU);
}

static void RsMotor_FillBytes(uint8_t data[8], uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        data[i] = value;
    }
}

static uint16_t RsMotor_MakeModeId(uint8_t mode, uint8_t motor_id)
{
    return (uint16_t)((((uint16_t)mode & 0x7U) << 8U) | (uint16_t)motor_id);
}

static uint32_t RsMotor_MakePrivateExtId(uint8_t mode, uint8_t motor_id)
{
    return (((uint32_t)mode & 0x1FU) << 24U) |
           (((uint32_t)s_host_id & 0xFFU) << 8U) |
           (uint32_t)motor_id;
}

static uint8_t RsMotor_PrepareSimpleFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || std_id == 0 || data == 0) {
        return 0U;
    }

    *std_id = motor_id;
    RsMotor_FillBytes(data, 0xFFU);
    return 1U;
}

static void RsMotor_ResetState(uint8_t motor_id)
{
    RsMotorState *motor = &g_rs_motors[motor_id - 1U];
    RsMotorModel model = motor->model;
    float target_position_rad = motor->target_position_rad;
    float target_speed_rad_s = motor->target_speed_rad_s;
    float zero_offset_rad = motor->zero_offset_rad;

    motor->id = motor_id;
    motor->online = 0U;
    motor->model = model;
    motor->raw_position_rad = 0.0f;
    motor->last_raw_position_rad = 0.0f;
    motor->wrap_count = 0;
    motor->total_position_rad = 0.0f;
    motor->total_angle_deg = 0.0f;
    motor->zero_offset_rad = zero_offset_rad;
    motor->position_rad = 0.0f;
    motor->angle_deg = 0.0f;
    motor->speed_rad_s = 0.0f;
    motor->speed_deg_s = 0.0f;
    motor->torque_nm = 0.0f;
    motor->temperature_c = 0.0f;
    motor->mode_state = 0U;
    motor->fault = 0U;
    motor->warning = 0U;
    motor->target_position_rad = target_position_rad;
    motor->target_speed_rad_s = target_speed_rad_s;
    motor->update_count = 0U;
    motor->last_update_ms = 0U;
}

/* ==========================================================================
 * 初始化与配置
 * ========================================================================== */

void RsMotor_Init(void)
{
    s_host_id = RS_MOTOR_DEFAULT_HOST_ID;

    for (uint8_t i = 0U; i < RS_MOTOR_COUNT; ++i) {
        g_rs_motors[i].id = (uint8_t)(i + 1U);
        g_rs_motors[i].model = RS_MOTOR_MODEL_RS00;
        g_rs_motors[i].target_position_rad = 0.0f;
        g_rs_motors[i].target_speed_rad_s = 0.0f;
        g_rs_motors[i].zero_offset_rad = 0.0f;
        s_model_params[i] = s_rs00_params;
        RsMotor_ResetState((uint8_t)(i + 1U));
    }
}

uint8_t RsMotor_ConfigureMotor(uint8_t motor_id, RsMotorModel model)
{
    const RsMotorModelParams *params;

    if (!RsMotor_IsValidMotorId(motor_id)) {
        return 0U;
    }

    params = RsMotor_GetDefaultParams(model);
    if (params == 0) {
        return 0U;
    }

    s_model_params[motor_id - 1U] = *params;
    g_rs_motors[motor_id - 1U].model = model;
    RsMotor_ResetState(motor_id);
    return 1U;
}

uint8_t RsMotor_SetCustomModelParams(uint8_t motor_id, const RsMotorModelParams *params)
{
    if (!RsMotor_IsValidMotorId(motor_id) || params == 0) {
        return 0U;
    }
    if (params->position_max_rad <= params->position_min_rad) {
        return 0U;
    }

    s_model_params[motor_id - 1U] = *params;
    g_rs_motors[motor_id - 1U].model = RS_MOTOR_MODEL_CUSTOM;
    RsMotor_ResetState(motor_id);
    return 1U;
}

RsMotorState *RsMotor_GetState(uint8_t motor_id)
{
    if (!RsMotor_IsValidMotorId(motor_id)) {
        return 0;
    }
    return &g_rs_motors[motor_id - 1U];
}

uint8_t RsMotor_SetHostId(uint8_t host_id)
{
    if (host_id == 0U) {
        return 0U;
    }
    s_host_id = host_id;
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
    uint8_t mode = (uint8_t)((ext_id >> 24U) & 0x1FU);
    uint8_t host_id = (uint8_t)(ext_id & 0xFFU);

    return (mode == 2U && host_id == s_host_id) ? 1U : 0U;
}

/* ==========================================================================
 * 反馈解析
 * ========================================================================== */

uint8_t RsMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms)
{
    uint8_t motor_id;
    RsMotorState *motor;
    const RsMotorModelParams *params;
    uint16_t pos_u16;
    uint16_t speed_u12;
    uint16_t torque_u12;
    uint16_t temp_u;
    float raw_position_rad;
    float span_rad;
    float half_span_rad;

    if (!RsMotor_IsFeedbackId(std_id) || data == 0) {
        return 0U;
    }

    motor_id = data[0];
    if (!RsMotor_IsValidMotorId(motor_id)) {
        return 0U;
    }

    motor = &g_rs_motors[motor_id - 1U];
    params = &s_model_params[motor_id - 1U];

    pos_u16 = (uint16_t)(((uint16_t)data[1] << 8U) | (uint16_t)data[2]);
    speed_u12 = (uint16_t)(((uint16_t)data[3] << 4U) | ((uint16_t)data[4] >> 4U));
    torque_u12 = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8U) | (uint16_t)data[5]);

    raw_position_rad = RsMotor_UintToFloat(pos_u16,
                                           params->position_min_rad,
                                           params->position_max_rad,
                                           16U);

    if (motor->online != 0U) {
        float diff = raw_position_rad - motor->raw_position_rad;
        span_rad = params->position_max_rad - params->position_min_rad;
        half_span_rad = span_rad * 0.5f;

        /* 多圈展开:
         * 反馈值越过最大/最小边界时会发生大跳变。用半个反馈范围作阈值判断方向。
         * 例如 +12.5rad -> -12.5rad，diff 很负，说明正向又过了一段范围。 */
        if (diff > half_span_rad) {
            motor->wrap_count--;
        } else if (diff < -half_span_rad) {
            motor->wrap_count++;
        }
    }

    span_rad = params->position_max_rad - params->position_min_rad;

    motor->online = 1U;
    motor->last_raw_position_rad = motor->raw_position_rad;
    motor->raw_position_rad = raw_position_rad;
    motor->total_position_rad = raw_position_rad + (float)motor->wrap_count * span_rad;
    motor->total_angle_deg = motor->total_position_rad * RS_MOTOR_DEG_PER_RAD;
    motor->position_rad = motor->total_position_rad - motor->zero_offset_rad;
    motor->angle_deg = motor->position_rad * RS_MOTOR_DEG_PER_RAD;
    motor->speed_rad_s = RsMotor_UintToFloat(speed_u12,
                                             params->speed_min_rad_s,
                                             params->speed_max_rad_s,
                                             12U);
    motor->speed_deg_s = motor->speed_rad_s * RS_MOTOR_DEG_PER_RAD;
    motor->torque_nm = RsMotor_UintToFloat(torque_u12,
                                           params->torque_min_nm,
                                           params->torque_max_nm,
                                           12U);

    if (params->feedback_has_status_bits != 0U) {
        motor->mode_state = (uint8_t)((data[6] >> 6U) & 0x03U);
        motor->fault = (uint8_t)((data[6] >> 5U) & 0x01U);
        motor->warning = (uint8_t)((data[6] >> 4U) & 0x01U);
        temp_u = (uint16_t)((((uint16_t)data[6] & 0x0FU) << 8U) | (uint16_t)data[7]);
    } else {
        temp_u = (uint16_t)(((uint16_t)data[6] << 8U) | (uint16_t)data[7]);
    }
    motor->temperature_c = (float)temp_u * 0.1f;

    motor->update_count++;
    motor->last_update_ms = now_ms;

    return motor_id;
}

uint8_t RsMotor_HandlePrivateFeedback(uint32_t ext_id, const uint8_t data[8], uint32_t now_ms)
{
    uint8_t mode;
    uint8_t motor_id;
    uint8_t host_id;
    RsMotorState *motor;
    const RsMotorModelParams *params;
    uint16_t pos_u16;
    uint16_t speed_u16;
    uint16_t torque_u16;
    uint16_t temp_u16;
    float raw_position_rad;
    float span_rad;
    float half_span_rad;

    if (!RsMotor_IsPrivateFeedbackId(ext_id) || data == 0) {
        return 0U;
    }

    mode = (uint8_t)((ext_id >> 24U) & 0x1FU);
    host_id = (uint8_t)(ext_id & 0xFFU);
    motor_id = (uint8_t)((ext_id >> 8U) & 0xFFU);

    if (mode != 2U || host_id != s_host_id || !RsMotor_IsValidMotorId(motor_id)) {
        return 0U;
    }

    motor = &g_rs_motors[motor_id - 1U];
    params = &s_model_params[motor_id - 1U];

    pos_u16 = (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
    speed_u16 = (uint16_t)(((uint16_t)data[2] << 8U) | (uint16_t)data[3]);
    torque_u16 = (uint16_t)(((uint16_t)data[4] << 8U) | (uint16_t)data[5]);
    temp_u16 = (uint16_t)(((uint16_t)data[6] << 8U) | (uint16_t)data[7]);

    raw_position_rad = RsMotor_UintToFloat(pos_u16,
                                           params->position_min_rad,
                                           params->position_max_rad,
                                           16U);

    if (motor->online != 0U) {
        float diff = raw_position_rad - motor->raw_position_rad;
        span_rad = params->position_max_rad - params->position_min_rad;
        half_span_rad = span_rad * 0.5f;
        if (diff > half_span_rad) {
            motor->wrap_count--;
        } else if (diff < -half_span_rad) {
            motor->wrap_count++;
        }
    }

    span_rad = params->position_max_rad - params->position_min_rad;
    motor->online = 1U;
    motor->last_raw_position_rad = motor->raw_position_rad;
    motor->raw_position_rad = raw_position_rad;
    motor->total_position_rad = raw_position_rad + (float)motor->wrap_count * span_rad;
    motor->total_angle_deg = motor->total_position_rad * RS_MOTOR_DEG_PER_RAD;
    motor->position_rad = motor->total_position_rad - motor->zero_offset_rad;
    motor->angle_deg = motor->position_rad * RS_MOTOR_DEG_PER_RAD;
    motor->speed_rad_s = RsMotor_UintToFloat(speed_u16,
                                             params->speed_min_rad_s,
                                             params->speed_max_rad_s,
                                             16U);
    motor->speed_deg_s = motor->speed_rad_s * RS_MOTOR_DEG_PER_RAD;
    motor->torque_nm = RsMotor_UintToFloat(torque_u16,
                                           params->torque_min_nm,
                                           params->torque_max_nm,
                                           16U);
    motor->temperature_c = (float)temp_u16 * 0.1f;
    motor->mode_state = mode;
    motor->fault = 0U;
    motor->warning = 0U;
    motor->update_count++;
    motor->last_update_ms = now_ms;

    return motor_id;
}

void RsMotor_SetZeroToCurrent(uint8_t motor_id)
{
    RsMotorState *motor = RsMotor_GetState(motor_id);
    if (motor == 0) {
        return;
    }

    motor->zero_offset_rad = motor->total_position_rad;
    motor->position_rad = 0.0f;
    motor->angle_deg = 0.0f;
}

uint8_t RsMotor_SetTargetPositionDeg(uint8_t motor_id, float position_deg, float max_speed_deg_s)
{
    RsMotorState *motor = RsMotor_GetState(motor_id);
    if (motor == 0) {
        return 0U;
    }

    motor->target_position_rad = position_deg * RS_MOTOR_RAD_PER_DEG;
    motor->target_speed_rad_s = max_speed_deg_s * RS_MOTOR_RAD_PER_DEG;
    return 1U;
}

/* ==========================================================================
 * 控制帧打包
 * ========================================================================== */

uint8_t RsMotor_BuildEnableFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (!RsMotor_PrepareSimpleFrame(motor_id, std_id, data)) {
        return 0U;
    }
    data[7] = 0xFCU;
    return 1U;
}

uint8_t RsMotor_BuildStopFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (!RsMotor_PrepareSimpleFrame(motor_id, std_id, data)) {
        return 0U;
    }

    data[7] = 0xFDU;
    return 1U;
}

uint8_t RsMotor_BuildClearFaultFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (!RsMotor_PrepareSimpleFrame(motor_id, std_id, data)) {
        return 0U;
    }

    data[6] = 0xFFU;
    data[7] = 0xFBU;
    return 1U;
}

uint8_t RsMotor_BuildSetZeroFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    if (!RsMotor_PrepareSimpleFrame(motor_id, std_id, data)) {
        return 0U;
    }
    data[7] = 0xFEU;
    return 1U;
}

uint8_t RsMotor_BuildSetRunModeFrame(uint8_t motor_id, RsMotorRunMode mode,
                                     uint16_t *std_id, uint8_t data[8])
{
    if (!RsMotor_PrepareSimpleFrame(motor_id, std_id, data)) {
        return 0U;
    }

    data[6] = (uint8_t)mode;
    data[7] = 0xFCU;
    return 1U;
}

uint8_t RsMotor_BuildSetProtocolFrame(uint8_t motor_id, RsMotorProtocol protocol,
                                      uint16_t *std_id, uint8_t data[8])
{
    if (!RsMotor_PrepareSimpleFrame(motor_id, std_id, data)) {
        return 0U;
    }

    data[6] = (uint8_t)protocol;
    data[7] = 0xFDU;
    return 1U;
}

uint8_t RsMotor_BuildPrivateEnableFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == 0 || data == 0) {
        return 0U;
    }

    *ext_id = RsMotor_MakePrivateExtId(3U, motor_id);
    RsMotor_FillBytes(data, 0x00U);
    return 1U;
}

uint8_t RsMotor_BuildPrivateStopFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == 0 || data == 0) {
        return 0U;
    }

    *ext_id = RsMotor_MakePrivateExtId(4U, motor_id);
    RsMotor_FillBytes(data, 0x00U);
    return 1U;
}

uint8_t RsMotor_BuildPrivateRunModeFrame(uint8_t motor_id, uint8_t run_mode,
                                         uint32_t *ext_id, uint8_t data[8])
{
    uint16_t index = 0x7005U;

    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == 0 || data == 0) {
        return 0U;
    }

    *ext_id = RsMotor_MakePrivateExtId(0x12U, motor_id);
    RsMotor_FillBytes(data, 0x00U);
    data[0] = (uint8_t)(index & 0xFFU);
    data[1] = (uint8_t)((index >> 8U) & 0xFFU);
    data[4] = run_mode;
    return 1U;
}

uint8_t RsMotor_BuildPrivateParamWriteFrame(uint8_t motor_id, uint16_t index, float value,
                                            uint32_t *ext_id, uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == 0 || data == 0) {
        return 0U;
    }

    *ext_id = RsMotor_MakePrivateExtId(0x12U, motor_id);
    RsMotor_FillBytes(data, 0x00U);
    data[0] = (uint8_t)(index & 0xFFU);
    data[1] = (uint8_t)((index >> 8U) & 0xFFU);
    RsMotor_WriteFloatLe(value, &data[4]);
    return 1U;
}

uint8_t RsMotor_BuildPrivatePositionFrame(uint8_t motor_id, float position_rad,
                                          float limit_spd_rad_s, uint32_t *ext_id, uint8_t data[8])
{
    if (!RsMotor_IsValidMotorId(motor_id) || ext_id == 0 || data == 0) {
        return 0U;
    }

    *ext_id = RsMotor_MakePrivateExtId(0x12U, motor_id);
    RsMotor_FillBytes(data, 0x00U);
    data[0] = 0x16U;
    data[1] = 0x70U;
    RsMotor_WriteFloatLe(position_rad, &data[4]);
    (void)limit_spd_rad_s;
    return 1U;
}

uint8_t RsMotor_BuildPositionFrame(uint8_t motor_id, float position_deg, float max_speed_deg_s,
                                   uint16_t *std_id, uint8_t data[8])
{
    float position_rad;
    float speed_rad_s;

    if (!RsMotor_IsValidMotorId(motor_id) || std_id == 0 || data == 0) {
        return 0U;
    }

    position_rad = position_deg * RS_MOTOR_RAD_PER_DEG;
    speed_rad_s = max_speed_deg_s * RS_MOTOR_RAD_PER_DEG;

    g_rs_motors[motor_id - 1U].target_position_rad = position_rad;
    g_rs_motors[motor_id - 1U].target_speed_rad_s = speed_rad_s;

    *std_id = RsMotor_MakeModeId(RS_MOTOR_CAN_MODE_POSITION, motor_id);
    RsMotor_WriteFloatLe(position_rad, &data[0]);
    RsMotor_WriteFloatLe(speed_rad_s, &data[4]);

    return 1U;
}

uint8_t RsMotor_BuildPositionFrameFromTarget(uint8_t motor_id, uint16_t *std_id, uint8_t data[8])
{
    RsMotorState *motor = RsMotor_GetState(motor_id);
    if (motor == 0 || std_id == 0 || data == 0) {
        return 0U;
    }

    *std_id = RsMotor_MakeModeId(RS_MOTOR_CAN_MODE_POSITION, motor_id);
    RsMotor_WriteFloatLe(motor->target_position_rad, &data[0]);
    RsMotor_WriteFloatLe(motor->target_speed_rad_s, &data[4]);

    return 1U;
}
