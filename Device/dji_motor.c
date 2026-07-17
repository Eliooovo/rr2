#include "dji_motor.h"

DjiMotorState g_dji_motors[DJI_MOTOR_COUNT] = {0};

static int16_t s_current_cmd[DJI_MOTOR_COUNT] = {0};

static int16_t DjiMotor_BytesToI16(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)((uint16_t)high << 8U) | (uint16_t)low);
}

static uint16_t DjiMotor_BytesToU16(uint8_t high, uint8_t low)
{
    return (uint16_t)((uint16_t)((uint16_t)high << 8U) | (uint16_t)low);
}

static int16_t DjiMotor_LimitCurrent(int16_t current)
{
    if (current > DJI_MOTOR_CURRENT_LIMIT) {
        return DJI_MOTOR_CURRENT_LIMIT;
    }
    if (current < -DJI_MOTOR_CURRENT_LIMIT) {
        return -DJI_MOTOR_CURRENT_LIMIT;
    }
    return current;
}

void DjiMotor_Init(void)
{
    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        g_dji_motors[i].id = (uint8_t)(i + 1U);
        g_dji_motors[i].online = 0U;
        g_dji_motors[i].encoder = 0U;
        g_dji_motors[i].last_encoder = 0U;  //encoder 是 DJI C620 反馈里的 电机机械角度编码器值，当前单圈角度0~8191，单位是 1/8192 圈
        g_dji_motors[i].speed_rpm = 0;      // C620 反馈的转速，单位 rpm
        g_dji_motors[i].given_current = 0;  // C620 回传的原始电流/转矩相关量，不是 STM32 发出的命令值
        g_dji_motors[i].temperature = 0U;   // C620 反馈温度，通常可按摄氏度理解
        g_dji_motors[i].round_count = 0;
        g_dji_motors[i].total_encoder = 0;
        g_dji_motors[i].total_angle_deg = 0.0f;
        g_dji_motors[i].update_count = 0U;
        g_dji_motors[i].last_update_ms = 0U;
        s_current_cmd[i] = 0;
    }
}

uint8_t DjiMotor_IsFeedbackId(uint16_t std_id)
{
    return (std_id >= DJI_MOTOR_FEEDBACK_ID_MIN && std_id <= DJI_MOTOR_FEEDBACK_ID_MAX) ? 1U : 0U;
}

uint8_t DjiMotor_GetMotorIdFromFeedbackId(uint16_t std_id)
{
    if (!DjiMotor_IsFeedbackId(std_id)) {
        return 0U;
    }
    return (uint8_t)(std_id - DJI_MOTOR_FEEDBACK_ID_BASE + 1U);
}

uint8_t DjiMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms)
{
    uint8_t motor_id = DjiMotor_GetMotorIdFromFeedbackId(std_id);
    if (motor_id == 0U) {
        return 0U;
    }

    DjiMotorState *motor = &g_dji_motors[motor_id - 1U];
    uint16_t encoder = DjiMotor_BytesToU16(data[0], data[1]);
    uint16_t previous_encoder = motor->encoder;

    /*
     * C620 反馈的 encoder 只有 0~8191 单圈值。
     * 这里先做基础多圈累计，后续升降位置控制可以直接用 total_encoder/total_angle_deg。
     */
    if (motor->online != 0U) {
        int32_t diff = (int32_t)encoder - (int32_t)previous_encoder;
        if (diff > DJI_MOTOR_ENCODER_HALF) {
            motor->round_count--;
        } else if (diff < -DJI_MOTOR_ENCODER_HALF) {
            motor->round_count++;
        }
    }

    motor->online = 1U;
    motor->last_encoder = previous_encoder;
    motor->encoder = encoder;
    motor->speed_rpm = DjiMotor_BytesToI16(data[2], data[3]);
    motor->given_current = DjiMotor_BytesToI16(data[4], data[5]);
    motor->temperature = data[6];
    motor->total_encoder = motor->round_count * DJI_MOTOR_ENCODER_RANGE + (int32_t)motor->encoder;
    motor->total_angle_deg = ((float)motor->total_encoder * 360.0f) / (float)DJI_MOTOR_ENCODER_RANGE;
    motor->update_count++;
    motor->last_update_ms = now_ms;

    return motor_id;
}

DjiMotorState *DjiMotor_GetState(uint8_t motor_id)
{
    if (motor_id == 0U || motor_id > DJI_MOTOR_COUNT) {
        return 0;
    }
    return &g_dji_motors[motor_id - 1U];
}

void DjiMotor_ClearCurrents(void)
{
    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        s_current_cmd[i] = 0;
    }
}

void DjiMotor_SetCurrent(uint8_t motor_id, int16_t current)
{
    if (motor_id == 0U || motor_id > DJI_MOTOR_COUNT) {
        return;
    }
    s_current_cmd[motor_id - 1U] = DjiMotor_LimitCurrent(current);
}

int16_t DjiMotor_GetCurrent(uint8_t motor_id)
{
    if (motor_id == 0U || motor_id > DJI_MOTOR_COUNT) {
        return 0;
    }
    return s_current_cmd[motor_id - 1U];
}

uint8_t DjiMotor_BuildCurrentFrame(uint16_t cmd_id, uint8_t data[8])
{
    uint8_t start_index;

    if (cmd_id == DJI_MOTOR_CMD_ID_1_TO_4) {
        start_index = 0U;
    } else if (cmd_id == DJI_MOTOR_CMD_ID_5_TO_8) {
        start_index = 4U;
    } else {
        return 0U;
    }

    for (uint8_t i = 0U; i < 4U; ++i) {
        int16_t current = s_current_cmd[start_index + i];
        data[i * 2U] = (uint8_t)(((uint16_t)current >> 8U) & 0xFFU);
        data[i * 2U + 1U] = (uint8_t)((uint16_t)current & 0xFFU);
    }

    return 1U;
}

void DjiMotor_BuildAllCurrentFrames(uint8_t data_1_to_4[8], uint8_t data_5_to_8[8])
{
    (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_1_TO_4, data_1_to_4);
    (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_5_TO_8, data_5_to_8);
}
