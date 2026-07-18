/**
 * @file    dji_motor.c
 * @brief   DJI M3508 电机驱动 (C620 电调) — CAN 协议层
 *
 * 硬件: DJI RoboMaster M3508 三相无刷减速电机 + C620 电调
 * 通信: CAN 2.0B, 1Mbps
 *
 * ---- CAN 协议说明 ----
 *
 * [反馈帧] 电调 → MCU, 1kHz 自动上报
 *   ID: 0x201~0x208, 8 字节
 *   ┌──────────┬──────────┬──────────┬──────────┬──────┬──────┐
 *   │ Byte 0-1 │ Byte 2-3 │ Byte 4-5 │ Byte 6   │ Byte7│      │
 *   │ 机械角度 │ 转速     │ 转矩电流 │ 电调温度 │ 保留 │      │
 *   │ 0~8191   │ rpm      │ int16    │ ℃       │      │      │
 *   └──────────┴──────────┴──────────┴──────────┴──────┴──────┘
 *
 * [命令帧] MCU → 电调
 *   0x200: 电机 1~4 的电流 (各 2 字节大端, 共 8 字节)
 *   0x1FF: 电机 5~8 的电流
 *   电流范围: ±16384
 *
 * ---- 多圈累计逻辑 ----
 * C620 只反馈单圈角度 (0~8191)，本模块通过检测相邻两次反馈的跳变
 * 来推断过圈方向并累加圈数，从而支持超过 ±360° 的位置控制。
 *
 * ---- 使用示例 (底盘 4 个麦轮电机, FDCAN1) ----
 *
 * // 初始化
 * DjiMotor_Init();
 *
 * // CAN 接收回调中
 * if (DjiMotor_IsFeedbackId(RxHeader.Identifier)) {
 *     DjiMotor_HandleFeedback(RxHeader.Identifier, RxData, HAL_GetTick());
 * }
 *
 * // PID 控制循环 (1kHz 定时器)
 * for (int i = 1; i <= 4; i++) {
 *     DjiMotorState *m = DjiMotor_GetState(i);
 *     int16_t current = pid_calculate(&pid[i], target_rpm[i], m->speed_rpm);
 *     DjiMotor_SetCurrent(i, current);
 * }
 * uint8_t tx_data[8];
 * DjiMotor_BuildCurrentFrame(0x200, tx_data);
 * HAL_FDCAN_AddMessageToTxFifo(&hfdcan1, &TxHeader, tx_data);
 */

#include "dji_motor.h"

/* ==========================================================================
 * 全局变量
 * ========================================================================== */

/* 所有电机的实时状态数组，由 CAN 反馈处理函数更新 */
DjiMotorState g_dji_motors[DJI_MOTOR_COUNT] = {0};

/* 电流指令缓存，由 DjiMotor_SetCurrent 写入，由 BuildCurrentFrame 读出打包 */
static int16_t s_current_cmd[DJI_MOTOR_COUNT] = {0};

/* ==========================================================================
 * 工具函数 (static, 仅本文件内使用)
 * ========================================================================== */

/**
 * @brief  将两个字节按大端序合成 int16_t
 * @param  high 高字节
 * @param  low  低字节
 * @return 合成的有符号 16 位整数
 */
static int16_t DjiMotor_BytesToI16(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)((uint16_t)high << 8U) | (uint16_t)low);
}

/**
 * @brief  将两个字节按大端序合成 uint16_t
 */
static uint16_t DjiMotor_BytesToU16(uint8_t high, uint8_t low)
{
    return (uint16_t)((uint16_t)((uint16_t)high << 8U) | (uint16_t)low);
}

/**
 * @brief  电流限幅，裁剪到 [-16384, 16384]
 */
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

/* ==========================================================================
 * 公有函数实现
 * ========================================================================== */

/**
 * @brief  初始化所有电机状态
 *
 * 将 8 个电机的状态结构体清零，ID 依次设为 1~8，online 标志清零。
 * 上电后调用一次即可。
 */
void DjiMotor_Init(void)
{
    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        g_dji_motors[i].id            = (uint8_t)(i + 1U);
        g_dji_motors[i].online        = 0U;
        g_dji_motors[i].encoder       = 0U;
        g_dji_motors[i].last_encoder  = 0U;
        g_dji_motors[i].speed_rpm     = 0;
        g_dji_motors[i].given_current = 0;
        g_dji_motors[i].temperature   = 0U;
        g_dji_motors[i].round_count   = 0;
        g_dji_motors[i].total_encoder = 0;
        g_dji_motors[i].total_angle_deg = 0.0f;
        g_dji_motors[i].update_count  = 0U;
        g_dji_motors[i].last_update_ms = 0U;
        s_current_cmd[i] = 0;
    }
}

/**
 * @brief  判断 CAN ID 是否为 DJI 电机反馈帧
 * @return 1=是 DJI 反馈帧, 0=不是
 */
uint8_t DjiMotor_IsFeedbackId(uint16_t std_id)
{
    return (std_id >= DJI_MOTOR_FEEDBACK_ID_MIN && std_id <= DJI_MOTOR_FEEDBACK_ID_MAX) ? 1U : 0U;
}

/**
 * @brief  从 CAN ID 提取电机编号
 *
 * 映射关系: 0x201→1, 0x202→2, ..., 0x208→8
 *
 * @param  std_id  CAN 标准帧 ID
 * @return 电机编号 1~8，非 DJI 反馈帧返回 0
 */
uint8_t DjiMotor_GetMotorIdFromFeedbackId(uint16_t std_id)
{
    if (!DjiMotor_IsFeedbackId(std_id)) {
        return 0U;
    }
    return (uint8_t)(std_id - DJI_MOTOR_FEEDBACK_ID_BASE + 1U);
}

/**
 * @brief  解析一条 CAN 反馈帧，更新对应电机的状态
 *
 * 这是整个驱动最核心的函数。
 * 每次收到 0x201~0x208 的 CAN 帧时调用，完成:
 *   1. 解析机械角度、转速、电流、温度
 *   2. 多圈累计: 通过前后两次 encoder 的跳变检测过圈
 *   3. 更新 online 标志和时间戳
 *
 * 多圈原理:
 *   C620 只上报 0~8191 的单圈角度。当电机从 8190 跳到 100 时，
 *   差值 diff = 100 - 8190 = -8090，小于 -4096 (半圈)，判定为正向过一圈，
 *   round_count++。反之 diff > 4096 则反向过一圈，round_count--。
 *
 * @param  std_id  CAN 消息 ID (0x201~0x208)
 * @param  data    8 字节 CAN 数据负载
 * @param  now_ms  当前系统毫秒时间戳 (HAL_GetTick())
 * @return 解析到的电机 ID (1~8)，非 DJI 反馈帧返回 0
 */
uint8_t DjiMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms)
{
    /* 1. 从 CAN ID 获取电机编号 */
    uint8_t motor_id = DjiMotor_GetMotorIdFromFeedbackId(std_id);
    if (motor_id == 0U) {
        return 0U;  /* 不是 DJI 反馈帧 */
    }

    /* 2. 解析数据字段 */
    DjiMotorState *motor = &g_dji_motors[motor_id - 1U];

    uint16_t encoder           = DjiMotor_BytesToU16(data[0], data[1]);  /* 当前机械角度 0~8191 */
    uint16_t previous_encoder  = motor->encoder;                          /* 上一次的机械角度 */

    /* 3. 多圈累计: 检测编码器跳变判断过圈 */
    if (motor->online != 0U) {
        /* 只有已经在线才做多圈累计 (第一次收到反馈时 previous_encoder=0，
         * 可能导致误判，所以跳过) */
        int32_t diff = (int32_t)encoder - (int32_t)previous_encoder;

        if (diff > DJI_MOTOR_ENCODER_HALF) {
            /* diff > 4096: 例如从 100 跳到 8000，说明编码器反向溢出 (反向过圈) */
            motor->round_count--;
        } else if (diff < -DJI_MOTOR_ENCODER_HALF) {
            /* diff < -4096: 例如从 8000 跳到 100，说明编码器正向溢出 (正向过圈) */
            motor->round_count++;
        }
        /* |diff| <= 4096: 正常单圈内运动，圈数不变 */
    }

    /* 4. 更新电机状态 */
    motor->online         = 1U;                                           /* 标记在线 */
    motor->last_encoder   = previous_encoder;                             /* 保存上一次编码器值 */
    motor->encoder        = encoder;                                      /* 更新当前编码器值 */
    motor->speed_rpm      = DjiMotor_BytesToI16(data[2], data[3]);       /* 转速 rpm */
    motor->given_current  = DjiMotor_BytesToI16(data[4], data[5]);       /* 电调实际电流 */
    motor->temperature    = data[6];                                      /* 温度 ℃ */
    motor->total_encoder  = motor->round_count * DJI_MOTOR_ENCODER_RANGE
                            + (int32_t)motor->encoder;                    /* 多圈累计编码器值 */
    motor->total_angle_deg = ((float)motor->total_encoder * 360.0f)
                             / (float)DJI_MOTOR_ENCODER_RANGE;            /* 多圈累计角度 (度) */
    motor->update_count++;                                                /* 反馈计数 +1 */
    motor->last_update_ms = now_ms;                                       /* 更新时间戳 */

    return motor_id;
}

/**
 * @brief  获取指定电机的状态指针
 * @param  motor_id  1~8
 * @return 指向 DjiMotorState 的指针，非法 ID 返回 NULL
 *
 * 使用示例:
 *   DjiMotorState *m = DjiMotor_GetState(1);
 *   if (m && m->online) {
 *       float angle = m->total_angle_deg;
 *       int16_t rpm = m->speed_rpm;
 *   }
 */
DjiMotorState *DjiMotor_GetState(uint8_t motor_id)
{
    if (motor_id == 0U || motor_id > DJI_MOTOR_COUNT) {
        return 0;
    }
    return &g_dji_motors[motor_id - 1U];
}

/**
 * @brief  将所有电机的电流指令清零
 *
 * 紧急停止时调用，让所有电机停止输出转矩。
 * 注意: 只清零软件缓存，上层仍需调用 Build + HAL FDCAN 发送。
 */
void DjiMotor_ClearCurrents(void)
{
    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        s_current_cmd[i] = 0;
    }
}

/**
 * @brief  设置单个电机的电流指令
 *
 * 写入软件缓存，不发送 CAN。需要在上层定时发送。
 * 电流值自动限幅到 ±16384。
 *
 * @param  motor_id  电机编号 1~8
 * @param  current   目标电流 [-16384, 16384]
 *                   - 正值: 电机正转 (具体方向取决于接线)
 *                   - 负值: 电机反转
 *                   - 0: 停止输出转矩
 *
 * 注意: 这是电流指令，不是速度指令。
 *       要实现速度控制，需要配合 PID 将目标转速转换为电流指令。
 */
void DjiMotor_SetCurrent(uint8_t motor_id, int16_t current)
{
    if (motor_id == 0U || motor_id > DJI_MOTOR_COUNT) {
        return;
    }
    s_current_cmd[motor_id - 1U] = DjiMotor_LimitCurrent(current);
}

/**
 * @brief  读取当前缓存的电流指令
 * @return 上次 DjiMotor_SetCurrent 设置的值
 *
 * 注意: 这是 MCU 发出的电流命令值，不是电调回传的实际电流。
 *       电调回传的实际电流在 DjiMotorState.given_current 中。
 */
int16_t DjiMotor_GetCurrent(uint8_t motor_id)
{
    if (motor_id == 0U || motor_id > DJI_MOTOR_COUNT) {
        return 0;
    }
    return s_current_cmd[motor_id - 1U];
}

/**
 * @brief  按 DJI 协议打包 4 个电机的电流为 8 字节 CAN 数据
 *
 * 格式: 4 个 int16_t，每个占 2 字节 (大端序)，共 8 字节
 *   [M1_high, M1_low, M2_high, M2_low, M3_high, M3_low, M4_high, M4_low]
 *
 * @param  cmd_id  0x200 打包电机 1~4，0x1FF 打包电机 5~8
 * @param  data    输出的 8 字节缓冲区
 * @return 1=成功, 0=cmd_id 非法 (不是 0x200 也不是 0x1FF)
 */
uint8_t DjiMotor_BuildCurrentFrame(uint16_t cmd_id, uint8_t data[8])
{
    uint8_t start_index;

    /* 根据 CAN ID 确定起始电机索引 */
    if (cmd_id == DJI_MOTOR_CMD_ID_1_TO_4) {
        start_index = 0U;   /* 电机 1~4 对应索引 0~3 */
    } else if (cmd_id == DJI_MOTOR_CMD_ID_5_TO_8) {
        start_index = 4U;   /* 电机 5~8 对应索引 4~7 */
    } else {
        return 0U;          /* 非法 CAN ID */
    }

    /* 把 4 个电机的电流指令按大端序填入 8 字节 */
    for (uint8_t i = 0U; i < 4U; ++i) {
        int16_t current = s_current_cmd[start_index + i];
        data[i * 2U]     = (uint8_t)(((uint16_t)current >> 8U) & 0xFFU);  /* 高字节 */
        data[i * 2U + 1U] = (uint8_t)((uint16_t)current & 0xFFU);         /* 低字节 */
    }

    return 1U;
}

/**
 * @brief  同时打包两台 CAN 命令帧 (8 个电机)
 *
 * 底盘场景 (电机 1~4) 通常只需要发送 0x200 这一帧。
 *
 * @param  data_1_to_4  输出: 电机 1~4 的 8 字节 (用于 CAN ID 0x200)
 * @param  data_5_to_8  输出: 电机 5~8 的 8 字节 (用于 CAN ID 0x1FF)
 */
void DjiMotor_BuildAllCurrentFrames(uint8_t data_1_to_4[8], uint8_t data_5_to_8[8])
{
    (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_1_TO_4, data_1_to_4);
    (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_5_TO_8, data_5_to_8);
}
