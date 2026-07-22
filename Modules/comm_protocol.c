/**
 * @file    comm_protocol.c
 * @brief   USB CDC 通讯协议实现 — 帧解析、指令下发、反馈上报
 *
 * 数据流:
 *   Jetson USB → CDC_Receive_HS → Comm_OnUsbReceived → 环形缓冲区
 *   Comm_ParseRx: 找帧头 0xAA → 收 46 字节 → 验证帧尾 0x55 → 解析 11 个 float
 *   Comm_ApplyCommand: 分发到 Chassis/Lift/RS actuator 控制模块
 *   Comm_SendFeedbackPeriodic: 打包 46 字节反馈帧 → CDC_Transmit_HS → Jetson
 */

#include "comm_protocol.h"

#include <string.h>

#include "chassis.h"
#include "dji_motor.h"
#include "lift.h"
#include "main.h"
#include "rs_actuator.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"

#define COMM_RX_BUFFER_SIZE 256U    /* 环形缓冲区大小 */

/* 升降电机配对: 前组→电机1/2, 后组→电机3/4 */
#define COMM_LIFT_PAIR_FRONT_A LIFT_MOTOR_1
#define COMM_LIFT_PAIR_FRONT_B LIFT_MOTOR_2
#define COMM_LIFT_PAIR_REAR_A  LIFT_MOTOR_3
#define COMM_LIFT_PAIR_REAR_B  LIFT_MOTOR_4

/* Ozone 调试用: 观察 USB 是否收到有效速度命令。
 * 收到一帧合法命令后，count 增加，vx/vy/vw 更新为上位机发来的前三个 float。 */
volatile uint32_t g_comm_rx_valid_frame_count;
volatile float g_comm_rx_vx;
volatile float g_comm_rx_vy;
volatile float g_comm_rx_vw;
volatile uint32_t g_comm_rx_vx_bits;
volatile uint32_t g_comm_rx_vy_bits;
volatile uint32_t g_comm_rx_vw_bits;
volatile uint8_t g_comm_rx_last_packet[COMM_PACKET_SIZE];
volatile uint32_t g_comm_rx_float_bits[COMM_FLOAT_COUNT];
volatile float g_comm_apply_vx;
volatile float g_comm_apply_vy;
volatile float g_comm_apply_vw;
volatile uint32_t g_comm_apply_count;

/* --- 环形缓冲区 (USB CDC 中断写入 → 主循环读出) --- */
static volatile uint16_t s_rx_head;             /* 写指针 (中断上下文) */
static volatile uint16_t s_rx_tail;             /* 读指针 (主循环上下文) */
static uint8_t  s_rx_buffer[COMM_RX_BUFFER_SIZE];

/* --- 帧解析状态 --- */
static uint8_t  s_parse_buffer[COMM_PACKET_SIZE];  /* 正在组装的一帧 */
static uint16_t s_parse_index;                     /* 当前已收集的字节数 */

/* --- 命令与状态 --- */
static CommFrameFloats s_last_command;        /* 最近一次收到的完整命令 */
static uint8_t  s_has_command;                /* 是否收到过命令 (调试用) */
static uint8_t  s_new_command_pending;        /* 有新命令待执行 */
static uint8_t  s_lift_zeroed;                /* 升降是否已设零点 (首次在线时自动触发) */
static uint32_t s_last_feedback_ms;           /* 上次发送反馈的时间 */

#if !COMM_USB_TEST_FRAME_ENABLE
static uint8_t  s_feedback_packet[COMM_PACKET_SIZE];  /* 二进制反馈帧 */
#endif
#if COMM_USB_TEST_FRAME_ENABLE
static uint8_t  s_test_frame[COMM_PACKET_SIZE] = {
    [0]                     = COMM_FEEDBACK_HEAD,     /* 头字节 */
    [COMM_PACKET_SIZE - 1U] = COMM_FEEDBACK_TAIL,     /* 尾字节 */
    /* 中间 44 字节 float 全 0，用于验证 USB CDC 链路 */
};
#endif

/* ==========================================================================
 * 环形缓冲区
 * ========================================================================== */

/* 写入一字节 (USB CDC 中断中调用) */
static void Comm_RxPush(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % COMM_RX_BUFFER_SIZE);

    if (next == s_rx_tail) {
        /* 缓冲区满，丢弃最旧的一个字节 */
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % COMM_RX_BUFFER_SIZE);
    }

    s_rx_buffer[s_rx_head] = byte;
    s_rx_head = next;
}

/* 读出一字节 (主循环中调用)，返回 0=空 */
static uint8_t Comm_RxPop(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head) return 0U;

    *byte = s_rx_buffer[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % COMM_RX_BUFFER_SIZE);
    return 1U;
}

/* ==========================================================================
 * float 序列化 (小端序)
 * ========================================================================== */

/* 从 4 字节 buffer 读出 float (LE) */
static float Comm_ReadFloatLe(const uint8_t *data)
{
    float value;
    uint8_t bytes[4];

    bytes[0] = data[0]; bytes[1] = data[1];
    bytes[2] = data[2]; bytes[3] = data[3];
    (void)memcpy(&value, bytes, sizeof(value));
    return value;
}

static uint32_t Comm_ReadU32Le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

#if !COMM_USB_TEST_FRAME_ENABLE
/* 将 float 写入 4 字节 buffer (LE) */
static void Comm_WriteFloatLe(uint8_t *data, float value)
{
    uint8_t bytes[4];
    (void)memcpy(bytes, &value, sizeof(value));
    data[0] = bytes[0]; data[1] = bytes[1];
    data[2] = bytes[2]; data[3] = bytes[3];
}
#endif

/* ==========================================================================
 * 帧解析
 * ========================================================================== */

/* 将一帧 46 字节解包为 11 个 float，存入 s_last_command */
static void Comm_UnpackCommand(const uint8_t packet[COMM_PACKET_SIZE])
{
    float *fields[COMM_FLOAT_COUNT] = {
        &s_last_command.vx,             &s_last_command.vy,
        &s_last_command.vw,             &s_last_command.front_lift,
        &s_last_command.rear_lift,      &s_last_command.rs_actuator_0,
        &s_last_command.rs_actuator_1,  &s_last_command.rs_actuator_2,
        &s_last_command.rs_actuator_3,  &s_last_command.rs_actuator_4,
        &s_last_command.rs_actuator_5,
    };

    for (uint8_t i = 0U; i < COMM_PACKET_SIZE; ++i) {
        g_comm_rx_last_packet[i] = packet[i];
    }

    for (uint8_t i = 0U; i < COMM_FLOAT_COUNT; ++i) {
        g_comm_rx_float_bits[i] = Comm_ReadU32Le(&packet[1U + i * 4U]);
        *fields[i] = Comm_ReadFloatLe(&packet[1U + i * 4U]);
    }

    s_has_command        = 1U;
    s_new_command_pending = 1U;
    g_comm_rx_valid_frame_count++;
    g_comm_rx_vx = s_last_command.vx;
    g_comm_rx_vy = s_last_command.vy;
    g_comm_rx_vw = s_last_command.vw;
    g_comm_rx_vx_bits = g_comm_rx_float_bits[0];
    g_comm_rx_vy_bits = g_comm_rx_float_bits[1];
    g_comm_rx_vw_bits = g_comm_rx_float_bits[2];
}

/* 从环形缓冲区取字节，按帧头帧尾动态定位一帧 */
static void Comm_ParseRx(void)
{
    uint8_t byte;

    while (Comm_RxPop(&byte) != 0U) {
        if (s_parse_index == 0U) {
            if (byte != COMM_CMD_HEAD) continue;  /* 不是帧头，跳过 */
        }

        s_parse_buffer[s_parse_index++] = byte;

        if (s_parse_index >= COMM_PACKET_SIZE) {
            /* 收满 46 字节: 验证帧头帧尾 */
            if (s_parse_buffer[0] == COMM_CMD_HEAD &&
                s_parse_buffer[COMM_PACKET_SIZE - 1U] == COMM_CMD_TAIL) {
                Comm_UnpackCommand(s_parse_buffer);
            }
            s_parse_index = 0U;  /* 继续找下一帧 */
        }
    }
}

/* ==========================================================================
 * 升降零点与指令执行
 * ========================================================================== */

/* 检查 4 个升降电机是否全部在线 */
static uint8_t Comm_LiftFeedbackReady(uint32_t now_ms)
{
    for (uint8_t motor_id = 1U; motor_id <= 4U; ++motor_id) {
        DjiMotorState *state = DjiMotor_GetState(motor_id);
        if (state == 0 || state->online == 0U ||
            (now_ms - state->last_update_ms) > LIFT_OFFLINE_TIMEOUT_MS) {
            return 0U;
        }
    }
    return 1U;
}

/* 首次在线时自动把当前位置设为零点 (只执行一次) */
static void Comm_ZeroLiftWhenReady(uint32_t now_ms)
{
    if (s_lift_zeroed != 0U || Comm_LiftFeedbackReady(now_ms) == 0U) return;

    Lift_SetZeroToCurrent(COMM_LIFT_PAIR_FRONT_A);
    Lift_SetZeroToCurrent(COMM_LIFT_PAIR_FRONT_B);
    Lift_SetZeroToCurrent(COMM_LIFT_PAIR_REAR_A);
    Lift_SetZeroToCurrent(COMM_LIFT_PAIR_REAR_B);
    s_lift_zeroed = 1U;
}

/* 将收到的命令分发到各控制模块 */
static void Comm_ApplyCommand(void)
{
    if (s_new_command_pending == 0U) return;//没有新命令，直接返回
    s_new_command_pending = 0U;

    /* 底盘: 上位机发送 vx/vy 单位 m/s，vw 单位 rad/s。
     * Chassis_SetVelocity() 内部会换算成 C620 反馈侧 rpm。 */
    g_comm_apply_count++;
    g_comm_apply_vx = s_last_command.vx;
    g_comm_apply_vy = s_last_command.vy;
    g_comm_apply_vw = s_last_command.vw;
    Chassis_SetVelocity(s_last_command.vx,
                        s_last_command.vy,
                        s_last_command.vw);

    /* 升降: 等首次在线 → 设零点 → 位置 PID */
    //Comm_ZeroLiftWhenReady(now_ms);
    if (s_lift_zeroed != 0U) {
        Lift_SetTargetPositionDeg(COMM_LIFT_PAIR_FRONT_A, s_last_command.front_lift);
        Lift_SetTargetPositionDeg(COMM_LIFT_PAIR_FRONT_B, s_last_command.front_lift);
        Lift_SetTargetPositionDeg(COMM_LIFT_PAIR_REAR_A,  s_last_command.rear_lift);
        Lift_SetTargetPositionDeg(COMM_LIFT_PAIR_REAR_B,  s_last_command.rear_lift);
    }

    /* RS actuator 0: generic RobStride position actuator.
     * The lower control layer does not care where the motor is mounted. */
    (void)RsActuator_SetTargetDeg(RS_ACTUATOR_0, s_last_command.rs_actuator_0);

    /* Other RS fields will map to more RsActuatorId entries after their
     * physical motor IDs are confirmed. */
}

/* ==========================================================================
 * 反馈发送
 * ========================================================================== */

#if !COMM_USB_TEST_FRAME_ENABLE
/* 打包反馈帧: 底盘速度 + 升降位置 */
static void Comm_PackFeedback(uint8_t packet[COMM_PACKET_SIZE])
{
    CommFrameFloats feedback;
    const float *fields[COMM_FLOAT_COUNT];

    /* 底盘速度暂用命令值回显 (里程计未实现) */
    feedback.vx = s_last_command.vx;
    feedback.vy = s_last_command.vy;
    feedback.vw = s_last_command.vw;

    /* 升降位置: 前后两组的平均角度 */
    if (s_lift_zeroed != 0U) {
        feedback.front_lift =
            (Lift_GetPositionDeg(COMM_LIFT_PAIR_FRONT_A) +
             Lift_GetPositionDeg(COMM_LIFT_PAIR_FRONT_B)) * 0.5f;
        feedback.rear_lift =
            (Lift_GetPositionDeg(COMM_LIFT_PAIR_REAR_A) +
             Lift_GetPositionDeg(COMM_LIFT_PAIR_REAR_B)) * 0.5f;
    } else {
        feedback.front_lift = 0.0f;
        feedback.rear_lift  = 0.0f;
    }

    /* Keep packet layout unchanged. Field 5 currently reports RS actuator 0. */
    feedback.rs_actuator_0 = RsActuator_GetPositionDeg(RS_ACTUATOR_0);
    feedback.rs_actuator_1 = 0.0f;
    feedback.rs_actuator_2 = 0.0f;
    feedback.rs_actuator_3 = 0.0f;
    feedback.rs_actuator_4 = 0.0f;
    feedback.rs_actuator_5 = 0.0f;

    fields[0]  = &feedback.vx;
    fields[1]  = &feedback.vy;
    fields[2]  = &feedback.vw;
    fields[3]  = &feedback.front_lift;
    fields[4]  = &feedback.rear_lift;
    fields[5]  = &feedback.rs_actuator_0;
    fields[6]  = &feedback.rs_actuator_1;
    fields[7]  = &feedback.rs_actuator_2;
    fields[8]  = &feedback.rs_actuator_3;
    fields[9]  = &feedback.rs_actuator_4;
    fields[10] = &feedback.rs_actuator_5;

    packet[0] = COMM_FEEDBACK_HEAD;
    for (uint8_t i = 0U; i < COMM_FLOAT_COUNT; ++i) {
        Comm_WriteFloatLe(&packet[1U + i * 4U], *fields[i]);
    }
    packet[COMM_PACKET_SIZE - 1U] = COMM_FEEDBACK_TAIL;
}
#endif

/* 定时发送反馈到 USB CDC */
static void Comm_SendFeedbackPeriodic(void)
{
    uint32_t now_ms = HAL_GetTick();

#if COMM_USB_TEST_FRAME_ENABLE
    /* 测试模式: 发送固定 46 字节帧 (头尾 0xAA/0x55, float 全 0) */
    if ((now_ms - s_last_feedback_ms) < COMM_USB_TEST_FRAME_PERIOD_MS) return;
    s_last_feedback_ms = now_ms;
    (void)CDC_Transmit_HS(s_test_frame, COMM_PACKET_SIZE);
#else
    /* 正式模式: 发送实时反馈帧 */
    if ((now_ms - s_last_feedback_ms) < COMM_FEEDBACK_PERIOD_MS) return;
    s_last_feedback_ms = now_ms;
    Comm_PackFeedback(s_feedback_packet);
    (void)CDC_Transmit_HS(s_feedback_packet, COMM_PACKET_SIZE);
#endif
}

/* ==========================================================================
 * 公开接口
 * ========================================================================== */

void Comm_Init(void)
{
    s_rx_head        = 0U;
    s_rx_tail        = 0U;
    s_parse_index    = 0U;
    s_has_command    = 0U;
    s_new_command_pending = 0U;
    s_lift_zeroed    = 0U;
    s_last_feedback_ms = 0U;
    g_comm_rx_valid_frame_count = 0U;
    g_comm_rx_vx = 0.0f;
    g_comm_rx_vy = 0.0f;
    g_comm_rx_vw = 0.0f;
    g_comm_rx_vx_bits = 0U;
    g_comm_rx_vy_bits = 0U;
    g_comm_rx_vw_bits = 0U;
    g_comm_apply_vx = 0.0f;
    g_comm_apply_vy = 0.0f;
    g_comm_apply_vw = 0.0f;
    g_comm_apply_count = 0U;
    for (uint8_t i = 0U; i < COMM_PACKET_SIZE; ++i) {
        g_comm_rx_last_packet[i] = 0U;
    }
    for (uint8_t i = 0U; i < COMM_FLOAT_COUNT; ++i) {
        g_comm_rx_float_bits[i] = 0U;
    }
    (void)memset(&s_last_command, 0, sizeof(s_last_command));
}

/* 主循环每圈调用: 解析 RX → 执行指令 → 发送反馈 */
void Comm_RunPeriodic(void)
{
    Comm_ParseRx();//解析收到的指令
    Comm_ZeroLiftWhenReady(HAL_GetTick());//检查升降电机是否在线，若在线则设零点
    Comm_ApplyCommand();
    Comm_SendFeedbackPeriodic();//发送反馈
}

/* USB CDC 接收到数据时回调 (中断上下文)，将数据推入环形缓冲区 */
void Comm_OnUsbReceived(const uint8_t *data, uint32_t len)
{
    if (data == 0) return;

    for (uint32_t i = 0U; i < len; ++i) {
        Comm_RxPush(data[i]);
    }
}

uint8_t Comm_HasCommand(void)
{
    return s_has_command;
}

const CommFrameFloats *Comm_GetLastCommand(void)
{
    return &s_last_command;
}
