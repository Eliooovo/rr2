/**
 * @file    rc_control.c
 * @brief   FS-i6 IBUS 协议解析、通道映射与超时保护。
 *
 * 参考实现：Rscontrol2/Core/Src/fsi6_thread.c
 *
 * IBUS 帧格式（共 32 字节）：
 *   [0]     = 0x20  帧头
 *   [1]     = 0x40
 *   [2..29] = 14 通道 × uint16 LE (CH1..CH14)
 *   [30..31]= checksum = 0xFFFF - sum(bytes[0..29])，uint16 LE
 *
 * 通道映射：
 *   CH1 (右摇杆 X)  → vy (横向平移)
 *   CH2 (右摇杆 Y)  → vx (前后平移)
 *   CH4 (左摇杆 X)  → wz (旋转)
 *   CH5 (三档拨杆)  → 升降位置增量控制
 *   CH3, CH6..CH14  → 暂不使用
 *
 * 安全性：
 *   - 100 ms 无有效帧 → 底盘速度清零，升降保持，释放 USB 控制权
 *   -   2 s 无 RX 中断且从未收到有效帧 → 重拉 USART10 接收
 */

#include "rc_control.h"

#include "comm_app.h"
#include "main.h"
#include "usart.h"

/* ================================================================
 *  可配置常量
 * ================================================================ */

/* IBUS 协议 */
#define RC_IBUS_FRAME_SIZE    32U
#define RC_IBUS_HEADER0       0x20U
#define RC_IBUS_HEADER1       0x40U
#define RC_IBUS_CHANNEL_COUNT 14U

/* 通道索引（0-based，对应 IBUS 帧中通道顺序） */
#define RC_CH_RIGHT_X 0U /* CH1 - 右摇杆水平 */
#define RC_CH_RIGHT_Y 1U /* CH2 - 右摇杆垂直 */
#define RC_CH_LEFT_X  3U /* CH4 - 左摇杆水平 */

/* 三档拨杆 SWC（CH5）用于升降 */
#define RC_CH_LIFT 4U

/* 归一化参数 */
#define RC_CHANNEL_MIN     1000.0f
#define RC_CHANNEL_CENTER  1500.0f
#define RC_CHANNEL_MAX     2000.0f
#define RC_CHANNEL_DEADBAND 0.06f

/* 底盘速度限幅（与 ChassisApp 内部限幅配合） */
#define RC_MAX_VX_M_S   1.0f
#define RC_MAX_VY_M_S   1.0f
#define RC_MAX_WZ_RAD_S 3.0f

/* 三档拨杆升降阈值（归一化值） */
#define RC_LIFT_UP_THRESHOLD   0.35f
#define RC_LIFT_DOWN_THRESHOLD (-0.35f)

/* 升降增量式控制参数 */
#define RC_LIFT_RATE_M_S     0.05f  /* 拨杆推到底时升降速率 */
#define RC_LIFT_MAX_HEIGHT_M 0.30f  /* 升降上限 */

/**
 * CH5 拨杆方向。若拨杆往上打升、往下打降，设置为 1.0f；
 * 若方向反了（往上打反而降），设置为 -1.0f。
 *
 * 如果这里是 -1.0f 还不对，说明遥控器里 CH5 通道反向了，
 * 进 FS-i6 菜单 Functions → Reverse → CH5 设为 Rev 即可。
 */
#define RC_LIFT_DIRECTION  -1.0f

/* 时序（ms） */
#define RC_CONTROL_PERIOD_MS 10U  /* 控制写入周期 */
#define RC_FAILSAFE_TIMEOUT_MS 100U /* 信号丢失超时 */
#define RC_RX_WATCHDOG_MS      2000U /* 硬件重拉间隔 */

/* ================================================================
 *  私有状态（全部模块静态）
 * ================================================================ */

/* -- IBUS 字节接收 -- */
static uint8_t  s_rx_byte;
static uint8_t  s_frame[RC_IBUS_FRAME_SIZE];
static uint8_t  s_frame_pos;

/* -- 解析后的通道值（ISR 写入，主循环读取） -- */
static volatile uint16_t s_channels[RC_IBUS_CHANNEL_COUNT];

/* -- 归一化后的控制量 -- */
static volatile float s_vx_m_s;
static volatile float s_vy_m_s;
static volatile float s_wz_rad_s;
static volatile float s_lift_position_m;

/* -- 时间戳与看门狗 -- */
/**
 * 最后一次收到有效 IBUS 帧的时刻（HAL_GetTick()）。
 *
 * 【重要】此变量由 ISR（RcControl_ProcessFrame）和主循环
 * （RcControl_ApplyChannels）共同写入：
 *   - ISR 收到有效帧时立即打时间戳，这是"信号有效"的唯一标志位。
 *   - 主循环 ApplyChannels 再次刷新，防止 100ms 超时误触发。
 *
 * 不能只在 ApplyChannels 里设置：因为 s_last_valid_ms 初始为 0，
 * RunPeriodic 判断 s_last_valid_ms != 0 才会调用 ApplyChannels。
 * 若 ISR 不设值 → ApplyChannels 永远进不去 → 鸡生蛋蛋生鸡死锁。
 */
static uint32_t s_last_valid_ms;
static uint32_t s_last_control_ms;
static uint32_t s_last_rx_irq_count;
static uint32_t s_last_watchdog_check_ms;
static volatile uint8_t s_ever_valid;

/* -- 调试计数器（volatile 便于调试器查看） -- */
static volatile uint32_t s_rx_irq_count;
static volatile uint32_t s_valid_frame_count;
static volatile uint32_t s_checksum_error_count;
static volatile uint32_t s_sync_error_count;
static volatile uint32_t s_uart_error_count;

/* ================================================================
 *  前向声明
 * ================================================================ */

static float  RcControl_NormalizeChannel(uint16_t value);
static void   RcControl_ProcessFrame(void);
static void   RcControl_PushByte(uint8_t data);
static void   RcControl_ApplyChannels(uint32_t now_ms,
                                      uint32_t elapsed_ms);

/* ================================================================
 *  归一化
 * ================================================================ */

/**
 * 将 IBUS 原始 PWM 值（典型 1000~2000，中位 1500）归一化到 [-1, 1]，
 * 并施加 ±6% 死区。
 */
static float RcControl_NormalizeChannel(uint16_t value)
{
    float normalized =
        ((float)value - RC_CHANNEL_CENTER) /
        ((RC_CHANNEL_MAX - RC_CHANNEL_MIN) * 0.5f);

    if (normalized > 1.0f) {
        normalized = 1.0f;
    } else if (normalized < -1.0f) {
        normalized = -1.0f;
    }

    if (normalized < RC_CHANNEL_DEADBAND &&
        normalized > -RC_CHANNEL_DEADBAND) {
        normalized = 0.0f;
    }

    return normalized;
}

/* ================================================================
 *  IBUS 帧校验与解析（ISR 上下文）
 * ================================================================ */

/**
 * 校验并解析一帧完整 IBUS 数据。
 * ISR 上下文调用：只做校验和通道解析，不操控 g_comm_app_command。
 */
static void RcControl_ProcessFrame(void)
{
    uint16_t received_checksum;
    uint16_t checksum;
    uint16_t channels[RC_IBUS_CHANNEL_COUNT];
    uint8_t  i;

    /* 解析帧尾 checksum（LE） */
    received_checksum =
        (uint16_t)(s_frame[31] << 8) | s_frame[30];

    /* 二次确认帧头 */
    if (s_frame[0] != RC_IBUS_HEADER0 ||
        s_frame[1] != RC_IBUS_HEADER1) {
        s_sync_error_count++;
        return;
    }

    /* 计算 IBUS checksum：0xFFFF - sum(bytes[0..29]) */
    checksum = 0xFFFFU;
    for (i = 0U; i < RC_IBUS_FRAME_SIZE - 2U; i++) {
        checksum = (uint16_t)(checksum - s_frame[i]);
    }

    if (checksum != received_checksum) {
        s_checksum_error_count++;
        return;
    }

    /* 提取 14 通道（LE uint16，从偏移 2 开始） */
    for (i = 0U; i < RC_IBUS_CHANNEL_COUNT; i++) {
        channels[i] =
            (uint16_t)(s_frame[2U + (uint16_t)i * 2U]) |
            (uint16_t)((uint16_t)s_frame[3U + (uint16_t)i * 2U] << 8);
    }

    /* 全部校验通过才写入全局数组 */
    for (i = 0U; i < RC_IBUS_CHANNEL_COUNT; i++) {
        s_channels[i] = channels[i];
    }
    __DMB();
    s_valid_frame_count++;

    /* 标记有效帧时间戳，使主循环能进入 ApplyChannels。
     * 必须在 ISR 中设置，因为 ApplyChannels 以此判断是否有有效信号。 */
    s_last_valid_ms = HAL_GetTick();
    s_ever_valid    = 1U;
}

/**
 * IBUS 字节状态机：帧同步 + 收完 32 字节后调用 RcControl_ProcessFrame。
 * ISR 上下文调用。
 */
static void RcControl_PushByte(uint8_t data)
{
    /* 等待第一个帧头 */
    if (s_frame_pos == 0U) {
        if (data != RC_IBUS_HEADER0) {
            s_sync_error_count++;
            return;
        }
        s_frame[s_frame_pos++] = data;
        return;
    }

    /* 校验第二个帧头 */
    if (s_frame_pos == 1U) {
        if (data != RC_IBUS_HEADER1) {
            s_frame_pos = 0U;
            s_sync_error_count++;
            /* 支持重同步：如果刚好是下一个帧的帧头，直接开始新帧 */
            if (data == RC_IBUS_HEADER0) {
                s_frame[s_frame_pos++] = data;
            }
            return;
        }
        s_frame[s_frame_pos++] = data;
        return;
    }

    /* 收剩余字节 */
    s_frame[s_frame_pos++] = data;

    if (s_frame_pos >= RC_IBUS_FRAME_SIZE) {
        s_frame_pos = 0U;
        RcControl_ProcessFrame();
    }
}

/* ================================================================
 *  通道映射与应用（主循环上下文）
 * ================================================================ */

/**
 * 将归一化后的通道值写入 g_comm_app_command。
 * 主循环上下文调用。
 *
 * @param now_ms      HAL_GetTick() 当前值
 * @param elapsed_ms  距上次 ApplyChannels 的毫秒数，用于 dt 缩放
 */
static void RcControl_ApplyChannels(uint32_t now_ms,
                                    uint32_t elapsed_ms)
{
    float right_x  = RcControl_NormalizeChannel(
        s_channels[RC_CH_RIGHT_X]);
    float right_y  = RcControl_NormalizeChannel(
        s_channels[RC_CH_RIGHT_Y]);
    float left_x   = RcControl_NormalizeChannel(
        s_channels[RC_CH_LEFT_X]);
    float lift_sw  = RcControl_NormalizeChannel(
        s_channels[RC_CH_LIFT]) * RC_LIFT_DIRECTION;
    float dt_s     = (float)elapsed_ms * 0.001f;

    /* ---- 底盘 ---- */
    s_vx_m_s    = right_y * RC_MAX_VX_M_S;
    s_vy_m_s    = right_x * RC_MAX_VY_M_S;
    s_wz_rad_s  = -left_x * RC_MAX_WZ_RAD_S;

    /* ---- 升降（增量累积 + dt 缩放） ---- */
    if (lift_sw > RC_LIFT_UP_THRESHOLD) {
        s_lift_position_m += RC_LIFT_RATE_M_S * dt_s;
    } else if (lift_sw < RC_LIFT_DOWN_THRESHOLD) {
        s_lift_position_m -= RC_LIFT_RATE_M_S * dt_s;
    }
    /* 钳位 */
    if (s_lift_position_m < 0.0f) {
        s_lift_position_m = 0.0f;
    }
    if (s_lift_position_m > RC_LIFT_MAX_HEIGHT_M) {
        s_lift_position_m = RC_LIFT_MAX_HEIGHT_M;
    }

    /* ---- 写入全局命令邮箱 ---- */
    g_comm_app_command.chassis_vx_m_s        = s_vx_m_s;
    g_comm_app_command.chassis_vy_m_s        = s_vy_m_s;
    g_comm_app_command.chassis_wz_rad_s      = s_wz_rad_s;
    g_comm_app_command.lift_front_position_m = s_lift_position_m;
    g_comm_app_command.lift_rear_position_m  = s_lift_position_m;
    /* 其它轴（KFS、武器等）保持不动：不再写入，由上一次 USB 指令保留 */

    __DMB();
    g_comm_app_command.sequence++;
    g_comm_app_command.valid = 1U;

    s_last_valid_ms = now_ms;
    s_ever_valid    = 1U;

    (void)now_ms;
}

/* ================================================================
 *  公开 API
 * ================================================================ */

void RcControl_Init(void)
{
    s_frame_pos              = 0U;
    s_last_valid_ms          = 0U;
    s_last_control_ms        = 0U;
    s_last_watchdog_check_ms = 0U;
    s_last_rx_irq_count      = 0U;
    s_vx_m_s                 = 0.0f;
    s_vy_m_s                 = 0.0f;
    s_wz_rad_s               = 0.0f;
    s_lift_position_m        = 0.0f;
    s_ever_valid             = 0U;

    /* 清空各计数器和诊断变量 */
    s_rx_irq_count        = 0U;
    s_valid_frame_count   = 0U;
    s_checksum_error_count = 0U;
    s_sync_error_count    = 0U;
    s_uart_error_count    = 0U;

    /* 启动 USART10 单字节中断接收 */
    (void)HAL_UART_AbortReceive_IT(&huart10);
    __HAL_UART_CLEAR_PEFLAG(&huart10);

    if (HAL_UART_Receive_IT(&huart10, &s_rx_byte, 1U) != HAL_OK) {
        /* 第一次启动失败，重试一次 */
        (void)HAL_UART_AbortReceive_IT(&huart10);
        __HAL_UART_CLEAR_PEFLAG(&huart10);
        HAL_Delay(1);
        (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte, 1U);
    }
}

void RcControl_RunPeriodic(void)
{
    uint32_t now_ms     = HAL_GetTick();
    uint32_t elapsed_ms;

    /* ---- 控制周期限速 ---- */
    if (s_last_control_ms != 0U) {
        uint32_t diff = (uint32_t)(now_ms - s_last_control_ms);
        if (diff < RC_CONTROL_PERIOD_MS) {
            return;
        }
    }
    elapsed_ms = (s_last_control_ms != 0U)
                     ? (uint32_t)(now_ms - s_last_control_ms)
                     : RC_CONTROL_PERIOD_MS;
    s_last_control_ms = now_ms;

    /* ---- RX 硬件看门狗（2 s 无 IRQ 且从未收到有效帧 → 重拉） ---- */
    {
        uint32_t wd_elapsed =
            (uint32_t)(now_ms - s_last_watchdog_check_ms);

        if (wd_elapsed > RC_RX_WATCHDOG_MS) {
            if (s_rx_irq_count == s_last_rx_irq_count &&
                s_ever_valid == 0U) {
                (void)HAL_UART_AbortReceive_IT(&huart10);
                __HAL_UART_CLEAR_PEFLAG(&huart10);
                (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte,
                                          1U);
            }
            s_last_watchdog_check_ms = now_ms;
            s_last_rx_irq_count      = s_rx_irq_count;
        }
    }

    /* ---- 信号超时检测（100 ms） ---- */
    if (s_last_valid_ms != 0U) {
        uint32_t elapsed =
            (uint32_t)(now_ms - s_last_valid_ms);

        if (elapsed > RC_FAILSAFE_TIMEOUT_MS) {
            s_last_valid_ms = 0U;
            s_vx_m_s        = 0.0f;
            s_vy_m_s        = 0.0f;
            s_wz_rad_s      = 0.0f;

            g_comm_app_command.chassis_vx_m_s   = 0.0f;
            g_comm_app_command.chassis_vy_m_s   = 0.0f;
            g_comm_app_command.chassis_wz_rad_s = 0.0f;
            g_comm_app_command.valid            = 0U;
            /* 升降位置保持不变（不在超时时清零，避免突然坠落） */
        }
    }

    /* ---- 有效信号则更新命令 ---- */
    if (s_last_valid_ms != 0U) {
        RcControl_ApplyChannels(now_ms, elapsed_ms);
    }
}

uint8_t RcControl_IsActive(void)
{
    if (s_last_valid_ms == 0U) {
        return 0U;
    }

    /* 双重确认：距上次有效帧不超过超时 */
    if ((uint32_t)(HAL_GetTick() - s_last_valid_ms) >
        RC_FAILSAFE_TIMEOUT_MS) {
        return 0U;
    }

    return 1U;
}

/* ================================================================
 *  HAL UART 回调（覆盖 weak 实现）
 *
 * 当前项目中：
 *   - USART1 有 IRQ 但无人调用 HAL_UART_Receive_IT，回调不会触发
 *   - UART7 使用阻塞模式，不产生中断回调
 *   - USART10 是唯一的中断接收 UART
 *
 * 日后若新增其他 UART 中断接收源，在此函数内加 else-if 分发即可。
 * ================================================================ */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10) {
        s_rx_irq_count++;
        RcControl_PushByte(s_rx_byte);
        /* 重拉下一次单字节接收 */
        (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10) {
        s_uart_error_count++;
        s_frame_pos = 0U;
        (void)HAL_UART_AbortReceive_IT(&huart10);
        __HAL_UART_CLEAR_PEFLAG(&huart10);
        (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte, 1U);
    }
}
