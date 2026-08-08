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
 *   CH6 (两档开关)  → 遥控链路有效标志
 *   CH3, CH7..CH14  → 暂不使用
 *
 * 安全性：
 *   - CH6 离开 -100% / +100% 端点 → 底盘清零，升降保持，释放 USB 控制权
 *   -  2 s 无 RX 中断 → 重拉 USART10 接收
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

/* 两档开关 SWA（CH6）用作遥控链路有效标志。 */
#define RC_CH_LINK 5U

/* 归一化参数 */
#define RC_CHANNEL_MIN     1000.0f
#define RC_CHANNEL_CENTER  1500.0f
#define RC_CHANNEL_MAX     2000.0f
#define RC_CHANNEL_DEADBAND 0.06f

/* 底盘速度限幅（与 ChassisApp 内部限幅配合） */
#define RC_MAX_VX_M_S   0.5f
#define RC_MAX_VY_M_S   0.5f
#define RC_MAX_WZ_RAD_S 1.0f

/* 三档拨杆升降阈值（归一化值） */
#define RC_LIFT_UP_THRESHOLD   0.35f
#define RC_LIFT_DOWN_THRESHOLD (-0.35f)

/* CH6 只有接近 -100% / +100% 时才算在线。保留 10% 端点容差，
 * failsafe 设为 32% 时会稳定落在离线区间。 */
#define RC_LINK_ONLINE_THRESHOLD 0.90f

/* 升降离散位置步进参数（边沿触发，给 LiftApp PID 静态目标） */
#define RC_LIFT_STEP_M      0.05f  /* 每次拨杆推一下 = ±5 cm */
#define RC_LIFT_MAX_HEIGHT_M 0.30f /* 升降上限 */

/**
 * CH5 拨杆方向。若拨杆往上打升、往下打降，设置为 1.0f；
 * 若方向反了（往上打反而降），设置为 -1.0f。
 *
 * 如果这里是 -1.0f 还不对，说明遥控器里 CH5 通道反向了，
 * 进 FS-i6 菜单 Functions → Reverse → CH5 设为 Rev 即可。
 */
#define RC_LIFT_DIRECTION  -1.0f

/* 时序（ms） */
#define RC_RX_WATCHDOG_MS 2000U /* 硬件重拉间隔 */

/* ================================================================
 *  私有状态（全部模块静态）
 * ================================================================ */

/* -- IBUS 字节接收 -- */
static uint8_t  s_rx_byte;
static uint8_t  s_frame[RC_IBUS_FRAME_SIZE];
static uint8_t  s_frame_pos;

/* -- 解析后的通道值 -- */
/* ISR 发布区：以 s_valid_frame_count 作为帧完成标志。 */
static volatile uint16_t s_channels[RC_IBUS_CHANNEL_COUNT];
/* 主循环稳定快照：控制逻辑只读该数组。 */
static uint16_t s_active_channels[RC_IBUS_CHANNEL_COUNT];

/* -- 归一化后的控制量 -- */
static volatile float s_vx_m_s;
static volatile float s_vy_m_s;
static volatile float s_wz_rad_s;
static volatile float s_lift_position_m;
static int8_t s_lift_sw_state; /* CH5 上一次状态: 1=上, 0=回中, -1=下 */

/* -- 遥控器链路状态机 -- */
typedef enum {
    RC_LINK_OFFLINE = 0, /* RC 不写邮箱，USB 接管 */
    RC_LINK_ONLINE       /* RC 每轮覆盖底盘和升降 */
} rc_link_state_t;

static rc_link_state_t s_link_state = RC_LINK_OFFLINE;

/* -- 有效帧与看门狗 -- */
static uint32_t s_seen_valid_frame_count;
static uint32_t s_last_rx_irq_count;
static uint32_t s_last_watchdog_check_ms;

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
static uint8_t RcControl_IsLinkChannelOnline(void);
static void   RcControl_ProcessFrame(void);
static void   RcControl_PushByte(uint8_t data);
static uint32_t RcControl_SnapshotLatestFrame(void);
static void   RcControl_ApplyChannels(void);
static void   RcControl_EnterOffline(void);

/* ================================================================
 *  归一化
 * ================================================================ */

/**
 * 将 IBUS 原始 PWM 值（典型 1000~2000，中位 1500）归一化到 [-1, 1]，
 * 并施加 ±6% 死区。
 */
static float RcControl_NormalizeChannel(uint16_t value)
{
    /* IBUS 舵量通道的有效范围为 1000~2000。接收机 failsafe
     * 帧可能在高位携带标志；这类值不得被钳为满杆。 */
    if ((float)value < RC_CHANNEL_MIN ||
        (float)value > RC_CHANNEL_MAX) {
        return 0.0f;
    }

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

/**
 * 读取 CH5 三档拨杆当前状态。
 * @return 1=上拨, 0=回中, -1=下拨
 */
static int8_t RcControl_GetCh5State(void)
{
    float lift_sw = RcControl_NormalizeChannel(
        s_active_channels[RC_CH_LIFT]) * RC_LIFT_DIRECTION;

    if (lift_sw > RC_LIFT_UP_THRESHOLD) {
        return 1;
    } else if (lift_sw < RC_LIFT_DOWN_THRESHOLD) {
        return -1;
    }
    return 0;
}

/**
 * CH6 为两档开关，正常帧应位于 -100% 或 +100% 端点。
 * 接收机 failsafe 设为 32%，落入中间区间时视为发射机离线。
 */
static uint8_t RcControl_IsLinkChannelOnline(void)
{
    uint16_t link_raw = s_active_channels[RC_CH_LINK];
    float link_value;

    /* 先严格拒绝越界原始值，避免带高位 failsafe 标志的
     * 通道被 NormalizeChannel 误当成 +/-100% 端点。 */
    if ((float)link_raw < RC_CHANNEL_MIN ||
        (float)link_raw > RC_CHANNEL_MAX) {
        return 0U;
    }

    link_value = RcControl_NormalizeChannel(link_raw);

    return (link_value <= -RC_LINK_ONLINE_THRESHOLD ||
            link_value >= RC_LINK_ONLINE_THRESHOLD)
               ? 1U
               : 0U;
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

    /* 全部校验通过才发布。计数器最后写入，作为
     * 主循环判断通道数组已完整更新的标志。 */
    for (i = 0U; i < RC_IBUS_CHANNEL_COUNT; i++) {
        s_channels[i] = channels[i];
    }
    __DMB();
    s_valid_frame_count++;
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
 * 从 ISR 发布区读取一份稳定快照。若读取期间新帧到达，
 * 帧计数会变化，主循环重读直到前后计数一致。
 *
 * @return 有效帧计数。
 */
static uint32_t RcControl_SnapshotLatestFrame(void)
{
    uint32_t count_before;
    uint32_t count_after;
    uint8_t  i;

    do {
        count_before = s_valid_frame_count;
        __DMB();
        for (i = 0U; i < RC_IBUS_CHANNEL_COUNT; i++) {
            s_active_channels[i] = s_channels[i];
        }
        __DMB();
        count_after = s_valid_frame_count;
    } while (count_before != count_after);

    return count_after;
}

/**
 * 从外部数据源同步升降累加器位置。
 *
 * 优先级：
 *   1. 电机反馈（g_comm_app_feedback.lift_front_position_m）—— 物理实际位置
 *   2. 上位机指令（g_comm_app_command.lift_front_position_m）—— 最近一次 USB 目标
 *   3. 保持当前 s_lift_position_m（无有效外部源）
 *
 * 同步后钳位到 [0, RC_LIFT_MAX_HEIGHT_M] 并对齐到最近的步进网格，
 * 使后续 CH5 边沿触发行为可预测。
 */
static void RcControl_SyncLiftFromExternal(void)
{
    float src_m = s_lift_position_m; /* 保底：维持当前值 */

    if (g_comm_app_feedback.lift_valid != 0U) {
        src_m = g_comm_app_feedback.lift_front_position_m;
    }
    /* 不回退到 g_comm_app_command：Jetson 可能持续发送全零帧，
     * 回退会导致升降位置被错误同步为 0。无有效反馈时保持当前值。 */

    /* 钳位 */
    if (src_m < 0.0f) {
        src_m = 0.0f;
    }
    if (src_m > RC_LIFT_MAX_HEIGHT_M) {
        src_m = RC_LIFT_MAX_HEIGHT_M;
    }

    /* 对齐到最近的离散步进网格 */
    {
        int32_t steps = (int32_t)((src_m / RC_LIFT_STEP_M) + 0.5f);
        s_lift_position_m = (float)steps * RC_LIFT_STEP_M;
    }

    /* 防 float 边界情况二次钳位 */
    if (s_lift_position_m < 0.0f) {
        s_lift_position_m = 0.0f;
    }
    if (s_lift_position_m > RC_LIFT_MAX_HEIGHT_M) {
        s_lift_position_m = RC_LIFT_MAX_HEIGHT_M;
    }
}

/**
 * 将归一化后的通道值写入 g_comm_app_command。
 * 主循环上下文调用。
 */
static void RcControl_ApplyChannels(void)
{
    float right_x  = RcControl_NormalizeChannel(
        s_active_channels[RC_CH_RIGHT_X]);
    float right_y  = RcControl_NormalizeChannel(
        s_active_channels[RC_CH_RIGHT_Y]);
    float left_x   = RcControl_NormalizeChannel(
        s_active_channels[RC_CH_LEFT_X]);
    int8_t sw_state = RcControl_GetCh5State();

    /* ---- 底盘 ---- */
    s_vx_m_s    = right_x * RC_MAX_VX_M_S;
    s_vy_m_s    = right_y * RC_MAX_VY_M_S;
    s_wz_rad_s  = -left_x * RC_MAX_WZ_RAD_S;

    /* ---- 升降：边沿触发离散位置步进 ---- */

    /* 只在 回中→上 或 回中→下 的边沿触发一次步进。
     * 拨杆保持不动 → 目标不变 → LiftApp PID 可达稳态 → 不抖。 */
    if (s_lift_sw_state == 0 && sw_state == 1) {
        s_lift_position_m += RC_LIFT_STEP_M;
    } else if (s_lift_sw_state == 0 && sw_state == -1) {
        s_lift_position_m -= RC_LIFT_STEP_M;
    }
    s_lift_sw_state = sw_state;

    /* 钳位 */
    if (s_lift_position_m < 0.0f) {
        s_lift_position_m = 0.0f;
    }
    if (s_lift_position_m > RC_LIFT_MAX_HEIGHT_M) {
        s_lift_position_m = RC_LIFT_MAX_HEIGHT_M;
    }

    /* ---- 写入全局命令邮箱 ---- */
    g_comm_app_command.chassis_vx_m_s   = s_vx_m_s;
    g_comm_app_command.chassis_vy_m_s   = s_vy_m_s;
    g_comm_app_command.chassis_wz_rad_s = s_wz_rad_s;

    /* 升降每轮写入，跟底盘一样。RC ONLINE 期间上位机全零帧被覆盖。 */
    g_comm_app_command.lift_front_position_m =
        s_lift_position_m;
    g_comm_app_command.lift_rear_position_m =
        s_lift_position_m;
    /* 其它轴（KFS、武器等）保持不动：RC 不写，上位机独享 */

    __DMB();
    g_comm_app_command.sequence++;
    g_comm_app_command.valid = 1U;
}

/** 进入离线状态：底盘立即清零，升降目标保持，不再覆盖 USB。 */
static void RcControl_EnterOffline(void)
{
    s_vx_m_s = 0.0f;
    s_vy_m_s = 0.0f;
    s_wz_rad_s = 0.0f;

    g_comm_app_command.chassis_vx_m_s = 0.0f;
    g_comm_app_command.chassis_vy_m_s = 0.0f;
    g_comm_app_command.chassis_wz_rad_s = 0.0f;
    s_link_state = RC_LINK_OFFLINE;
}

/* ================================================================
 *  公开 API
 * ================================================================ */

void RcControl_Init(void)
{
    uint8_t i;

    s_frame_pos              = 0U;
    s_seen_valid_frame_count = 0U;
    s_last_watchdog_check_ms = 0U;
    s_last_rx_irq_count      = 0U;
    s_vx_m_s                 = 0.0f;
    s_vy_m_s                 = 0.0f;
    s_wz_rad_s               = 0.0f;
    s_lift_position_m        = 0.0f;
    s_lift_sw_state          = 0;
    s_link_state             = RC_LINK_OFFLINE;

    for (i = 0U; i < RC_IBUS_CHANNEL_COUNT; i++) {
        s_channels[i]        = 0U;
        s_active_channels[i] = 0U;
    }

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
    uint32_t frame_count = RcControl_SnapshotLatestFrame();
    uint32_t now_ms      = HAL_GetTick();
    uint8_t  new_valid_frame =
        (frame_count != s_seen_valid_frame_count) ? 1U : 0U;

    if (new_valid_frame != 0U) {
        s_seen_valid_frame_count = frame_count;
    }

    /* ---- RX 硬件看门狗（2 s 无 IRQ → 重拉） ---- */
    {
        uint32_t wd_elapsed =
            (uint32_t)(now_ms - s_last_watchdog_check_ms);

        if (wd_elapsed >= RC_RX_WATCHDOG_MS) {
            if (s_rx_irq_count == s_last_rx_irq_count) {
                s_frame_pos = 0U;
                (void)HAL_UART_AbortReceive_IT(&huart10);
                __HAL_UART_CLEAR_PEFLAG(&huart10);
                (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte,
                                          1U);
            }
            s_last_watchdog_check_ms = now_ms;
            s_last_rx_irq_count      = s_rx_irq_count;
        }
    }

    /* ---- 遥控器链路状态机 ---- */
    if (s_link_state == RC_LINK_OFFLINE) {
        if (new_valid_frame != 0U &&
            RcControl_IsLinkChannelOnline() != 0U) {
            /* 首帧/恢复帧到达：以实际升降位置重建累加器，
             * 并同步 CH5 状态，避免上线时产生虚假边沿。 */
            RcControl_SyncLiftFromExternal();
            s_lift_sw_state = RcControl_GetCh5State();
            s_link_state    = RC_LINK_ONLINE;
            RcControl_ApplyChannels();
        }
    } else { /* RC_LINK_ONLINE */
        if (RcControl_IsLinkChannelOnline() == 0U) {
            /* CH6 进入 failsafe 中间区间。 */
            RcControl_EnterOffline();
        } else {
            /* RC 在 CommApp 之后运行，在线时每轮覆盖 USB 底盘/升降。 */
            RcControl_ApplyChannels();
        }
    }
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
