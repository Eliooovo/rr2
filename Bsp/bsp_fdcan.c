/**
 * @file    bsp_fdcan.c
 * @brief   FDCAN 板级支持包实现
 *
 * 为 DJI C620 电调 CAN 通信提供:
 *   - CAN 初始化与滤波器配置
 *   - CAN 帧收发 (基于 HAL FDCAN 驱动)
 *   - 接收中断回调 → DJI 电机驱动层
 *
 * DJI C620 电调 CAN 协议参数:
 *   波特率:   1 Mbps (经典 CAN, 非 FD)
 *   反馈频率: 1 kHz (电调自动上报，无需 MCU 请求)
 *   反馈 ID:  0x201~0x208
 *   命令 ID:  0x200, 0x1FF
 *   帧格式:   标准帧 (11 位 ID), 数据帧, 每帧 8 字节
 */

#include "bsp_fdcan.h"
#include "dji_motor.h"   /* DJI 电机驱动, 解析 CAN 反馈帧 */
#include "rs_motor.h"    /* 灵足电机驱动, 解析 FDCAN3 上的 MIT 反馈 */

volatile uint32_t g_fdcan1_hal_rx_callback_count = 0U;

volatile uint32_t g_fdcan1_rx_callback_count = 0U;
volatile uint32_t g_fdcan2_rx_callback_count = 0U;
volatile uint32_t g_fdcan3_rx_callback_count = 0U;

volatile uint32_t g_fdcan1_receive_ok_count = 0U;
volatile uint32_t g_fdcan1_receive_len = 0U;
volatile uint32_t g_fdcan1_last_id = 0U;
volatile uint32_t g_fdcan1_dji_feedback_count = 0U;
volatile uint32_t g_fdcan1_non_dji_count = 0U;
volatile uint32_t g_fdcan1_send_ok_count = 0U;
volatile uint32_t g_fdcan2_send_ok_count = 0U;
volatile uint32_t g_fdcan3_send_ok_count = 0U;

volatile uint32_t g_fdcan1_send_fail_count = 0U;
volatile uint32_t g_fdcan1_tx_fifo_free_level = 0U;
volatile uint32_t g_fdcan3_tx_fifo_free_level = 0U;
volatile uint32_t g_fdcan1_hal_error = 0U;
volatile uint32_t g_fdcan3_hal_error = 0U;
volatile uint32_t g_fdcan1_last_error_code = 0U;
volatile uint32_t g_fdcan3_last_error_code = 0U;

volatile uint32_t g_fdcan1_error_passive = 0U;
volatile uint32_t g_fdcan3_error_passive = 0U;

volatile uint32_t g_fdcan1_warning = 0U;
volatile uint32_t g_fdcan3_warning = 0U;

volatile uint32_t g_fdcan1_bus_off = 0U;
volatile uint32_t g_fdcan3_bus_off = 0U;

volatile uint32_t g_fdcan1_tx_error_count = 0U;
volatile uint32_t g_fdcan3_tx_error_count = 0U;

volatile uint32_t g_fdcan1_rx_error_count = 0U;
volatile uint32_t g_fdcan3_rx_error_count = 0U;

volatile uint32_t g_fdcan3_receive_ok_count = 0U;
volatile uint32_t g_fdcan3_receive_len = 0U;
volatile uint32_t g_fdcan3_last_id = 0U;
volatile uint32_t g_fdcan3_last_is_extended = 0U;
volatile uint32_t g_fdcan3_rs_feedback_count = 0U;
volatile uint32_t g_fdcan3_non_rs_count = 0U;

static void fdcan1_update_debug_status(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status;
    FDCAN_ErrorCountersTypeDef error_counters;

    g_fdcan1_tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    g_fdcan1_hal_error = HAL_FDCAN_GetError(&hfdcan1);

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
        g_fdcan1_last_error_code = protocol_status.LastErrorCode;
        g_fdcan1_error_passive = protocol_status.ErrorPassive;
        g_fdcan1_warning = protocol_status.Warning;
        g_fdcan1_bus_off = protocol_status.BusOff;
    }

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
        g_fdcan1_tx_error_count = error_counters.TxErrorCnt;
        g_fdcan1_rx_error_count = error_counters.RxErrorCnt;
    }
}
//函数用于更新 FDCAN3 的调试状态，包括获取 TX FIFO 空闲级别、错误状态、协议状态和错误计数器等信息，并将其存储在全局变量中，以便调试和监控 FDCAN3 的运行状态。
static void fdcan3_update_debug_status(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status;
    FDCAN_ErrorCountersTypeDef error_counters;

    g_fdcan3_tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3);
    g_fdcan3_hal_error = HAL_FDCAN_GetError(&hfdcan3);

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan3, &protocol_status) == HAL_OK) {
        g_fdcan3_last_error_code = protocol_status.LastErrorCode;
        g_fdcan3_error_passive = protocol_status.ErrorPassive;
        g_fdcan3_warning = protocol_status.Warning;
        g_fdcan3_bus_off = protocol_status.BusOff;
    }
    // 获取 FDCAN3 的错误计数器信息，包括发送错误计数和接收错误计数，并将其存储在全局变量中，以便调试和监控 FDCAN3 的运行状态。
    if (HAL_FDCAN_GetErrorCounters(&hfdcan3, &error_counters) == HAL_OK) {
        g_fdcan3_tx_error_count = error_counters.TxErrorCnt;
        g_fdcan3_rx_error_count = error_counters.RxErrorCnt;
    }
}
/* ==========================================================================
 * CAN 初始化
 * ========================================================================== */

/**
 * @brief  启动所有 CAN 总线
 *
 * 调用顺序:
 *   1. can_filter_init() — 配置接收滤波器
 *   2. HAL_FDCAN_Start() — 启动 FDCAN 外设 (FDAN1/2/3)
 *   3. HAL_FDCAN_ActivateNotification() — 使能 RX FIFO0 新消息中断
 *
 * 之后每当有 CAN 帧到达，HAL 会自动调用 HAL_FDCAN_RxFifo0Callback()。
 */
void bsp_can_init(void)
{
    can_filter_init();

    /* 启动 3 路 FDCAN 外设 */
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);

    /* 使能 RX FIFO0 中断通知: 每收到一帧触发一次 HAL_FDCAN_RxFifo0Callback */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief  配置 CAN 接收滤波器
 *
 * 当前策略: 全通滤波器 (不过滤任何 ID)
 *   - FilterID1 = 0x00, FilterID2 = 0x00 即 Mask=0 全通
 *   - 所有 CAN 帧都接收，由上层软件按 ID 判断是否处理
 *   - 全局过滤: 拒绝不匹配的标准/扩展 ID 及远程帧
 *
 * 配置项:
 *   IdType       = STANDARD_ID          11 位标准帧
 *   FilterType   = FILTER_MASK          掩码模式
 *   FilterConfig = FILTER_TO_RXFIFO0    匹配的帧放入 RX FIFO0
 *   Watermark    = 1                    每收到 1 帧就触发中断
 */
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType       = FDCAN_STANDARD_ID;              /* 只处理标准帧 (11 位 ID) */
    fdcan_filter.FilterIndex  = 0;                              /* 使用滤波器 0 */
    fdcan_filter.FilterType   = FDCAN_FILTER_MASK;              /* 掩码模式: ID & Mask == FilterID */
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;        /* 匹配的帧存入 RX FIFO0 */
    fdcan_filter.FilterID1    = 0x00;                           /* 掩码全 0 → 不过滤，所有 ID 都接收 */
    fdcan_filter.FilterID2    = 0x00;

    /* 三路 CAN 都配置相同的滤波器 */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
    HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);

    /* FDCAN3 接灵足电机。灵足默认私有协议使用 29 位扩展帧，所以这里
     * 给 FDCAN3 额外配置一个扩展帧全通滤波器。 */
    fdcan_filter.IdType       = FDCAN_EXTENDED_ID;
    fdcan_filter.FilterIndex  = 0;
    fdcan_filter.FilterType   = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1    = 0x00000000U;
    fdcan_filter.FilterID2    = 0x00000000U;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);

    /*
     * 全局过滤配置:
     *   拒绝不匹配的标准 ID 和扩展 ID 的帧
     *   拒绝远程帧 (C620 不使用远程帧)
     *   注: 由于上面滤波器是全通，实际所有标准数据帧都会被接收
     */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /*
     * RX FIFO0 水位线设为 1:
     *   每收到 1 帧就触发 FDCAN_IT_RX_FIFO0_NEW_MESSAGE 中断
     *   C620 以 1kHz 上报，所以中断频率 ≈1kHz/FDCAN 总线
     */
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);

    /* 以下为保留配置 (暂时不用) */
    // HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}

/* ==========================================================================
 * CAN 帧发送
 * ========================================================================== */

/**
 * @brief  通过 FDCAN 发送一帧标准 CAN 数据
 *
 * 封装 HAL 的发送流程，配置经典 CAN 帧参数。
 *
 * @param  hfdcan  FDCAN 句柄 (&hfdcan1~3)
 * @param  id      11 位标准帧 ID
 * @param  data    数据缓冲区
 * @param  len     数据长度 (字节)，必须 ≤ 8
 * @return  0=发送成功, 1=发送失败 (长度 >8 或 TX FIFO 满)
 *
 * 帧参数:
 *   - 标准 ID (11 位)
 *   - 数据帧
 *   - 经典 CAN 模式 (非 FD, 非 BRS)
 *   - 错误状态: ESI_ACTIVE
 *   - 不记录 TX 事件
 */
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;

    pTxHeader.Identifier          = id;                         /* CAN ID */
    pTxHeader.IdType              = FDCAN_STANDARD_ID;          /* 11 位标准帧 */
    pTxHeader.TxFrameType         = FDCAN_DATA_FRAME;           /* 数据帧 */

    if (len > 8) {
        return 1;                                               /* 经典 CAN 最长 8 字节 */
    }
    pTxHeader.DataLength          = FDCAN_DLC_BYTES_8;

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;           /* 节点错误状态: 正常 */
    pTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;              /* 不使用可变速率 */
    pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;          /* 经典 CAN 格式 */
    pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;         /* 不记录发送事件 */
    pTxHeader.MessageMarker       = 0;                          /* 消息标记 (不用) */

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) {
        if (hfdcan == &hfdcan1) {
            g_fdcan1_send_fail_count++;
            fdcan1_update_debug_status();
        }
        return 1;                                               /* TX FIFO 满或其他错误 */
    }
    if (hfdcan == &hfdcan1) {
        g_fdcan1_send_ok_count++;
        fdcan1_update_debug_status();
    }
    if (hfdcan == &hfdcan2) {
        g_fdcan2_send_ok_count++;
    }
    if (hfdcan == &hfdcan3) {
        g_fdcan3_send_ok_count++;
        fdcan3_update_debug_status();
    }
        return 0;

}

uint8_t fdcanx_send_ext_data(hcan_t *hfdcan, uint32_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;

    pTxHeader.Identifier          = id & 0x1FFFFFFFU;
    pTxHeader.IdType              = FDCAN_EXTENDED_ID;
    pTxHeader.TxFrameType         = FDCAN_DATA_FRAME;

    if (len > 8) {
        return 1;
    }
    pTxHeader.DataLength          = FDCAN_DLC_BYTES_8;

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker       = 0;

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) == HAL_OK) ? 0U : 1U;
}

/* ==========================================================================
 * CAN 帧接收
 * ========================================================================== */

/**
 * @brief  从 FDCAN RX FIFO0 读取一帧 CAN 数据
 *
 * 在 RX 中断回调中调用，从硬件 FIFO 取出收到的 CAN 帧。
 *
 * @param  hfdcan  FDCAN 句柄
 * @param  rec_id  输出: 接收到的 CAN ID (标准帧 11 位)
 * @param  buf     输出: 数据缓冲区 (需至少 8 字节)
 * @return 实际数据长度 (字节), 0 表示 FIFO 空或无新消息
 *
 * 注: DJI C620 反馈帧固定为 8 字节，所以正常返回值应为 8。
 *     返回值 ≤ 8 时使用 DataLength 字段，> 8 时参考 FDCAN DLC 编码。
 */
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
    {
        /* 保存 CAN ID */
        *rec_id = pRxHeader.Identifier;

        /* 解析数据长度: FDCAN DLC 编码 → 实际字节数 */
        if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;            /* 0~8 直接对应 */
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_12)
            len = 12;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_16)
            len = 16;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_20)
            len = 20;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_24)
            len = 24;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_32)
            len = 32;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_48)
            len = 48;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_64)
            len = 64;
        else
            len = 0;

        return len;
    }
    return 0;
}

uint8_t fdcanx_receive_any(hcan_t *hfdcan, uint32_t *rec_id, uint8_t *is_extended, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) != HAL_OK) {
        return 0U;
    }

    *rec_id = pRxHeader.Identifier;
    *is_extended = (pRxHeader.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;

    if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)
        len = pRxHeader.DataLength;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_12)
        len = 12;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_16)
        len = 16;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_20)
        len = 20;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_24)
        len = 24;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_32)
        len = 32;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_48)
        len = 48;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_64)
        len = 64;
    else
        len = 0;

    return len;
}


/* ==========================================================================
 * CAN 接收回调 — 中断上下文，将 CAN 数据喂入上层驱动
 *
 * 调用链:
 *   CAN 总线 (C620 电调 1kHz 上报)
 *     ↓ 硬件中断
 *   HAL_FDCAN_RxFifo0Callback()
 *     ↓ 按 hfdcan 实例分发
 *   fdcan1_rx_callback() / fdcan2 / fdcan3
 *     ↓ 读取 CAN 帧 + 调用上层解析
 *   DjiMotor_HandleFeedback()  (仅 FDCAN1，底盘电机)
 *     ↓ 更新 g_dji_motors[] 全局状态
 *   PID 控制循环 读取 g_dji_motors[].speed_rpm 等
 *
 * 时间节点:
 *   C620 电调以 1kHz 频率上报，4 个电调各自独立发送
 *   → 每个电机 1kHz，所以 FDCAN1 上每秒约 4000 帧 (4 电机)
 *   → 中断频率约 4kHz (每个电机每帧触发一次回调)
 *
 * CAN ID 分配:
 *   0x201 → 电机 1 (右前)
 *   0x202 → 电机 2 (左前)
 *   0x203 → 电机 3 (左后)
 *   0x204 → 电机 4 (右后)
 *   0x205~0x208 → 电机 5~8 (FDCAN2 上的升降电机)
 * ========================================================================== */

/*
 * FDCAN1 接收缓冲区 (底盘 4 个麦轮电机)
 * 每次中断读一帧 8 字节
 */
uint8_t  rx_data1[8] = {0};
uint16_t rec_id1;

/**
 * @brief  FDCAN1 接收回调 — 底盘 3508 电机 CAN 反馈处理
 *
 * 这是连接 CAN 硬件层和 DJI 电机驱动层的关键函数。
 * 从 FDCAN1 RX FIFO0 读取 8 字节 CAN 帧，
 * 然后调用 DjiMotor_HandleFeedback() 解析并更新电机状态。
 *
 * 关键细节:
 *   - 只处理 8 字节帧 (DJI C620 反馈帧固定长度)
 *   - 非 8 字节帧 (如远程帧、错误帧) 直接忽略
 *   - rec_id1 携带 CAN ID，由 DjiMotor_HandleFeedback 内部判断是否为 0x201~0x208
 */
void fdcan1_rx_callback(void)
{
    uint8_t len1;

    g_fdcan1_rx_callback_count++;
    len1 = fdcanx_receive(&hfdcan1, &rec_id1, rx_data1); //从硬件 FIFO 读取一帧 CAN 数据
    g_fdcan1_receive_len = len1;
    g_fdcan1_last_id = rec_id1;
    fdcan1_update_debug_status();

    if (len1 == 8U)
    {
        uint8_t motor_id;

        g_fdcan1_receive_ok_count++;
        motor_id = DjiMotor_HandleFeedback(rec_id1, rx_data1, HAL_GetTick()); // 解析 CAN 帧并更新电机状态
        if (motor_id != 0U) {
            g_fdcan1_dji_feedback_count++;
        } else {
            g_fdcan1_non_dji_count++;
        }
    }
}

/*
 * FDCAN2 接收缓冲区 (升降 4 个电机，目前暂不处理)
 */
uint8_t  rx_data2[8] = {0};
uint16_t rec_id2;

/**
 * @brief  FDCAN2 接收回调 — 升降 3508 电机 (TODO: 接入)
 *
 * 当前只读取 CAN 帧，不调用 DjiMotor_HandleFeedback。
 * 底盘用 FDCAN1 就够，升降电机后续再对接。
 */
void fdcan2_rx_callback(void)
{
    uint8_t len2;
    g_fdcan2_rx_callback_count++;
    len2 = fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
    if (len2 == 8U)
    {
        DjiMotor_HandleFeedback(rec_id2, rx_data2, HAL_GetTick());
    
    }
}

/*
 * FDCAN3 接收缓冲区 (灵足电机，目前暂不处理)
 */
uint8_t  rx_data3[8] = {0};
uint32_t rec_id3;
uint8_t  rec_id3_is_extended;

/**
 * @brief  FDCAN3 接收回调 — 灵足电机 (TODO: 接入)
 *
 * 当前只读取 CAN 帧。灵足电机使用灵足私有 CAN 协议，
 * 不能用 DjiMotor_HandleFeedback 解析。
 */
void fdcan3_rx_callback(void)
{
    uint8_t len3;
    uint8_t motor_id = 0U;

    g_fdcan3_rx_callback_count++;
    len3 = fdcanx_receive_any(&hfdcan3, &rec_id3, &rec_id3_is_extended, rx_data3);
    g_fdcan3_receive_len = len3;
    g_fdcan3_last_id = rec_id3;
    g_fdcan3_last_is_extended = rec_id3_is_extended;

    if (len3 == 8U)
    {
        g_fdcan3_receive_ok_count++;
        if (rec_id3_is_extended != 0U) {
            motor_id = RsMotor_HandlePrivateFeedback(rec_id3, rx_data3, HAL_GetTick());
        } else {
            motor_id = RsMotor_HandleFeedback((uint16_t)rec_id3, rx_data3, HAL_GetTick());
        }

        if (motor_id != 0U) {
            g_fdcan3_rs_feedback_count++;
        } else {
            g_fdcan3_non_rs_count++;
        }
    }
}

/**
 * @brief  HAL FDCAN RX FIFO0 全局中断回调
 *
 * 由 HAL 库在 FDCAN RX FIFO0 收到新消息时自动调用。
 * 根据触发中断的 CAN 外设实例分发到对应的回调函数。
 *
 * @param  hfdcan      触发中断的 FDCAN 句柄
 * @param  RxFifo0ITs  中断类型标志 (此处固定为 FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1)
    {
        g_fdcan1_hal_rx_callback_count++;
        fdcan1_rx_callback();    /* 底盘电机 CAN 反馈 */
    }
    if (hfdcan == &hfdcan2)
    {
        fdcan2_rx_callback();    /* 升降电机 CAN 反馈 */
    }
    if (hfdcan == &hfdcan3)
    {
        fdcan3_rx_callback();    /* 灵足电机 CAN 反馈 */
    }
}
