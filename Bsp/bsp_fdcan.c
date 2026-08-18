/**
 * @file    bsp_fdcan.c
 * @brief   FDCAN 板级初始化、滤波与统一接收分发
 */

#include "bsp_fdcan.h"

#include "dji_motor_group.h"
#include "rs_motor.h"

volatile uint32_t g_fdcan1_hal_rx_callback_count;
volatile uint32_t g_fdcan1_rx_callback_count;
volatile uint32_t g_fdcan2_rx_callback_count;
volatile uint32_t g_fdcan3_rx_callback_count;
volatile uint32_t g_fdcan1_receive_ok_count;
volatile uint32_t g_fdcan1_receive_len;
volatile uint32_t g_fdcan1_last_id;
volatile uint32_t g_fdcan1_dji_feedback_count;
volatile uint32_t g_fdcan1_non_dji_count;
volatile uint32_t g_fdcan1_tx_fifo_free_level;
volatile uint32_t g_fdcan1_hal_error;
volatile uint32_t g_fdcan1_last_error_code;
volatile uint32_t g_fdcan1_error_passive;
volatile uint32_t g_fdcan1_warning;
volatile uint32_t g_fdcan1_bus_off;
volatile uint32_t g_fdcan1_tx_error_count;
volatile uint32_t g_fdcan1_rx_error_count;

static volatile uint32_t s_fdcan3_rx_frame_count;
static volatile uint32_t s_fdcan3_rx_handled_count;
static volatile uint32_t s_fdcan3_rx_unhandled_count;
static volatile uint32_t s_fdcan3_rx_read_error_count;
static volatile uint32_t s_fdcan3_rx_fifo_max_fill;
static volatile uint32_t s_fdcan3_rx_fifo_full_count;
static volatile uint32_t s_fdcan3_rx_message_lost_count;

static void bsp_fdcan1_update_debug_status(void)
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

void bsp_can_init(void)
{
    can_filter_init();

    (void)HAL_FDCAN_Start(&hfdcan1);
    (void)HAL_FDCAN_Start(&hfdcan2);
    (void)HAL_FDCAN_Start(&hfdcan3);

    (void)HAL_FDCAN_ActivateNotification(
        &hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    (void)HAL_FDCAN_ActivateNotification(
        &hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    (void)HAL_FDCAN_ActivateNotification(
        &hfdcan3,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_RX_FIFO0_FULL |
            FDCAN_IT_RX_FIFO0_MESSAGE_LOST,
        0U);
}

void can_filter_init(void)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    (void)HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = (0x02UL << 24U);
    filter.FilterID2 = (0x1FUL << 24U);
    (void)HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);

    (void)HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan2,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan3,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE);

    (void)HAL_FDCAN_ConfigFifoWatermark(
        &hfdcan1, FDCAN_CFG_RX_FIFO0, 1U);
    (void)HAL_FDCAN_ConfigFifoWatermark(
        &hfdcan2, FDCAN_CFG_RX_FIFO0, 1U);
    (void)HAL_FDCAN_ConfigFifoWatermark(
        &hfdcan3, FDCAN_CFG_RX_FIFO0, 1U);
}

static uint8_t bsp_fdcan_handle_rx_frame(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t handled = 0U;

    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &header,
                               data) != HAL_OK) {
        if (hfdcan == &hfdcan3) {
            s_fdcan3_rx_read_error_count++;
        }
        return 0U;
    }

    if (hfdcan == &hfdcan3) {
        s_fdcan3_rx_frame_count++;
    }

    if (hfdcan == &hfdcan1) {
        g_fdcan1_receive_ok_count++;
        g_fdcan1_receive_len =
            (header.DataLength == FDCAN_DLC_BYTES_8) ? 8U : 0U;
        g_fdcan1_last_id = header.Identifier;
    }

    if (hfdcan == &hfdcan1 || hfdcan == &hfdcan2) {
        handled = dji_motor_group_handle_rx(hfdcan,
                                            &header,
                                            data,
                                            HAL_GetTick());
    } else {
        handled = rs_motor_handle_rx(hfdcan,
                                     &header,
                                     data,
                                     HAL_GetTick());
    }

    if (hfdcan == &hfdcan1) {
        if (handled != 0U) {
            g_fdcan1_dji_feedback_count++;
        } else {
            g_fdcan1_non_dji_count++;
        }
        bsp_fdcan1_update_debug_status();
    } else if (hfdcan == &hfdcan3) {
        if (handled != 0U) {
            s_fdcan3_rx_handled_count++;
        } else {
            s_fdcan3_rx_unhandled_count++;
        }
    }
    return 1U;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its)
{
    uint32_t pending_frames;

    if (hfdcan == &hfdcan1) {
        g_fdcan1_hal_rx_callback_count++;
        g_fdcan1_rx_callback_count++;
    } else if (hfdcan == &hfdcan2) {
        g_fdcan2_rx_callback_count++;
    } else if (hfdcan == &hfdcan3) {
        g_fdcan3_rx_callback_count++;
        if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_FULL) != 0U) {
            s_fdcan3_rx_fifo_full_count++;
        }
        if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
            s_fdcan3_rx_message_lost_count++;
        }
    } else {
        return;
    }

    pending_frames = HAL_FDCAN_GetRxFifoFillLevel(hfdcan,
                                                   FDCAN_RX_FIFO0);
    if (hfdcan == &hfdcan3 &&
        pending_frames > s_fdcan3_rx_fifo_max_fill) {
        s_fdcan3_rx_fifo_max_fill = pending_frames;
    }

    while (pending_frames > 0U) {
        if (bsp_fdcan_handle_rx_frame(hfdcan) == 0U) {
            break;
        }
        pending_frames--;
    }
}
