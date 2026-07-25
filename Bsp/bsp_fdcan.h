/**
 * @file    bsp_fdcan.h
 * @brief   FDCAN 板级初始化、滤波与统一接收分发
 */

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "fdcan.h"
#include "main.h"

extern volatile uint32_t g_fdcan1_hal_rx_callback_count;
extern volatile uint32_t g_fdcan1_rx_callback_count;
extern volatile uint32_t g_fdcan1_receive_ok_count;
extern volatile uint32_t g_fdcan1_receive_len;
extern volatile uint32_t g_fdcan1_last_id;
extern volatile uint32_t g_fdcan1_dji_feedback_count;
extern volatile uint32_t g_fdcan1_non_dji_count;
extern volatile uint32_t g_fdcan1_tx_fifo_free_level;
extern volatile uint32_t g_fdcan1_hal_error;
extern volatile uint32_t g_fdcan1_last_error_code;
extern volatile uint32_t g_fdcan1_error_passive;
extern volatile uint32_t g_fdcan1_warning;
extern volatile uint32_t g_fdcan1_bus_off;
extern volatile uint32_t g_fdcan1_tx_error_count;
extern volatile uint32_t g_fdcan1_rx_error_count;

/** 配置过滤器、启动三路 FDCAN 并使能 RX FIFO0 通知。 */
void bsp_can_init(void);

/** FDCAN1/2 接收 DJI 标准帧，FDCAN3 接收 RobStride 扩展反馈。 */
void can_filter_init(void);

#endif /* BSP_FDCAN_H */
