/**
 * @file    bsp_fdcan.h
 * @brief   FDCAN 板级支持包 — 底盘 CAN 总线通信层
 *
 * 硬件: STM32H723VGT6, 3 路 FDCAN (经典 CAN 模式, 1Mbps)
 *
 * 总线分配 (来自 README.md):
 *   FDCAN1 → 底盘 4 个 DJI M3508 麦轮电机 (C620 电调)
 *   FDCAN2 → 升降 4 个 DJI M3508 电机
 *   FDCAN3 → 灵足电机 (夹爪等)
 *
 * 工作流程:
 *   发送: 上层 PID 控制循环 → DjiMotor_SetCurrent() →
 *         DjiMotor_BuildCurrentFrame() → fdcanx_send_data() → HAL FDCAN TX
 *   接收: C620 电调以 1kHz 自动上报 → HAL FDCAN RX 中断 →
 *         HAL_FDCAN_RxFifo0Callback() → fdcan1_rx_callback() →
 *         DjiMotor_HandleFeedback() → 更新 g_dji_motors[]
 */

#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "main.h"
#include "fdcan.h"

/* 类型别名，简化 HAL 句柄引用 */
#define hcan_t FDCAN_HandleTypeDef

extern volatile uint32_t g_fdcan1_hal_rx_callback_count;
extern volatile uint32_t g_fdcan1_rx_callback_count;
extern volatile uint32_t g_fdcan1_receive_ok_count;
extern volatile uint32_t g_fdcan1_receive_len;
extern volatile uint32_t g_fdcan1_last_id;
extern volatile uint32_t g_fdcan1_dji_feedback_count;
extern volatile uint32_t g_fdcan1_non_dji_count;
extern volatile uint32_t g_fdcan1_send_ok_count;
extern volatile uint32_t g_fdcan1_send_fail_count;
extern volatile uint32_t g_fdcan1_tx_fifo_free_level;
extern volatile uint32_t g_fdcan1_hal_error;
extern volatile uint32_t g_fdcan1_last_error_code;
extern volatile uint32_t g_fdcan1_error_passive;
extern volatile uint32_t g_fdcan1_warning;
extern volatile uint32_t g_fdcan1_bus_off;
extern volatile uint32_t g_fdcan1_tx_error_count;
extern volatile uint32_t g_fdcan1_rx_error_count;

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/* CAN 总线初始化：配置滤波器 + 启动 3 路 FDCAN + 使能 RX FIFO0 中断通知 */
void bsp_can_init(void);

/* CAN 滤波器初始化：设置全通滤波器 (不过滤任何 ID) + 全局过滤配置 */
void can_filter_init(void);

/* ==========================================================================
 * 数据收发
 * ========================================================================== */

/**
 * @brief  发送一帧 CAN 数据
 * @param  hfdcan  FDCAN 句柄 (&hfdcan1 / &hfdcan2 / &hfdcan3)
 * @param  id      CAN 标准帧 ID (11 位)
 * @param  data    数据缓冲区
 * @param  len     数据长度 (≤8，经典 CAN 帧)
 * @return  0=成功, 1=失败 (长度超限或 HAL 发送失败)
 */
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

/**
 * @brief  从 FDCAN RX FIFO0 读取一帧数据
 * @param  hfdcan  FDCAN 句柄
 * @param  rec_id  输出: 接收到的 CAN ID
 * @param  buf     输出: 接收数据缓冲区 (至少 8 字节)
 * @return 实际接收的数据长度 (字节), 0 表示 FIFO 空或读取失败
 */
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);

/* ==========================================================================
 * CAN 接收回调 (中断上下文调用)
 *
 * 调用链:
 *   C620 电调发送 CAN 帧 →
 *   STM32 FDCAN 硬件接收 →
 *   HAL_FDCAN_RxFifo0Callback() (HAL 中断回调) →
 *   fdcan1_rx_callback() / fdcan2_rx_callback() / fdcan3_rx_callback() →
 *   DjiMotor_HandleFeedback() (仅 FDCAN1，底盘电机)
 * ========================================================================== */

void fdcan1_rx_callback(void);   /* FDCAN1 接收回调: 底盘 4 个 3508 电机反馈 */
void fdcan2_rx_callback(void);   /* FDCAN2 接收回调: 升降 4 个 3508 电机反馈 */
void fdcan3_rx_callback(void);   /* FDCAN3 接收回调: 灵足电机反馈 */

#endif /* __BSP_FDCAN_H_ */
