/**
 * @file    comm_protocol.h
 * @brief   USB 虚拟串口通讯协议 — 与 Jetson Nano 上位机通信
 *
 * 帧格式 (固定长度 46 字节，二进制):
 *   命令帧:   0xAA + 11 float (LE) + 0x55
 *   反馈帧:   0xAA + 11 float (LE) + 0x55
 *
 * 11 个 float 字段顺序一致，双向共用同一布局，Jetson 端只需切换头尾即可。
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define COMM_FLOAT_COUNT        11U                         /* 每帧 11 个 float */
#define COMM_PACKET_SIZE        (1U + COMM_FLOAT_COUNT * 4U + 1U)  /* 1 + 44 + 1 = 46 字节 */
#define COMM_CMD_HEAD           0xAAU                      /* 命令帧头 */
#define COMM_CMD_TAIL           0x55U                      /* 命令帧尾 */
#define COMM_FEEDBACK_HEAD      0xAAU                      /* 反馈帧头 */
#define COMM_FEEDBACK_TAIL      0x55U                      /* 反馈帧尾 */
#define COMM_FEEDBACK_PERIOD_MS 20U                        /* 反馈帧发送周期 (二进制模式) */

/* 调试: 发送固定 46 字节测试帧 (头尾填好，float 全 0)，验证 USB CDC 链路 */
#define COMM_USB_TEST_FRAME_ENABLE 0U
#define COMM_USB_TEST_FRAME_PERIOD_MS 1000U

/* 一帧里的 11 个 float 字段 */
typedef struct {
    float vx;                 /* 底盘 X 速度 (rpm) */
    float vy;                 /* 底盘 Y 速度 (rpm) */
    float vw;                 /* 底盘 yaw 角速度 (rpm) */
    float front_lift;         /* 前升降组目标位置 (度) */
    float rear_lift;          /* 后升降组目标位置 (度) */
    float rs_actuator_0;      /* RS 位置执行器 0 目标 (度), 当前映射 RS00 ID3 */
    float rs_actuator_1;      /* RS 位置执行器 1 目标 (度), 预留 */
    float rs_actuator_2;      /* RS 位置执行器 2 目标 (度), 预留 */
    float rs_actuator_3;      /* RS 位置执行器 3 目标 (度), 预留 */
    float rs_actuator_4;      /* RS 位置执行器 4 目标 (度), 预留 */
    float rs_actuator_5;      /* RS 位置执行器 5 目标 (度), 预留 */
} CommFrameFloats;

/* ==========================================================================
 * 公开接口
 * ========================================================================== */

void Comm_Init(void);                               /* 初始化缓冲区、状态 */
void Comm_RunPeriodic(void);                        /* main() while(1) 中每圈都调: 解析→执行→反馈 */
void Comm_OnUsbReceived(const uint8_t *data, uint32_t len);  /* USB CDC 收到数据时回调 */

uint8_t Comm_HasCommand(void);                      /* 是否收到过指令 */
const CommFrameFloats *Comm_GetLastCommand(void);   /* 获取最近一次收到的命令 */

extern volatile uint32_t g_comm_rx_valid_frame_count;
extern volatile float g_comm_rx_vx;
extern volatile float g_comm_rx_vy;
extern volatile float g_comm_rx_vw;
extern volatile uint32_t g_comm_rx_vx_bits;
extern volatile uint32_t g_comm_rx_vy_bits;
extern volatile uint32_t g_comm_rx_vw_bits;
extern volatile uint8_t g_comm_rx_last_packet[COMM_PACKET_SIZE];
extern volatile uint32_t g_comm_rx_float_bits[COMM_FLOAT_COUNT];
extern volatile float g_comm_apply_vx;
extern volatile float g_comm_apply_vy;
extern volatile float g_comm_apply_vw;
extern volatile uint32_t g_comm_apply_count;

#ifdef __cplusplus
}
#endif

#endif
