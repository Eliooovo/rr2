/**
 * @file    rc_control.h
 * @brief   FS-i6 IBUS 遥控接收与解析模块。
 *
 * 通过 USART10 接收 FS-i6 接收机（IA6B/IA10B）的 IBUS 协议帧，
 * 解析通道值并写入全局命令邮箱 g_comm_app_command，实现对底盘和
 * 升降的遥控。当遥控有效时，USB CDC 上位机命令被忽略。
 */

#ifndef RC_CONTROL_H
#define RC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- 公开 API ---- */

/** 初始化 USART10 单字节中断接收。应在 HAL 外设初始化后调用。 */
void RcControl_Init(void);

/**
 * 周期处理：组帧、通道映射、超时检测。
 * 应放在主循环中调用；内部自动限速为 10 ms 周期。
 */
void RcControl_RunPeriodic(void);

/**
 * 返回 1 表示最近 RC_FAILSAFE_TIMEOUT_MS 内收到过有效 IBUS 帧，
 * 遥控器正在主动控制。
 */
uint8_t RcControl_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_CONTROL_H */
