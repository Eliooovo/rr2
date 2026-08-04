/**
 * @file    rc_control.h
 * @brief   FS-i6 IBUS 遥控接收与解析模块。
 *
 * 通过 USART10 接收 FS-i6 接收机（IA6B/IA10B）的 IBUS 协议帧，
 * 解析通道值并写入全局命令邮箱 g_comm_app_command，实现对底盘和
 * 升降的遥控。
 *
 * 控制权管理：
 *   - 上电默认 RC 主动控制（ACTIVE）
 *   - CH5 回中保持 2 秒 → 让出控制权给上位机（YIELDED），
 *     此状态下 RC 不写邮箱，上位机独占所有控制权
 *   - CH5 离开中位 → 夺回控制权，自动从电机反馈同步升降位置
 *   - 信号丢失 → 强制让出
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
 * 周期处理：组帧、通道映射、超时检测、控制权让出状态机。
 * 每主循环周期均执行，RC 在 CommApp 之后运行以保持优先。
 * 应放在主循环中 CommApp_RunPeriodic() 之后调用。
 */
void RcControl_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_CONTROL_H */
