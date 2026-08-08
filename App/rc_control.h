/**
 * @file    rc_control.h
 * @brief   FS-i6 IBUS 遥控接收与解析模块。
 *
 * 通过 USART10 接收 FS-i6 接收机（IA6B/IA10B）的 IBUS 协议帧，
 * 解析通道值并写入全局命令邮箱 g_comm_app_command，实现对底盘和
 * 升降的遥控。
 *
 * 控制权管理：
 *   - 上电默认离线，USB 可控制全部轴
 *   - 收到有效 IBUS 帧且 CH6 接近 -100% / +100% → 在线
 *   - 在线时 RC 每轮覆盖底盘+升降，其它轴仍由 USB 控制
 *   - CH6 离开两个端点（如 failsafe 32%）→ 离线，USB 接管
 *   - 信号恢复 → 自动上线，从电机反馈同步升降位置
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
 * 周期处理：通道快照、通道映射、CH6 在线/离线状态机和接收看门狗。
 * 每主循环周期均执行，RC 在 CommApp 之后运行以保持优先。
 * 应放在主循环中 CommApp_RunPeriodic() 之后调用。
 */
void RcControl_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_CONTROL_H */
