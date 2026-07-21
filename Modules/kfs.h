#ifndef KFS_H
#define KFS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "rs_motor.h"

/* ==========================================================================
 * KFS 机构层
 *
 * 分层原则:
 *   - Device/rs_motor.* 管灵足电机协议，不知道电机装在哪
 *   - Modules/kfs.* 管 KFS 机构配置，知道哪个关节用哪个电机
 *
 * 当前实物:
 *   - KFS 升降电机: RS00, FDCAN3, motor_id=1
 *
 * 后续扩展:
 *   - KFS 根部旋转: RS03
 *   - KFS 末端旋转: RS00
 *   - KFS 开合:     RS05
 * ========================================================================== */

#define KFS_CONTROL_PERIOD_MS          20U
#define KFS_OFFLINE_TIMEOUT_MS         100U

#define KFS_LIFT_MOTOR_ID              3U
#define KFS_LIFT_MOTOR_MODEL           RS_MOTOR_MODEL_RS00
#define KFS_LIFT_DIRECTION             1
#define KFS_LIFT_DEFAULT_SPEED_DEG_S   90.0f

/* 这里先用宽限位，方便你测试 361deg 多圈目标。
 * 上车前应按真实丝杆/连杆行程改成机械安全范围。 */
#define KFS_LIFT_MIN_DEG               -7200.0f
#define KFS_LIFT_MAX_DEG                7200.0f

/* ==========================================================================
 * KFS 升降 RS00 上电测试
 *
 * 重要: 下面这组宏只用于调试/试车。
 * 打开后，MCU 上电会自动使能 KFS 升降 RS00，并运动到
 * KFS_LIFT_BOOT_TEST_TARGET_DEG 指定的位置。
 *
 * 第一次带机构测试建议保持小角度，例如 10deg 或 30deg；
 * 确认方向、限位、急停方式都没问题后，再测试 361deg 等多圈目标。
 * 正式由上位机控制时，把 KFS_LIFT_BOOT_TEST_ENABLE 改回 0。
 * ========================================================================== */
#define KFS_LIFT_BOOT_TEST_ENABLE       1U
#define KFS_LIFT_BOOT_TEST_DELAY_MS     1000U
#define KFS_LIFT_BOOT_TEST_TARGET_DEG   90.0f
#define KFS_LIFT_BOOT_TEST_SPEED_DEG_S  30.0f

typedef enum {
    KFS_JOINT_LIFT = 0,
    KFS_JOINT_ROOT_ROTATE,
    KFS_JOINT_END_ROTATE,
    KFS_JOINT_GRIP,
    KFS_JOINT_COUNT,
} KfsJointIndex;

void Kfs_Init(void);
void Kfs_RunPeriodic(void);
void Kfs_Stop(void);

void Kfs_SetLiftTargetDeg(float position_deg);
void Kfs_SetLiftZeroToCurrent(void);
void Kfs_DisableLift(void);

float Kfs_GetLiftTargetDeg(void);
float Kfs_GetLiftPositionDeg(void);
uint8_t Kfs_IsLiftOnline(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
