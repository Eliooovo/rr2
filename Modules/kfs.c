/**
 * @file    kfs.c
 * @brief   KFS 机构控制层 - 当前接入一个 RS00 升降电机
 *
 * RS 电机的位置 PID 在电机内部完成，所以这里不做“位置->速度->电流”的串级 PID。
 * 本模块只负责:
 *   1. 把 KFS 升降目标限制在安全范围内
 *   2. 配置 RS00 型号参数
 *   3. 通过 FDCAN3 周期性发送位置模式命令
 *   4. 从 rs_motor 读取多圈展开后的位置反馈
 */

#include "kfs.h"

#include "bsp_fdcan.h"
#include "main.h"

typedef struct {
    uint8_t motor_id;
    RsMotorModel model;
    int8_t direction;
    float default_speed_deg_s;
    float min_deg;
    float max_deg;
} KfsRsJointConfig;

static const KfsRsJointConfig s_lift_config = {
    KFS_LIFT_MOTOR_ID,
    KFS_LIFT_MOTOR_MODEL,
    KFS_LIFT_DIRECTION,
    KFS_LIFT_DEFAULT_SPEED_DEG_S,
    KFS_LIFT_MIN_DEG,
    KFS_LIFT_MAX_DEG
};

static float s_lift_target_deg;
static float s_lift_target_speed_deg_s;
static uint8_t s_lift_enabled;
static uint8_t s_lift_runmode_sent;
static uint8_t s_lift_enable_sent;
static uint8_t s_lift_limit_spd_sent;
static uint32_t s_last_control_ms;

#if KFS_LIFT_BOOT_TEST_ENABLE
/* 仅用于上电测试:
 * started=1 表示测试目标已经下发过，后续只由周期位置命令保持目标。 */
static uint8_t s_lift_boot_test_started;
static uint32_t s_lift_boot_test_start_ms;
#endif

static float Kfs_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static uint8_t Kfs_IsFinite(float value)
{
    return (value == value && value < 1000000.0f && value > -1000000.0f) ? 1U : 0U;
}

static void Kfs_SendFrameExt(uint32_t tx_id, uint8_t data[8])
{
    (void)fdcanx_send_ext_data(&hfdcan3, tx_id, data, 8U);
}

/* RS 电机进入位置控制需要几个动作:
 *   1. 切运行模式到私有协议位置模式(CSP, run_mode=5)
 *   2. 使能电机
 *   3. 写入速度限制
 *   4. 周期性写入目标位置
 * 这套序列只用于 KFS 升降的上电测试。 */
static uint8_t Kfs_PrepareLiftMotor(void)
{
    uint32_t tx_id;
    uint8_t data[8];

    if (s_lift_runmode_sent == 0U) {
        if (RsMotor_BuildPrivateRunModeFrame(s_lift_config.motor_id,
                                             5U,
                                             &tx_id,
                                             data) == 0U) {
            return 0U;
        }
        Kfs_SendFrameExt(tx_id, data);
        s_lift_runmode_sent = 1U;
        return 0U;
    }

    if (s_lift_enable_sent == 0U) {
        if (RsMotor_BuildPrivateEnableFrame(s_lift_config.motor_id, &tx_id, data) == 0U) {
            return 0U;
        }
        Kfs_SendFrameExt(tx_id, data);
        s_lift_enable_sent = 1U;
        return 0U;
    }

    if (s_lift_limit_spd_sent == 0U) {
        if (RsMotor_BuildPrivateParamWriteFrame(s_lift_config.motor_id,
                                                0x7017U,
                                                s_lift_target_speed_deg_s * 0.0174532925f,
                                                &tx_id,
                                                data) == 0U) {
            return 0U;
        }
        Kfs_SendFrameExt(tx_id, data);
        s_lift_limit_spd_sent = 1U;
        return 0U;
    }

    return 1U;
}

void Kfs_Init(void)
{
    (void)RsMotor_ConfigureMotor(s_lift_config.motor_id, s_lift_config.model);

    s_lift_target_deg = 0.0f;
    s_lift_target_speed_deg_s = s_lift_config.default_speed_deg_s;
    s_lift_enabled = 0U;
    s_lift_runmode_sent = 0U;
    s_lift_enable_sent = 0U;
    s_lift_limit_spd_sent = 0U;
    s_last_control_ms = 0U;

#if KFS_LIFT_BOOT_TEST_ENABLE
    s_lift_boot_test_started = 0U;
    s_lift_boot_test_start_ms = 0U;
#endif
}

void Kfs_Stop(void)
{
    uint32_t tx_id;
    uint8_t data[8];

    s_lift_enabled = 0U;
    s_lift_enable_sent = 0U;
    s_lift_limit_spd_sent = 0U;

    if (RsMotor_BuildPrivateStopFrame(s_lift_config.motor_id, &tx_id, data) != 0U) {
        Kfs_SendFrameExt(tx_id, data);
    }
}

void Kfs_SetLiftTargetDeg(float position_deg)
{
    if (!Kfs_IsFinite(position_deg)) {
        return;
    }

    s_lift_target_deg = Kfs_Clamp(position_deg,
                                  s_lift_config.min_deg,
                                  s_lift_config.max_deg);
    s_lift_target_speed_deg_s = s_lift_config.default_speed_deg_s;
    s_lift_enabled = 1U;
}

void Kfs_SetLiftZeroToCurrent(void)
{
    RsMotor_SetZeroToCurrent(s_lift_config.motor_id);
    s_lift_target_deg = 0.0f;
}

void Kfs_DisableLift(void)
{
    Kfs_Stop();
}

float Kfs_GetLiftTargetDeg(void)
{
    return s_lift_target_deg;
}

float Kfs_GetLiftPositionDeg(void)
{
    RsMotorState *motor = RsMotor_GetState(s_lift_config.motor_id);
    if (motor == 0) {
        return 0.0f;
    }

    return motor->angle_deg * (float)s_lift_config.direction;
}

uint8_t Kfs_IsLiftOnline(uint32_t now_ms)
{
    RsMotorState *motor = RsMotor_GetState(s_lift_config.motor_id);
    return (motor != 0 &&
            motor->online != 0U &&
            (now_ms - motor->last_update_ms) <= KFS_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

void Kfs_RunPeriodic(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms;
    uint32_t tx_id;
    uint8_t data[8];
    float motor_target_deg;

    if (s_last_control_ms == 0U) {
        s_last_control_ms = now_ms;
        return;
    }

    elapsed_ms = now_ms - s_last_control_ms;
    if (elapsed_ms < KFS_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;

#if KFS_LIFT_BOOT_TEST_ENABLE
    /* 仅用于上电测试:
     * 上电后等待一小段时间，再自动给 KFS 升降 RS00 一个固定位置目标。
     * 这样不需要上位机，也能验证 CAN、使能、位置模式、反馈和运动方向。
     * 正式联调上位机前，应在 kfs.h 中关闭 KFS_LIFT_BOOT_TEST_ENABLE。 */
    if (s_lift_boot_test_start_ms == 0U) {
        s_lift_boot_test_start_ms = now_ms;
    }
    if (s_lift_boot_test_started == 0U &&
        (now_ms - s_lift_boot_test_start_ms) >= KFS_LIFT_BOOT_TEST_DELAY_MS) {
        Kfs_SetLiftTargetDeg(KFS_LIFT_BOOT_TEST_TARGET_DEG);
        s_lift_target_speed_deg_s = KFS_LIFT_BOOT_TEST_SPEED_DEG_S;
        s_lift_boot_test_started = 1U;
    }
#endif

    if (s_lift_enabled == 0U) {
        return;
    }

    if (Kfs_PrepareLiftMotor() == 0U) {
        return;
    }

    motor_target_deg = s_lift_target_deg * (float)s_lift_config.direction;
    if (RsMotor_BuildPrivatePositionFrame(s_lift_config.motor_id,
                                          motor_target_deg * 0.0174532925f,
                                          s_lift_target_speed_deg_s * 0.0174532925f,
                                          &tx_id,
                                          data) != 0U) {
        Kfs_SendFrameExt(tx_id, data);
    }
}
