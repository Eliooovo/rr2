/**
 * @file    gripper_app.c
 * @brief   RS 电机夹爪两位置 MIT 控制
 */

#include "gripper_app.h"

#include "bsp_fdcan.h"
#include "main.h"
#include "rs_motor.h"

/* ---- 私有变量 ---- */
static rs_motor_t s_motor;
static volatile rs_motor_status_t s_init_status = RS_MOTOR_ERROR_NOT_INITIALIZED;
static volatile rs_motor_status_t s_ctrl_status = RS_MOTOR_ERROR_NOT_INITIALIZED;
volatile int g_gripper_app_step;

/* ---- 公共接口 ---- */

void GripperApp_Init(void)
{
    s_motor.config.hfdcan = &hfdcan3;
    s_motor.config.motor_id = 7U;
    s_motor.config.master_id = 0xFDU;
    s_motor.config.motor_type = RS_MOTOR_TYPE_5;
    s_motor.config.offline_timeout_ms = 100U;

    s_init_status = rs_motor_init(&s_motor);
}

void GripperApp_RunPeriodic(void)
{
    static uint32_t last_ctrl_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    float target_rad;

    if (s_init_status != RS_MOTOR_OK) {
        return;
    }

    /* 检测离线：feedback_count 停止增长 → 重置状态，下次重新使能。 */
    {
        rs_motor_feedback_t fb;
        if (rs_motor_get_feedback(&s_motor, &fb) == RS_MOTOR_OK) {
            static uint32_t last_fb_count = 0U;

            if (s_motor.state.feedback_count == last_fb_count) {
                // 超过 100ms 没新反馈 → 离线
                // 重置状态，下次重新 enable
                s_motor.internal.mode_applied = 0U;
                s_motor.state.enabled = 0U;
            }
            last_fb_count = s_motor.state.feedback_count;
        }
    }

    /* 按周期发 MIT 控制帧。 */
    if ((uint32_t)(now_ms - last_ctrl_ms) < GRIPPER_APP_CTRL_PERIOD_MS) {
        return;
    }
    last_ctrl_ms = now_ms;

    target_rad = (g_gripper_app_step == 0) ?
        GRIPPER_APP_POS_CLOSE_RAD :
        GRIPPER_APP_POS_OPEN_RAD;

    s_ctrl_status = rs_motor_motion_control(
        &s_motor,
        0.0f,                   /* torque_nm   — 零前馈 */
        target_rad,             /* position    — 0 或 -0.9 */
        0.0f,                   /* speed       — 目标速度为零 */
        GRIPPER_APP_KP,         /* kp          — 位置刚度 */
        GRIPPER_APP_KD);        /* kd          — 速度阻尼 */
}
