/**
 * @file    kfs_rotate_app.c
 * @brief   KFS 旋转关节位置控制（根部 PP，末端 CSP，限速 ~45°/s）。
 *
 * 管理两个旋转关节：根部（RS03, id=2）和末端（RS00, id=3）。
 * 上位机通过 USB 协议下发目标角度（单位 rad），App 在收到首个有效命令
 * 后使能对应电机，并以不超过 KFS_ROTATE_APP_MAX_SPEED_RAD_S 的速度平滑到位。
 * 反馈为单圈角度（依赖机械零位），不使用多圈累计。
 */

#include "kfs_rotate_app.h"

#include <float.h>

#include "comm_app.h"
#include "fdcan.h"
#include "main.h"
#include "rs_motor.h"

/* ---- 旋转关节描述符 ---- */

typedef struct {
    rs_motor_t motor;                     /* 电机实例 */

    /* 上位机命令/反馈字段绑定（Init 时设置，之后只读/只写）。 */
    const volatile float *command_rad;    /* 目标角度 rad */
    volatile float *feedback_rad;         /* 当前角度 rad */
    volatile uint8_t *feedback_valid;     /* 反馈有效标志 */

    /* 运行时状态 */
    float    target_position_rad;         /* 当前目标位置 */
    uint8_t  target_received;             /* 是否收到过有效命令 */
    uint8_t  use_pp;                      /* 1: PP，0: CSP */
    uint32_t last_command_sequence;       /* 上次处理的命令序列号 */
    uint32_t last_ctrl_ms;                /* 上次检查位置命令的时间戳 */
    volatile rs_motor_status_t last_control_status; /* 保留最后一次控制返回值 */
} kfs_rotate_joint_t;

/* ---- 关节实例 ---- */

static kfs_rotate_joint_t s_root;
static kfs_rotate_joint_t s_tip;
static uint8_t s_initialized;
static volatile rs_motor_status_t s_root_loc_kp_write_status;
static volatile rs_motor_status_t s_root_spd_kp_write_status;
static volatile rs_motor_status_t s_root_spd_ki_write_status;

/* ---- 辅助函数 ---- */

static uint8_t KfsRotateIsFinite(float value)
{
    return (value >= -FLT_MAX && value <= FLT_MAX) ? 1U : 0U;
}

static void KfsRotateJoint_ClearFeedback(kfs_rotate_joint_t *joint)
{
    *joint->feedback_rad = 0.0f;
    *joint->feedback_valid = 0U;
}

static void KfsRotateJoint_FailAndDisable(kfs_rotate_joint_t *joint)
{
    KfsRotateJoint_ClearFeedback(joint);
    (void)rs_motor_disable(&joint->motor);
}

static uint8_t KfsRotateJoint_FeedbackIsRecent(const kfs_rotate_joint_t *joint,
                                               uint32_t now_ms)
{
    return (joint->motor.state.online != 0U &&
            (uint32_t)(now_ms - joint->motor.state.last_update_ms) <
                KFS_ROTATE_APP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

/* 检查上位机新命令：sequence 递增说明收到新帧，更新目标。 */
static void KfsRotateJoint_UpdateTarget(kfs_rotate_joint_t *joint)
{
    uint32_t sequence = g_comm_app_command.sequence;
    float target_rad;

    if (g_comm_app_command.valid == 0U ||
        sequence == joint->last_command_sequence) {
        return;
    }
    joint->last_command_sequence = sequence;

    target_rad = *joint->command_rad;
    if (KfsRotateIsFinite(target_rad) == 0U) {
        /* 非有限值不覆盖旧目标，保持上一次有效角度命令。 */
        return;
    }

    if (joint->target_received == 0U ||
        target_rad != joint->target_position_rad) {
        joint->target_position_rad = target_rad;
    }
    joint->target_received = 1U;
}

static void KfsRotateJoint_UpdateFeedback(kfs_rotate_joint_t *joint,
                                          uint32_t now_ms)
{
    rs_motor_feedback_t fb;
    float angle_rad;

    if (KfsRotateJoint_FeedbackIsRecent(joint, now_ms) == 0U ||
        rs_motor_get_feedback(&joint->motor, &fb) != RS_MOTOR_OK) {
        KfsRotateJoint_ClearFeedback(joint);
        return;
    }

    angle_rad = fb.angle_rad;
    if (KfsRotateIsFinite(angle_rad) == 0U) {
        KfsRotateJoint_ClearFeedback(joint);
        return;
    }

    *joint->feedback_rad = angle_rad;
    *joint->feedback_valid = 1U;
}

static void KfsRotateJoint_RunPeriodic(kfs_rotate_joint_t *joint,
                                       uint32_t now_ms)
{
    rs_motor_status_t status;

    /* 离线判断由驱动按反馈时间戳处理，不能按主循环次数判断。 */
    if (rs_motor_update(&joint->motor, now_ms) != RS_MOTOR_OK) {
        KfsRotateJoint_FailAndDisable(joint);
        return;
    }

    /* 1. 更新反馈到上位机邮箱。 */
    KfsRotateJoint_UpdateFeedback(joint, now_ms);

    /* 2. 按周期检查命令、下发位置目标。 */
    if ((uint32_t)(now_ms - joint->last_ctrl_ms) <
        KFS_ROTATE_APP_CTRL_PERIOD_MS) {
        return;
    }
    joint->last_ctrl_ms = now_ms;

    KfsRotateJoint_UpdateTarget(joint);

    /*
     * 安全：未收到任何有效上位机命令前不使能电机，
     * 避免上电后在无目标的情况下自运动。
     */
    if (joint->target_received == 0U) {
        return;
    }

    /*
     * 周期重发位置目标作为 CAN keepalive。RobStride 的 Type 2
     * 实际反馈由控制交互刷新；只在目标变化时下发会使反馈超时。
     */
    if (joint->use_pp != 0U) {
        status = rs_motor_pp_position_control_limited(
            &joint->motor,
            KFS_ROTATE_APP_ROOT_PP_CURRENT_LIMIT_A,
            KFS_ROTATE_APP_MAX_SPEED_RAD_S,
            KFS_ROTATE_APP_ROOT_PP_ACCELERATION_RAD_S2,
            joint->target_position_rad);
    } else {
        status = rs_motor_csp_position_control(
            &joint->motor,
            KFS_ROTATE_APP_MAX_SPEED_RAD_S,
            joint->target_position_rad);
    }
    joint->last_control_status = status;
    if (status != RS_MOTOR_OK) {
        KfsRotateJoint_FailAndDisable(joint);
        return;
    }
}

/* ---- 关节初始化 ---- */

static void KfsRotateJoint_Init(kfs_rotate_joint_t *joint,
                                uint8_t motor_id,
                                rs_motor_type_t motor_type,
                                uint8_t use_pp,
                                const volatile float *command_rad,
                                volatile float *feedback_rad,
                                volatile uint8_t *feedback_valid)
{
    joint->command_rad = command_rad;
    joint->feedback_rad = feedback_rad;
    joint->feedback_valid = feedback_valid;
    joint->use_pp = use_pp;
    joint->last_control_status = RS_MOTOR_STATUS_NOT_INITIALIZED;

    KfsRotateJoint_ClearFeedback(joint);

    /* 硬件通信配置由 CubeMX/FDCAN3 和 BSP 负责，这里只注册 RobStride 实例。 */
    joint->motor.config.hfdcan = &hfdcan3;
    joint->motor.config.motor_id = motor_id;
    joint->motor.config.master_id = 0xFDU;
    joint->motor.config.motor_type = motor_type;
    joint->motor.config.offline_timeout_ms = KFS_ROTATE_APP_OFFLINE_TIMEOUT_MS;

    if (rs_motor_init(&joint->motor) != RS_MOTOR_OK) {
        KfsRotateJoint_FailAndDisable(joint);
        return;
    }

    joint->target_position_rad = 0.0f;
    joint->target_received = 0U;
    joint->last_command_sequence = 0U;
    joint->last_ctrl_ms = 0U;
}

/* ---- 公共接口 ---- */

void KfsRotateApp_Init(void)
{
    if (s_initialized != 0U) {
        return;
    }

    /* 参数合法性检查（只做一次）。 */
    if (KfsRotateIsFinite(KFS_ROTATE_APP_MAX_SPEED_RAD_S) == 0U ||
        KfsRotateIsFinite(KFS_ROTATE_APP_ROOT_PP_ACCELERATION_RAD_S2) == 0U ||
        KfsRotateIsFinite(KFS_ROTATE_APP_ROOT_PP_CURRENT_LIMIT_A) == 0U ||
        KFS_ROTATE_APP_MAX_SPEED_RAD_S <= 0.0f ||
        KFS_ROTATE_APP_ROOT_PP_ACCELERATION_RAD_S2 <= 0.0f ||
        KFS_ROTATE_APP_ROOT_PP_CURRENT_LIMIT_A <= 0.0f ||
        KFS_ROTATE_APP_CTRL_PERIOD_MS == 0U ||
        KFS_ROTATE_APP_CTRL_PERIOD_MS >=
            KFS_ROTATE_APP_OFFLINE_TIMEOUT_MS) {
        return;
    }

    /* 根部旋转：RS03, id=2, comm 字段 kfs_root_rotate。 */
    KfsRotateJoint_Init(
        &s_root,
        2U,
        RS_MOTOR_TYPE_3,
        1U,
        &g_comm_app_command.kfs_root_rotate_rad,
        &g_comm_app_feedback.kfs_root_rotate_rad,
        &g_comm_app_feedback.kfs_root_rotate_valid);

    /* 末端旋转：RS00, id=3, comm 字段 kfs_tip_rotate。 */
    KfsRotateJoint_Init(
        &s_tip,
        3U,
        RS_MOTOR_TYPE_0,
        0U,
        &g_comm_app_command.kfs_tip_rotate_rad,
        &g_comm_app_feedback.kfs_tip_rotate_rad,
        &g_comm_app_feedback.kfs_tip_rotate_valid);

    /*
     * RS03（根部）电机内部位置/速度环 PID（PP 模式仍使用）。
     * 提高位置环增益以增加偏离目标时的保持阻力，
     * 同时回调速度环比例增益，减少负载静止时对编码器
     * 微小速度抖动的放大。
     */
    s_root_loc_kp_write_status =
        rs_motor_write_parameter(&s_root.motor, RS_PARAM_LOC_KP, 60.0f);
    s_root_spd_kp_write_status =
        rs_motor_write_parameter(&s_root.motor, RS_PARAM_SPD_KP, 6.0f);
    s_root_spd_ki_write_status =
        rs_motor_write_parameter(&s_root.motor, RS_PARAM_SPD_KI, 0.02f);

    s_initialized = 1U;
}

void KfsRotateApp_RunPeriodic(void)
{
    uint32_t now_ms;

    if (s_initialized == 0U) {
        g_comm_app_feedback.kfs_root_rotate_rad = 0.0f;
        g_comm_app_feedback.kfs_root_rotate_valid = 0U;
        g_comm_app_feedback.kfs_tip_rotate_rad = 0.0f;
        g_comm_app_feedback.kfs_tip_rotate_valid = 0U;
        return;
    }

    now_ms = HAL_GetTick();
    KfsRotateJoint_RunPeriodic(&s_root, now_ms);
    KfsRotateJoint_RunPeriodic(&s_tip, now_ms);
}
