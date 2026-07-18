/**
 * @file    chassis.c
 * @brief   麦轮底盘控制 — 4 x DJI M3508 (FDCAN1), 速度 PID 闭环
 *
 * 数据流:
 *   上位机 → SetVelocityRpm(vx,vy,wz) → 麦轮解算 → 目标转速
 *   C620 反馈 → DjiMotor_HandleFeedback → g_dji_motors[].speed_rpm (中断上下文)
 *   ControlLoop: 目标转速 vs 实际转速 → PID → SetCurrent → CAN 0x200 发送
 */

#include "chassis.h"

#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "pid.h"

/* PID 参数 — 速度环，需根据实际负载整定 */
#define CHASSIS_SPEED_PID_KP       6.0f
#define CHASSIS_SPEED_PID_KI       1.0f
#define CHASSIS_SPEED_PID_KD       0.05f
#define CHASSIS_CURRENT_LIMIT      12000.0f    /* 输出限幅，C620 最大 16384 */
#define CHASSIS_INTEGRAL_LIMIT     6000.0f
#define CHASSIS_OFFLINE_TIMEOUT_MS 100U        /* 超过此时间无反馈视为离线 */

/* 底盘电机映射: 电机 ID 1,2,3,4 对应 FDCAN1 上的四个轮子 */
static const uint8_t s_motor_ids[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_MOTOR_ID_1, CHASSIS_MOTOR_ID_2,
    CHASSIS_MOTOR_ID_3, CHASSIS_MOTOR_ID_4,
};

/* 方向系数: 试车发现某轮反转时，在 chassis.h 改对应的 DIR 为 -1 */
static const int8_t s_motor_dirs[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_MOTOR_DIR_1, CHASSIS_MOTOR_DIR_2,
    CHASSIS_MOTOR_DIR_3, CHASSIS_MOTOR_DIR_4,
};

static PidController s_speed_pid[CHASSIS_MOTOR_COUNT];
static float        s_target_rpm[CHASSIS_MOTOR_COUNT];
static uint32_t     s_last_control_ms;

/* float 电流值 → int16_t，自动限幅 */
static int16_t Chassis_FloatToCurrent(float value)
{
    if (value > CHASSIS_CURRENT_LIMIT)  return (int16_t)CHASSIS_CURRENT_LIMIT;
    if (value < -CHASSIS_CURRENT_LIMIT) return (int16_t)-CHASSIS_CURRENT_LIMIT;
    return (int16_t)value;
}

/* ==========================================================================
 * 初始化 & 停止
 * ========================================================================== */

void Chassis_Init(void)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        Pid_Init(&s_speed_pid[i],
                 CHASSIS_SPEED_PID_KP, CHASSIS_SPEED_PID_KI, CHASSIS_SPEED_PID_KD,
                 -CHASSIS_CURRENT_LIMIT,  CHASSIS_CURRENT_LIMIT,
                 -CHASSIS_INTEGRAL_LIMIT, CHASSIS_INTEGRAL_LIMIT);
        s_target_rpm[i] = 0.0f;
    }

    s_last_control_ms = 0U;

#if CHASSIS_BOOT_TEST_ENABLE
    /* 上电自转: 四轮同速，验证接线和方向 */
    Chassis_SetWheelTargetRpm(CHASSIS_BOOT_TEST_RPM,
                              CHASSIS_BOOT_TEST_RPM,
                              CHASSIS_BOOT_TEST_RPM,
                              CHASSIS_BOOT_TEST_RPM);
#else
    Chassis_Stop();
#endif
}

/* 紧急停止: 清零目标转速 + 复位 PID + 清零电流 */
void Chassis_Stop(void)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        s_target_rpm[i] = 0.0f;
        Pid_Reset(&s_speed_pid[i]);
        DjiMotor_SetCurrent(s_motor_ids[i], 0);
    }
}

/* ==========================================================================
 * 目标转速设置
 * ========================================================================== */

/* 逐轮设置目标转速 (rpm)，顺序: 右前, 左前, 左后, 右后 */
void Chassis_SetWheelTargetRpm(float motor1_rpm, float motor2_rpm,
                               float motor3_rpm, float motor4_rpm)
{
    s_target_rpm[0] = motor1_rpm;
    s_target_rpm[1] = motor2_rpm;
    s_target_rpm[2] = motor3_rpm;
    s_target_rpm[3] = motor4_rpm;
}

/*
 * 麦轮逆运动学 — 车体速度 → 四轮目标转速
 *
 *   wheel1 (右前) = vx - vy - wz
 *   wheel2 (左前) = vx + vy + wz
 *   wheel3 (左后) = vx + vy - wz
 *   wheel4 (右后) = vx - vy + wz
 *
 * 参数由上位机下发的车体速度 (vx,vy,wz)，单位 rpm。
 */
void Chassis_SetVelocityRpm(float vx_rpm, float vy_rpm, float wz_rpm)
{
    Chassis_SetWheelTargetRpm(vx_rpm - vy_rpm - wz_rpm,
                              vx_rpm + vy_rpm + wz_rpm,
                              vx_rpm + vy_rpm - wz_rpm,
                              vx_rpm - vy_rpm + wz_rpm);
}

float Chassis_GetWheelTargetRpm(uint8_t wheel_index)
{
    if (wheel_index == 0U || wheel_index > CHASSIS_MOTOR_COUNT) return 0.0f;
    return s_target_rpm[wheel_index - 1U];
}

/* ==========================================================================
 * 控制循环 (1kHz)
 * ========================================================================== */

/*
 * 一次 PID 迭代:
 *   对每个轮子:
 *     ① 检查在线 (超时则电流置零、PID 复位)
 *     ② 目标转速 × 方向系数 → 与实际转速做 PID
 *     ③ PID 输出 → SetCurrent
 *   最后打包 0x200 帧发送到 FDCAN1
 */
void Chassis_ControlLoop(float dt_s)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t tx_data[8];

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        uint8_t motor_id = s_motor_ids[i];
        DjiMotorState *motor = DjiMotor_GetState(motor_id);
        int16_t current = 0;

        if (motor != 0 &&
            motor->online != 0U &&
            (now_ms - motor->last_update_ms) <= CHASSIS_OFFLINE_TIMEOUT_MS) {
            /* 在线: 速度环 PID */
            float target  = s_target_rpm[i] * (float)s_motor_dirs[i];
            float feedback = (float)motor->speed_rpm;
            current = Chassis_FloatToCurrent(
                Pid_Update(&s_speed_pid[i], target, feedback, dt_s));
        } else {
            /* 离线: 复位以避免恢复时积分冲击 */
            Pid_Reset(&s_speed_pid[i]);
        }

        DjiMotor_SetCurrent(motor_id, current);
    }

    /* 打包 4 个电机电流 → CAN ID 0x200 发送 */
    (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_1_TO_4, tx_data);
    (void)fdcanx_send_data(&hfdcan1, DJI_MOTOR_CMD_ID_1_TO_4, tx_data, 8U);
}

/*
 * 定时调度: main() 的 while(1) 中每次循环都调用，
 * 距离上次执行 ≥1ms 才实际执行一次控制迭代。
 */
void Chassis_RunPeriodic(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms;

    if (s_last_control_ms == 0U) {
        s_last_control_ms = now_ms;
        return;
    }

    elapsed_ms = now_ms - s_last_control_ms;
    if (elapsed_ms < CHASSIS_CONTROL_PERIOD_MS) return;

    s_last_control_ms = now_ms;
    Chassis_ControlLoop((float)elapsed_ms * 0.001f);
}
