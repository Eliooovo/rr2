/**
 * @file    chassis.c
 * @brief   麦轮底盘控制 — 4 x DJI M3508 (FDCAN1), 速度 PID 闭环
 *
 * 数据流:
 *   上位机 → SetVelocityRpm(vx,vy,wz) → 麦轮解算 → 目标转速
 *   C620 反馈 → DjiMotor_HandleFeedback → g_dji_motors[].speed_rpm (中断上下文)
 *   ControlLoop: 目标转速 vs 实际转速 → PID → SetCurrent → CAN 发送
 */

#include "chassis.h"

#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "pid.h"



static const ChassisMotorConfig s_motor_config[CHASSIS_MOTOR_COUNT] =
    CHASSIS_MOTOR_CONFIG_INIT;

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
                 s_motor_config[i].speed_kp,
                 s_motor_config[i].speed_ki,
                 s_motor_config[i].speed_kd,
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
        DjiMotor_SetCurrent(s_motor_config[i].motor_id, 0);
    }
}

/* ==========================================================================
 * 目标转速设置
 * ========================================================================== */

/* 逐轮设置目标转速 (rpm)，顺序: 右前, 左前, 左后, 右后 */
void Chassis_SetWheelTargetRpm(float rf_rpm, float lf_rpm,
                               float lb_rpm, float rb_rpm)
{
    s_target_rpm[CHASSIS_WHEEL_RF] = rf_rpm;
    s_target_rpm[CHASSIS_WHEEL_LF] = lf_rpm;
    s_target_rpm[CHASSIS_WHEEL_LB] = lb_rpm;
    s_target_rpm[CHASSIS_WHEEL_RB] = rb_rpm;
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

float Chassis_GetWheelTargetRpm(ChassisWheelIndex wheel)
{
    if ((uint8_t)wheel >= CHASSIS_MOTOR_COUNT) return 0.0f;
    return s_target_rpm[(uint8_t)wheel];
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
 *   最后按电调 ID 范围打包 0x200 / 0x1FF 帧发送到 FDCAN1
 */
void Chassis_ControlLoop(float dt_s)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t tx_data_1_to_4[8];
    uint8_t tx_data_5_to_8[8];
    uint8_t need_send_1_to_4 = 0U;
    uint8_t need_send_5_to_8 = 0U;

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        uint8_t motor_id = s_motor_config[i].motor_id;
        DjiMotorState *motor = DjiMotor_GetState(motor_id);
        int16_t current = 0;

        if (motor != 0 &&
            motor->online != 0U &&
            (now_ms - motor->last_update_ms) <= CHASSIS_OFFLINE_TIMEOUT_MS) {
            /* 在线: 速度环 PID */
            float target  = s_target_rpm[i] * (float)s_motor_config[i].direction;
            float feedback = (float)motor->speed_rpm;
            current = Chassis_FloatToCurrent(
                Pid_Update(&s_speed_pid[i], target, feedback, dt_s));
        } else {
            /* 离线: 复位以避免恢复时积分冲击 */
            Pid_Reset(&s_speed_pid[i]);
        }

        DjiMotor_SetCurrent(motor_id, current);
        if (motor_id >= 1U && motor_id <= 4U) {
            need_send_1_to_4 = 1U;
        } else if (motor_id >= 5U && motor_id <= 8U) {
            need_send_5_to_8 = 1U;
        }
    }

    if (need_send_1_to_4 != 0U) {
        (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_1_TO_4, tx_data_1_to_4);
        (void)fdcanx_send_data(&hfdcan1, DJI_MOTOR_CMD_ID_1_TO_4, tx_data_1_to_4, 8U);
    }

    if (need_send_5_to_8 != 0U) {
        (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_5_TO_8, tx_data_5_to_8);
        (void)fdcanx_send_data(&hfdcan1, DJI_MOTOR_CMD_ID_5_TO_8, tx_data_5_to_8, 8U);
    }
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
