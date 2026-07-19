/**
 * @file    lift.h
 * @brief   升降机构控制 — 4 x DJI M3508 (FDCAN2), 串级位置-速度 PID
 *
 * 控制结构: 位置环 (外环) → 目标速度 → 速度环 (内环) → C620 电流指令
 * 电机 ID: FDCAN2 上的 1~4 (与 FDCAN1 的底盘电机是不同物理总线，ID 可重复)
 */

#ifndef LIFT_H
#define LIFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LIFT_MOTOR_COUNT             4U     /* 升降电机数量 */
#define LIFT_CONTROL_PERIOD_MS       1U     /* 控制周期 1ms (1kHz) */
#define LIFT_OFFLINE_TIMEOUT_MS      100U   /* 电机超时判定离线 */
#define LIFT_POSITION_INTEGRAL_LIMIT 0.0f   /* 位置环积分限幅 (0=禁用) */
#define LIFT_SPEED_INTEGRAL_LIMIT    30000.0f
#define LIFT_PAIR_COUNT              2U     /* 前/后两组升降，每组 2 台电机 */
#define LIFT_PAIR_SYNC_KP            5.0f   /* 组内同步 P: 位置差(度) → 速度修正(rpm) */
#define LIFT_PAIR_SYNC_MAX_RPM       200.0f /* 组内同步修正限幅，避免单边补偿过猛 */

/* 上电自测: 等待电机全部在线后，以当前位置为零点，运动到 BOOT_TEST_DEG 度 */
#define LIFT_BOOT_TEST_ENABLE        0
#define LIFT_BOOT_TEST_DEG           3600.0f

/* 电机索引枚举，方便传参时区分四个电机 */
typedef enum {
    LIFT_MOTOR_1 = 0,
    LIFT_MOTOR_2,
    LIFT_MOTOR_3,
    LIFT_MOTOR_4,
} LiftMotorIndex;

/* 单台升降电机的完整配置 */
typedef struct {
    uint8_t motor_id;       /* C620 电调 ID (1~8) */
    int8_t  direction;      /* 位置正方向: 1 或 -1，试车时按需翻转 */
    float   position_kp;    /* 位置环 P */
    float   position_ki;    /* 位置环 I */
    float   position_kd;    /* 位置环 D */
    float   speed_kp;       /* 速度环 P */
    float   speed_ki;       /* 速度环 I */
    float   speed_kd;       /* 速度环 D */
    float   max_speed_rpm;  /* 位置环输出限幅 (rpm)，即最大运动速度 */
    float   current_limit;  /* 电流输出限幅，C620 最大 16384 */
} LiftMotorConfig;

/* 四台电机配置表，修改 PID 参数改这里 */
#define LIFT_MOTOR_CONFIG_INIT                                      \
    {                                                               \
        {1U, 1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 300.0f, 5000.0f}, \
        {2U, 1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 300.0f, 5000.0f}, \
        {3U, 1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 300.0f, 5000.0f}, \
        {4U, 1, 6.5f, 0.0f, 0.0f, 6.0f, 0.5f, 0.0f, 300.0f, 5000.0f}, \
    }

/* ==========================================================================
 * 初始化 & 运行
 * ========================================================================== */

void Lift_Init(void);              /* 初始化 PID、状态、boot test 标记 */
void Lift_RunPeriodic(void);       /* 在 main() while(1) 中调用，1ms 执行一次控制 */
void Lift_ControlLoop(float dt_s); /* 一次控制迭代 (由 RunPeriodic 内部调用) */
void Lift_Stop(void);              /* 紧急停止: 禁用所有电机 + 电流清零 */

/* ==========================================================================
 * 位置指令
 * ========================================================================== */

void Lift_SetTargetPositionDeg(LiftMotorIndex motor, float position_deg);
void Lift_SetAllTargetPositionDeg(float m1_deg, float m2_deg,
                                  float m3_deg, float m4_deg);

/* 禁用/使能 */
void Lift_DisableMotor(LiftMotorIndex motor);

/* 将当前位置记为零点 (会复位 PID) */
void Lift_SetZeroToCurrent(LiftMotorIndex motor);

/* ==========================================================================
 * 状态查询
 * ========================================================================== */

float Lift_GetPositionDeg(LiftMotorIndex motor);       /* 有效位置 = (原始角度 - 零点) × 方向 */
float Lift_GetTargetPositionDeg(LiftMotorIndex motor); /* 当前目标位置 */

#ifdef __cplusplus
}
#endif

#endif
