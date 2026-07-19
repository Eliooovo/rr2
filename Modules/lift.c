/**
 * @file    lift.c
 * @brief   升降机构 — 串级 PID (位置环 + 速度环), FDCAN2 发送
 *
 * 控制流:
 *   target_deg → 位置 PID → target_rpm → 速度 PID → C620 电流 → CAN 0x200/0x1FF → FDCAN2
 *   反馈: C620 → FDCAN2 RX中断 → DjiMotor_HandleFeedback → g_dji_motors[].speed_rpm / total_angle_deg
 */

#include "lift.h"

#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "pid.h"

/* 电机配置表 (来自 lift.h 的宏展开) */
static const LiftMotorConfig s_lift_config[LIFT_MOTOR_COUNT] =
    LIFT_MOTOR_CONFIG_INIT;

/* 位置环按“前/后两组”控制，速度环仍按单电机控制。
 * 两个电机共同驱动一个抬升时，位置 PID 看组平均位置，避免两个独立位置环互相打架。 */
static PidController s_pair_position_pid[LIFT_PAIR_COUNT];
static PidController s_speed_pid[LIFT_MOTOR_COUNT];

static float    s_target_position_deg[LIFT_MOTOR_COUNT]; /* 目标位置 (度) */
static float    s_zero_offset_deg[LIFT_MOTOR_COUNT];     /* 零点偏移: 有效位置 = raw - offset */
static uint8_t  s_motor_enabled[LIFT_MOTOR_COUNT];       /* 0=禁用, 1=使能 */
static uint32_t s_last_control_ms;                       /* 上次控制时刻 */

#if LIFT_BOOT_TEST_ENABLE
static uint8_t  s_boot_test_started;                     /* boot test 是否已触发 */
#endif

/* ==========================================================================
 * 工具函数
 * ========================================================================== */

static float Lift_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

/* 过滤 NaN 和异常大值 */
static uint8_t Lift_IsFinite(float value)
{
    return (value == value && value < 1000000.0f && value > -1000000.0f) ? 1U : 0U;
}

/* float → int16_t, 四舍五入 + 限幅 */
static int16_t Lift_FloatToCurrent(float value, float current_limit)
{
    value = Lift_Clamp(value, -current_limit, current_limit);
    if (value >= 0.0f) return (int16_t)(value + 0.5f);
    return (int16_t)(value - 0.5f);
}

static uint8_t Lift_IsValidIndex(LiftMotorIndex motor)
{
    return ((uint8_t)motor < LIFT_MOTOR_COUNT) ? 1U : 0U;
}

/* 电机 1/2 为前组，电机 3/4 为后组。 */
static uint8_t Lift_GetPairIndex(LiftMotorIndex motor)
{
    return ((uint8_t)motor < 2U) ? 0U : 1U;
}

static LiftMotorIndex Lift_GetPairMotorA(uint8_t pair_index)
{
    return (pair_index == 0U) ? LIFT_MOTOR_1 : LIFT_MOTOR_3;
}

static LiftMotorIndex Lift_GetPairMotorB(uint8_t pair_index)
{
    return (pair_index == 0U) ? LIFT_MOTOR_2 : LIFT_MOTOR_4;
}

/* 读取电机的原始多圈角度 (未减零点) */
static float Lift_GetRawPositionDeg(LiftMotorIndex motor)
{
    const LiftMotorConfig *config = &s_lift_config[(uint8_t)motor];
    DjiMotorState *state = DjiMotor_GetState(config->motor_id);
    if (state == 0) return 0.0f;
    return state->total_angle_deg;
}

static uint8_t Lift_MotorOnlineRecently(LiftMotorIndex motor, uint32_t now_ms)
{
    const LiftMotorConfig *config = &s_lift_config[(uint8_t)motor];
    DjiMotorState *state = DjiMotor_GetState(config->motor_id);

    return (state != 0 &&
            state->online != 0U &&
            (now_ms - state->last_update_ms) <= LIFT_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static uint8_t Lift_PairReady(uint8_t pair_index, uint32_t now_ms)
{
    LiftMotorIndex motor_a = Lift_GetPairMotorA(pair_index);
    LiftMotorIndex motor_b = Lift_GetPairMotorB(pair_index);

    return (s_motor_enabled[(uint8_t)motor_a] != 0U &&
            s_motor_enabled[(uint8_t)motor_b] != 0U &&
            Lift_MotorOnlineRecently(motor_a, now_ms) != 0U &&
            Lift_MotorOnlineRecently(motor_b, now_ms) != 0U) ? 1U : 0U;
}

/* 复位一组的位置 PID。单个电机禁用、离线或重新设零点时都要复位，
 * 防止恢复时位置环带着旧误差直接冲击速度目标。 */
static void Lift_ResetPairPositionPid(uint8_t pair_index)
{
    if (pair_index < LIFT_PAIR_COUNT) {
        Pid_Reset(&s_pair_position_pid[pair_index]);
    }
}

#if LIFT_BOOT_TEST_ENABLE
/* 检查四台电机是否全部在线 (boot test 启动条件) */
static uint8_t Lift_AllMotorsRecentlyOnline(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        DjiMotorState *state = DjiMotor_GetState(s_lift_config[i].motor_id);
        if (state == 0 || state->online == 0U ||
            (now_ms - state->last_update_ms) > LIFT_OFFLINE_TIMEOUT_MS) {
            return 0U;
        }
    }
    return 1U;
}
#endif

/* 发送电流帧到 FDCAN2。
 * 根据电机 ID 决定发 0x200 (ID 1~4) 还是 0x1FF (ID 5~8) */
static void Lift_SendCurrentFrames(void)
{
    uint8_t data_1_to_4[8];
    uint8_t data_5_to_8[8];
    uint8_t need_send_1_to_4 = 0U;
    uint8_t need_send_5_to_8 = 0U;

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        uint8_t motor_id = s_lift_config[i].motor_id;
        if (motor_id >= 1U && motor_id <= 4U) {
            need_send_1_to_4 = 1U;
        } else if (motor_id >= 5U && motor_id <= 8U) {
            need_send_5_to_8 = 1U;
        }
    }

    if (need_send_1_to_4 != 0U) {
        (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_1_TO_4, data_1_to_4);
        (void)fdcanx_send_data(&hfdcan2, DJI_MOTOR_CMD_ID_1_TO_4, data_1_to_4, 8U);
    }

    if (need_send_5_to_8 != 0U) {
        (void)DjiMotor_BuildCurrentFrame(DJI_MOTOR_CMD_ID_5_TO_8, data_5_to_8);
        (void)fdcanx_send_data(&hfdcan2, DJI_MOTOR_CMD_ID_5_TO_8, data_5_to_8, 8U);
    }
}

/* ==========================================================================
 * 初始化 & 停止
 * ========================================================================== */

void Lift_Init(void)
{
    for (uint8_t pair = 0U; pair < LIFT_PAIR_COUNT; ++pair) {
        uint8_t config_index = (pair == 0U) ? (uint8_t)LIFT_MOTOR_1 : (uint8_t)LIFT_MOTOR_3;

        /* 每组一个位置环: 前组用电机1的参数，后组用电机3的参数。
         * 组内两台电机的 position/max_speed 参数应保持一致。 */
        Pid_Init(&s_pair_position_pid[pair],
                 s_lift_config[config_index].position_kp,
                 s_lift_config[config_index].position_ki,
                 s_lift_config[config_index].position_kd,
                 -s_lift_config[config_index].max_speed_rpm,
                  s_lift_config[config_index].max_speed_rpm,
                 -LIFT_POSITION_INTEGRAL_LIMIT,
                  LIFT_POSITION_INTEGRAL_LIMIT);
    }

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        /* 速度环: 输出电流指令，限幅 ±current_limit */
        Pid_Init(&s_speed_pid[i],
                 s_lift_config[i].speed_kp,
                 s_lift_config[i].speed_ki,
                 s_lift_config[i].speed_kd,
                 -s_lift_config[i].current_limit,
                  s_lift_config[i].current_limit,
                 -LIFT_SPEED_INTEGRAL_LIMIT,
                  LIFT_SPEED_INTEGRAL_LIMIT);

        s_target_position_deg[i] = 0.0f;
        s_zero_offset_deg[i]     = 0.0f;
        s_motor_enabled[i]       = 0U;
    }

    s_last_control_ms = 0U;
#if LIFT_BOOT_TEST_ENABLE
    s_boot_test_started = 0U;   /* 等 RunPeriodic 中电机全部在线后再触发 */
#endif
}

/* 紧急停止 */
void Lift_Stop(void)
{
    for (uint8_t pair = 0U; pair < LIFT_PAIR_COUNT; ++pair) {
        Lift_ResetPairPositionPid(pair);
    }

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        s_motor_enabled[i] = 0U;
        Pid_Reset(&s_speed_pid[i]);
        DjiMotor_SetCurrent(s_lift_config[i].motor_id, 0);
    }
    Lift_SendCurrentFrames();
}

/* ==========================================================================
 * 位置指令
 * ========================================================================== */

/* 设目标位置并立即使能电机 */
void Lift_SetTargetPositionDeg(LiftMotorIndex motor, float position_deg)
{
    uint8_t index = (uint8_t)motor;
    if (!Lift_IsValidIndex(motor) || !Lift_IsFinite(position_deg)) return;

    s_target_position_deg[index] = position_deg;
    s_motor_enabled[index]       = 1U;
}

/* 四台同时设 */
void Lift_SetAllTargetPositionDeg(float motor1_deg, float motor2_deg,
                                  float motor3_deg, float motor4_deg)
{
    Lift_SetTargetPositionDeg(LIFT_MOTOR_1, motor1_deg);
    Lift_SetTargetPositionDeg(LIFT_MOTOR_2, motor2_deg);
    Lift_SetTargetPositionDeg(LIFT_MOTOR_3, motor3_deg);
    Lift_SetTargetPositionDeg(LIFT_MOTOR_4, motor4_deg);
}

/* 禁用单台电机，电流清零，PID 复位 */
void Lift_DisableMotor(LiftMotorIndex motor)
{
    uint8_t index = (uint8_t)motor;
    if (!Lift_IsValidIndex(motor)) return;

    s_motor_enabled[index] = 0U;
    Lift_ResetPairPositionPid(Lift_GetPairIndex(motor));
    Pid_Reset(&s_speed_pid[index]);
    DjiMotor_SetCurrent(s_lift_config[index].motor_id, 0);
}

/* 将当前位置设为零点: effective_position = raw - offset = 0
 * 同时将目标也设为 0、PID 复位 */
void Lift_SetZeroToCurrent(LiftMotorIndex motor)
{
    uint8_t index = (uint8_t)motor;
    if (!Lift_IsValidIndex(motor)) return;

    s_zero_offset_deg[index]     = Lift_GetRawPositionDeg(motor);
    s_target_position_deg[index] = 0.0f;
    Lift_ResetPairPositionPid(Lift_GetPairIndex(motor));
    Pid_Reset(&s_speed_pid[index]);
}

/* ==========================================================================
 * 状态查询
 * ========================================================================== */

/* 有效位置 = (原始多圈角度 - 零点偏移) × 方向系数 */
float Lift_GetPositionDeg(LiftMotorIndex motor)
{
    uint8_t index = (uint8_t)motor;
    if (!Lift_IsValidIndex(motor)) return 0.0f;

    return (Lift_GetRawPositionDeg(motor) - s_zero_offset_deg[index]) *
           (float)s_lift_config[index].direction;
}

float Lift_GetTargetPositionDeg(LiftMotorIndex motor)
{
    if (!Lift_IsValidIndex(motor)) return 0.0f;
    return s_target_position_deg[(uint8_t)motor];
}

/* ==========================================================================
 * 串级 PID 控制循环 (1kHz)
 *
 *   组平均位置误差 → 组位置 PID → 组目标速度 (rpm)
 *   组内位置差 → 同步补偿 → 两台电机各自目标速度
 *   单电机速度误差 → 单电机速度 PID → 电流指令 → SetCurrent
 *   最后打包发送到 FDCAN2
 *
 * 离线保护: 一组里任意电机离线/禁用时，该组两个电机都清零。
 * ========================================================================== */
void Lift_ControlLoop(float dt_s)
{
    uint32_t now_ms = HAL_GetTick();

    for (uint8_t pair = 0U; pair < LIFT_PAIR_COUNT; ++pair) {
        LiftMotorIndex motor_a = Lift_GetPairMotorA(pair);
        LiftMotorIndex motor_b = Lift_GetPairMotorB(pair);
        uint8_t index_a = (uint8_t)motor_a;
        uint8_t index_b = (uint8_t)motor_b;
        const LiftMotorConfig *config_a = &s_lift_config[index_a];
        const LiftMotorConfig *config_b = &s_lift_config[index_b];
        DjiMotorState *state_a = DjiMotor_GetState(config_a->motor_id);
        DjiMotorState *state_b = DjiMotor_GetState(config_b->motor_id);
        int16_t current_a = 0;
        int16_t current_b = 0;

        if (Lift_PairReady(pair, now_ms) != 0U) {
            float position_a = Lift_GetPositionDeg(motor_a);
            float position_b = Lift_GetPositionDeg(motor_b);
            float target_pair = (s_target_position_deg[index_a] +
                                 s_target_position_deg[index_b]) * 0.5f;
            float position_pair = (position_a + position_b) * 0.5f;

            /* 组位置环只看平均位置，保证一组两个电机以同一个目标速度启动/停止。 */
            float pair_target_rpm = Pid_Update(&s_pair_position_pid[pair],
                                               target_pair,
                                               position_pair,
                                               dt_s);

            /* 同步补偿只管组内位置差:
             * A 比 B 位置大时，A 的目标速度减小，B 的目标速度增大，让两边追平。 */
            float sync_error_deg = position_a - position_b;
            float sync_rpm = Lift_Clamp(sync_error_deg * LIFT_PAIR_SYNC_KP,
                                        -LIFT_PAIR_SYNC_MAX_RPM,
                                         LIFT_PAIR_SYNC_MAX_RPM);
            float target_rpm_a = (pair_target_rpm - sync_rpm) * (float)config_a->direction;
            float target_rpm_b = (pair_target_rpm + sync_rpm) * (float)config_b->direction;
            float raw_current_a = Pid_Update(&s_speed_pid[index_a],
                                             target_rpm_a,
                                             (float)state_a->speed_rpm,
                                             dt_s);
            float raw_current_b = Pid_Update(&s_speed_pid[index_b],
                                             target_rpm_b,
                                             (float)state_b->speed_rpm,
                                             dt_s);

            current_a = Lift_FloatToCurrent(raw_current_a, config_a->current_limit);
            current_b = Lift_FloatToCurrent(raw_current_b, config_b->current_limit);
        } else {
            /* 一组中只要有电机离线或禁用，就整组清零，避免单边拉抬升机构。 */
            Lift_ResetPairPositionPid(pair);
            Pid_Reset(&s_speed_pid[index_a]);
            Pid_Reset(&s_speed_pid[index_b]);
        }

        DjiMotor_SetCurrent(config_a->motor_id, current_a);
        DjiMotor_SetCurrent(config_b->motor_id, current_b);
    }

    Lift_SendCurrentFrames();
}

/* 定时调度: main() 中每次循环调用，≥1ms 执行一次控制迭代。
 * boot test 模式下，等四台电机全部在线后自动设零点并开始运动。 */
void Lift_RunPeriodic(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms;

    if (s_last_control_ms == 0U) {
        s_last_control_ms = now_ms;
        return;
    }

    elapsed_ms = now_ms - s_last_control_ms;
    if (elapsed_ms < LIFT_CONTROL_PERIOD_MS) return;

    s_last_control_ms = now_ms;

#if LIFT_BOOT_TEST_ENABLE
    /* 等四台电机全部在线后，以当前位置为零点，运动到 LIFT_BOOT_TEST_DEG。 */
    if (s_boot_test_started == 0U && Lift_AllMotorsRecentlyOnline(now_ms) != 0U) {
        Lift_SetZeroToCurrent(LIFT_MOTOR_1);
        Lift_SetZeroToCurrent(LIFT_MOTOR_2);
        Lift_SetZeroToCurrent(LIFT_MOTOR_3);
        Lift_SetZeroToCurrent(LIFT_MOTOR_4);
        Lift_SetAllTargetPositionDeg(LIFT_BOOT_TEST_DEG,
                                     LIFT_BOOT_TEST_DEG,
                                     LIFT_BOOT_TEST_DEG,
                                     LIFT_BOOT_TEST_DEG);
        s_boot_test_started = 1U;
    }
#endif

    Lift_ControlLoop((float)elapsed_ms * 0.001f);
}
