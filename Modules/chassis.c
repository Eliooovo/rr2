/**
 * @file    chassis.c
 * @brief   麦轮底盘控制 — 4 x DJI M3508 (FDCAN1), 速度 PID 闭环
 *
 * 数据流:
 *   上位机 → SetVelocity(vx,vy,wz) → 麦轮解算/单位换算 → 目标转速
 *   C620 反馈 → DjiMotor_HandleFeedback → g_dji_motors[].speed_rpm (中断上下文)
 *   ControlLoop: 目标转速 vs 实际转速 → PID → SetCurrent → CAN 发送
 */

#include "chassis.h"

#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "pid.h"


/* 四个轮子电机的配置，顺序必须和 ChassisWheelIndex 保持一致。 */
static const ChassisMotorConfig s_motor_config[CHASSIS_MOTOR_COUNT] = CHASSIS_MOTOR_CONFIG_INIT;
static PidController s_speed_pid[CHASSIS_MOTOR_COUNT];
static float        s_target_rpm[CHASSIS_MOTOR_COUNT];
static uint32_t     s_last_control_ms;

/* ==========================================================================
 * 上电跑车测试，只在 CHASSIS_BOOT_TEST_ENABLE=1 时启用
 * ========================================================================== */
#if CHASSIS_BOOT_TEST_ENABLE
static uint8_t      s_boot_test_active; // 上电跑车测试是否激活
static uint8_t      s_boot_test_started; // 上电跑车测试是否已开始
static uint32_t     s_boot_test_start_ms; // 上电跑车测试开始时间
#endif

/* Ozone 调试用:
 * g_chassis_cmd_* 记录最近一次速度命令输入；
 * g_chassis_target_rpm[] 记录换算后的四轮 C620 反馈侧目标 rpm，顺序: 右前、左前、左后、右后。 */
volatile float g_chassis_target_rpm[CHASSIS_MOTOR_COUNT];
volatile float g_chassis_cmd_vx;
volatile float g_chassis_cmd_vy;
volatile float g_chassis_cmd_vw;
volatile float g_chassis_cmd_vx_mps;
volatile float g_chassis_cmd_vy_mps;
volatile float g_chassis_cmd_wz_radps;
volatile uint32_t g_chassis_set_velocity_count;

/* float 电流值 → int16_t，自动限幅 */
static int16_t Chassis_FloatToCurrent(float value)
{
    if (value > CHASSIS_CURRENT_LIMIT)  return (int16_t)CHASSIS_CURRENT_LIMIT;
    if (value < -CHASSIS_CURRENT_LIMIT) return (int16_t)-CHASSIS_CURRENT_LIMIT;
    if (value >= 0.0f) return (int16_t)(value + 0.5f);
    return (int16_t)(value - 0.5f);
}

#if CHASSIS_LOW_SPEED_COMP_ENABLE
/* 低速补偿只在“目标速度很小但明确不为 0”时生效。
 * 方向跟随 target_rpm，不跟随 PID 输出，避免被阻挡时 PID 输出暂时过小导致补偿方向丢失。 */
static float Chassis_ApplyLowSpeedCompensation(float target_rpm, float current_cmd)
{
    float abs_target = (target_rpm >= 0.0f) ? target_rpm : -target_rpm;
    float abs_current = (current_cmd >= 0.0f) ? current_cmd : -current_cmd;

    if (abs_target < CHASSIS_TARGET_DEADBAND_RPM || abs_target > CHASSIS_LOW_SPEED_RPM) {
        return current_cmd;
    }

    if (abs_current >= CHASSIS_MIN_DRIVE_CURRENT) {
        return current_cmd;
    }

    return (target_rpm >= 0.0f) ? CHASSIS_MIN_DRIVE_CURRENT : -CHASSIS_MIN_DRIVE_CURRENT;
}
#endif

#if CHASSIS_BOOT_TEST_ENABLE
/* 上电跑车测试只在四个底盘电机都持续有反馈后开始计时。
 * 如果未在线就开始计时，前面可能只是发了目标但电机没有真正闭环运行。 */
static uint8_t Chassis_AllMotorsRecentlyOnline(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        DjiMotorState *motor = DjiMotor_GetState(s_motor_config[i].motor_id);

        if (motor == 0 ||
            motor->online == 0U ||
            (now_ms - motor->last_update_ms) > CHASSIS_OFFLINE_TIMEOUT_MS) {
            return 0U;
        }
    }

    return 1U;
}
#endif

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
        g_chassis_target_rpm[i] = 0.0f;
    }
    g_chassis_cmd_vx = 0.0f;
    g_chassis_cmd_vy = 0.0f;
    g_chassis_cmd_vw = 0.0f;
    g_chassis_cmd_vx_mps = 0.0f;
    g_chassis_cmd_vy_mps = 0.0f;
    g_chassis_cmd_wz_radps = 0.0f;
    g_chassis_set_velocity_count = 0U;

    s_last_control_ms = 0U;

#if CHASSIS_BOOT_TEST_ENABLE
    /* 上电跑车测试由 Chassis_RunPeriodic() 在电机全部在线后启动。 */
    s_boot_test_active = 1U;
    s_boot_test_started = 0U;
    s_boot_test_start_ms = 0U;
#else
    Chassis_Stop();
#endif
}

/* 紧急停止: 清零目标转速 + 复位 PID + 清零电流 */
void Chassis_Stop(void)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; ++i) {
        s_target_rpm[i] = 0.0f;
        g_chassis_target_rpm[i] = 0.0f;
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

    g_chassis_target_rpm[CHASSIS_WHEEL_RF] = rf_rpm;
    g_chassis_target_rpm[CHASSIS_WHEEL_LF] = lf_rpm;
    g_chassis_target_rpm[CHASSIS_WHEEL_LB] = lb_rpm;
    g_chassis_target_rpm[CHASSIS_WHEEL_RB] = rb_rpm;
}

/*
 * 麦轮逆运动学 — 车体物理速度 → 四轮电机反馈侧目标转速
 *
 * 上位机输入:
 *   vx_mps     前后方向速度, m/s
 *   vy_mps     左右方向速度, m/s
 *   wz_radps   车体 yaw 角速度, rad/s
 *
 * 先算每个轮子的线速度:
 *   wheel_linear = vx ± vy ± (Lx + Ly) * wz
 *
 * 再把轮子线速度换成 C620 反馈侧 rpm:
 *   wheel_rpm = wheel_linear / (2πR) * 60
 *   motor_rpm = wheel_rpm * reduction_ratio
 *
 * 注意: 速度 PID 比较的是 DjiMotorState.speed_rpm，即电机反馈侧 rpm，
 * 所以必须乘减速比 16。否则 1.5m/s 会被当成 1.5rpm，电机基本不动。
 */
void Chassis_SetVelocity(float vx_mps, float vy_mps, float wz_radps)
{
    const float rotation_radius_m = CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M;
    const float rpm_per_mps =
        (60.0f * CHASSIS_MOTOR_REDUCTION_RATIO) /
        (2.0f * CHASSIS_PI * CHASSIS_WHEEL_RADIUS_M);

    /* 调试阶段限速: 保持上位机协议单位不变，只在底盘内部降低实际执行速度。 */
   vx_mps   *= CHASSIS_VX_SCALE;
   vy_mps   *= CHASSIS_VY_SCALE;
   wz_radps *= CHASSIS_VW_SCALE;

    float rf_linear = vx_mps - vy_mps - rotation_radius_m * wz_radps;
    float lf_linear = vx_mps + vy_mps + rotation_radius_m * wz_radps;
    float lb_linear = vx_mps - vy_mps + rotation_radius_m * wz_radps;
    float rb_linear = vx_mps + vy_mps - rotation_radius_m * wz_radps;

    g_chassis_cmd_vx_mps = vx_mps;
    g_chassis_cmd_vy_mps = vy_mps;
    g_chassis_cmd_wz_radps = wz_radps;

    g_chassis_set_velocity_count++;
    g_chassis_cmd_vx = vx_mps;
    g_chassis_cmd_vy = vy_mps;
    g_chassis_cmd_vw = wz_radps;

    Chassis_SetWheelTargetRpm(rf_linear * rpm_per_mps,
                              lf_linear * rpm_per_mps,
                              lb_linear * rpm_per_mps,
                              rb_linear * rpm_per_mps);
}

/*
 * 麦轮逆运动学 — 抽象 rpm 命令 → 四轮目标转速
 *
 *   wheel1 (右前) = vx - vy - wz
 *   wheel2 (左前) = vx + vy + wz
 *   wheel3 (左后) = vx + vy - wz
 *   wheel4 (右后) = vx - vy + wz
 *
 * 参数单位是 rpm，仅用于直接按 rpm 调试；USB 上位机 m/s/rad/s 应调用
 * Chassis_SetVelocity()。
 */
void Chassis_SetVelocityRpm(float vx_rpm, float vy_rpm, float wz_rpm)
{
    g_chassis_set_velocity_count++;
    g_chassis_cmd_vx = vx_rpm;
    g_chassis_cmd_vy = vy_rpm;
    g_chassis_cmd_vw = wz_rpm;

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
            float current_cmd = Pid_Update(&s_speed_pid[i], target, feedback, dt_s);
#if CHASSIS_LOW_SPEED_COMP_ENABLE
            current_cmd = Chassis_ApplyLowSpeedCompensation(target, current_cmd);
#endif
            current = Chassis_FloatToCurrent(current_cmd);
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
    uint32_t now_ms = HAL_GetTick(); //当前系统时间 (ms)，用于计算 dt_s 和记录 last_update_ms
    uint32_t elapsed_ms;

    if (s_last_control_ms == 0U) {
        s_last_control_ms = now_ms;//若是第一次跑这个函数，则初始化上次执行时间为当前时间，避免 dt_s 过大
        return;
    }

    elapsed_ms = now_ms - s_last_control_ms;
    if (elapsed_ms < CHASSIS_CONTROL_PERIOD_MS) return;

    s_last_control_ms = now_ms;

#if CHASSIS_BOOT_TEST_ENABLE
    if (s_boot_test_active != 0U) {
        if (s_boot_test_started == 0U) {
            if (Chassis_AllMotorsRecentlyOnline(now_ms) != 0U) {
                /* 四个电机都在线后才开始跑，并从此刻开始计 1 秒。 */
                Chassis_SetVelocity(CHASSIS_BOOT_TEST_VX_MPS,
                                    CHASSIS_BOOT_TEST_VY_MPS,
                                    CHASSIS_BOOT_TEST_WZ_RADPS);
                s_boot_test_start_ms = now_ms;
                s_boot_test_started = 1U;
            }
        } else if ((now_ms - s_boot_test_start_ms) >= CHASSIS_BOOT_TEST_DURATION_MS) {
            Chassis_Stop();
            s_boot_test_active = 0U;
        }
    }
#endif

    Chassis_ControlLoop((float)elapsed_ms * 0.001f);
}
