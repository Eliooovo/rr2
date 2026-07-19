#include "lift.h"

#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "pid.h"

static const LiftMotorConfig s_lift_config[LIFT_MOTOR_COUNT] =
    LIFT_MOTOR_CONFIG_INIT;

static PidController s_position_pid[LIFT_MOTOR_COUNT];
static PidController s_speed_pid[LIFT_MOTOR_COUNT];
static float s_target_position_deg[LIFT_MOTOR_COUNT];
static float s_zero_offset_deg[LIFT_MOTOR_COUNT];
static uint8_t s_motor_enabled[LIFT_MOTOR_COUNT];
static uint32_t s_last_control_ms;
#if LIFT_BOOT_TEST_ENABLE
static uint8_t s_boot_test_started;
#endif

static float Lift_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static uint8_t Lift_IsFinite(float value)
{
    return (value == value && value < 1000000.0f && value > -1000000.0f) ? 1U : 0U;
}

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

static float Lift_GetRawPositionDeg(LiftMotorIndex motor)
{
    const LiftMotorConfig *config = &s_lift_config[(uint8_t)motor];
    DjiMotorState *state = DjiMotor_GetState(config->motor_id);

    if (state == 0) return 0.0f;
    return state->total_angle_deg;
}

#if LIFT_BOOT_TEST_ENABLE
static uint8_t Lift_AllMotorsRecentlyOnline(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        DjiMotorState *state = DjiMotor_GetState(s_lift_config[i].motor_id);

        if (state == 0 ||
            state->online == 0U ||
            (now_ms - state->last_update_ms) > LIFT_OFFLINE_TIMEOUT_MS) {
            return 0U;
        }
    }

    return 1U;
}
#endif

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

void Lift_Init(void)
{
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        Pid_Init(&s_position_pid[i],
                 s_lift_config[i].position_kp,
                 s_lift_config[i].position_ki,
                 s_lift_config[i].position_kd,
                 -s_lift_config[i].max_speed_rpm,
                 s_lift_config[i].max_speed_rpm,
                 -LIFT_POSITION_INTEGRAL_LIMIT,
                 LIFT_POSITION_INTEGRAL_LIMIT);

        Pid_Init(&s_speed_pid[i],
                 s_lift_config[i].speed_kp,
                 s_lift_config[i].speed_ki,
                 s_lift_config[i].speed_kd,
                 -s_lift_config[i].current_limit,
                 s_lift_config[i].current_limit,
                 -LIFT_SPEED_INTEGRAL_LIMIT,
                 LIFT_SPEED_INTEGRAL_LIMIT);

        s_target_position_deg[i] = 0.0f;
        s_zero_offset_deg[i] = 0.0f;
        s_motor_enabled[i] = 0U;
    }

    s_last_control_ms = 0U;
#if LIFT_BOOT_TEST_ENABLE
    s_boot_test_started = 0U;
#endif
}

void Lift_Stop(void)
{
    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        s_motor_enabled[i] = 0U;
        Pid_Reset(&s_position_pid[i]);
        Pid_Reset(&s_speed_pid[i]);
        DjiMotor_SetCurrent(s_lift_config[i].motor_id, 0);
    }

    Lift_SendCurrentFrames();
}

void Lift_SetTargetPositionDeg(LiftMotorIndex motor, float position_deg)
{
    uint8_t index = (uint8_t)motor;

    if (!Lift_IsValidIndex(motor) || !Lift_IsFinite(position_deg)) return;

    s_target_position_deg[index] = position_deg;
    s_motor_enabled[index] = 1U;
}

void Lift_SetAllTargetPositionDeg(float motor1_deg,
                                  float motor2_deg,
                                  float motor3_deg,
                                  float motor4_deg)
{
    Lift_SetTargetPositionDeg(LIFT_MOTOR_1, motor1_deg);
    Lift_SetTargetPositionDeg(LIFT_MOTOR_2, motor2_deg);
    Lift_SetTargetPositionDeg(LIFT_MOTOR_3, motor3_deg);
    Lift_SetTargetPositionDeg(LIFT_MOTOR_4, motor4_deg);
}

void Lift_DisableMotor(LiftMotorIndex motor)
{
    uint8_t index = (uint8_t)motor;

    if (!Lift_IsValidIndex(motor)) return;

    s_motor_enabled[index] = 0U;
    Pid_Reset(&s_position_pid[index]);
    Pid_Reset(&s_speed_pid[index]);
    DjiMotor_SetCurrent(s_lift_config[index].motor_id, 0);
}

void Lift_SetZeroToCurrent(LiftMotorIndex motor)
{
    uint8_t index = (uint8_t)motor;

    if (!Lift_IsValidIndex(motor)) return;

    s_zero_offset_deg[index] = Lift_GetRawPositionDeg(motor);
    s_target_position_deg[index] = 0.0f;
    Pid_Reset(&s_position_pid[index]);
    Pid_Reset(&s_speed_pid[index]);
}

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

void Lift_ControlLoop(float dt_s)
{
    uint32_t now_ms = HAL_GetTick();

    for (uint8_t i = 0U; i < LIFT_MOTOR_COUNT; ++i) {
        const LiftMotorConfig *config = &s_lift_config[i];
        DjiMotorState *state = DjiMotor_GetState(config->motor_id);
        int16_t current = 0;

        if (s_motor_enabled[i] != 0U &&
            state != 0 &&
            state->online != 0U &&
            (now_ms - state->last_update_ms) <= LIFT_OFFLINE_TIMEOUT_MS) {
            /* 位置环输出目标速度，速度环输出 C620 电流指令。 */
            float position_deg = Lift_GetPositionDeg((LiftMotorIndex)i);
            float target_speed_rpm = Pid_Update(&s_position_pid[i],
                                                s_target_position_deg[i],
                                                position_deg,
                                                dt_s);
            float raw_target_speed_rpm = target_speed_rpm * (float)config->direction;
            float raw_current = Pid_Update(&s_speed_pid[i],
                                           raw_target_speed_rpm,
                                           (float)state->speed_rpm,
                                           dt_s);

            current = Lift_FloatToCurrent(raw_current, config->current_limit);
        } else {
            Pid_Reset(&s_position_pid[i]);
            Pid_Reset(&s_speed_pid[i]);
        }

        DjiMotor_SetCurrent(config->motor_id, current);
    }

    Lift_SendCurrentFrames();
}

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
