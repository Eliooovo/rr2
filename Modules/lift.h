#ifndef LIFT_H
#define LIFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LIFT_MOTOR_COUNT             4U
#define LIFT_CONTROL_PERIOD_MS       1U
#define LIFT_OFFLINE_TIMEOUT_MS      100U
#define LIFT_POSITION_INTEGRAL_LIMIT 0.0f
#define LIFT_SPEED_INTEGRAL_LIMIT    30000.0f

/* 上电默认不自动运动；需要低速位置测试时再改为 1。 */
#define LIFT_BOOT_TEST_ENABLE        1
#define LIFT_BOOT_TEST_DEG           180.0f

typedef enum {
    LIFT_MOTOR_1 = 0,
    LIFT_MOTOR_2,
    LIFT_MOTOR_3,
    LIFT_MOTOR_4,
} LiftMotorIndex;

typedef struct {
    uint8_t motor_id;       /* C620 电调 ID: 1~8 */
    int8_t direction;       /* 位置正方向: 1 或 -1 */
    float position_kp;
    float position_ki;
    float position_kd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float max_speed_rpm;
    float current_limit;
} LiftMotorConfig;

/* 调试阶段用 FDCAN2 上 ID 1~4；最终装车如果改 ID，只改这里。 */
#define LIFT_MOTOR_CONFIG_INIT                                      \
    {                                                               \
        {1U, 1, 1.0f, 0.0f, 0.0f, 6.0f, 1.0f, 0.0f, 300.0f, 5000.0f}, \
        {2U, 1, 1.0f, 0.0f, 0.0f, 6.0f, 1.0f, 0.0f, 300.0f, 5000.0f}, \
        {3U, 1, 1.0f, 0.0f, 0.0f, 6.0f, 1.0f, 0.0f, 300.0f, 5000.0f}, \
        {4U, 1, 1.0f, 0.0f, 0.0f, 6.0f, 1.0f, 0.0f, 300.0f, 5000.0f}, \
    }

void Lift_Init(void);
void Lift_RunPeriodic(void);
void Lift_ControlLoop(float dt_s);
void Lift_Stop(void);

void Lift_SetTargetPositionDeg(LiftMotorIndex motor, float position_deg);
void Lift_SetAllTargetPositionDeg(float motor1_deg,
                                  float motor2_deg,
                                  float motor3_deg,
                                  float motor4_deg);
void Lift_DisableMotor(LiftMotorIndex motor);
void Lift_SetZeroToCurrent(LiftMotorIndex motor);

float Lift_GetPositionDeg(LiftMotorIndex motor);
float Lift_GetTargetPositionDeg(LiftMotorIndex motor);

#ifdef __cplusplus
}
#endif

#endif
