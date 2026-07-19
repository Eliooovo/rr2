#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CHASSIS_MOTOR_COUNT        4U
#define CHASSIS_CONTROL_PERIOD_MS  1U
#define CHASSIS_CURRENT_LIMIT      12000.0f
#define CHASSIS_INTEGRAL_LIMIT     30000.0f
#define CHASSIS_OFFLINE_TIMEOUT_MS 100U

/* 上电默认不自动运动。需要四轮低速试转时改为 1。 */
#define CHASSIS_BOOT_TEST_ENABLE   0
#define CHASSIS_BOOT_TEST_RPM      300.0f

typedef enum {
    CHASSIS_WHEEL_RF = 0,  /* 右前 */
    CHASSIS_WHEEL_LF,      /* 左前 */
    CHASSIS_WHEEL_LB,      /* 左后 */
    CHASSIS_WHEEL_RB,      /* 右后 */
} ChassisWheelIndex;

typedef struct {
    uint8_t motor_id;      /* C620 电调 ID: 1~8，对应反馈 ID 0x201~0x208 */
    int8_t direction;      /* 方向系数: 1 或 -1 */
    float speed_kp;
    float speed_ki;
    float speed_kd;
} ChassisMotorConfig;

/* 四个轮子的实车配置，顺序必须和 ChassisWheelIndex 保持一致。 */
#define CHASSIS_MOTOR_CONFIG_INIT                       \
    {                                                   \
        {1U,  1, 6.0f, 1.0f, 0.0f}, /* RF 右前 */      \
        {2U, -1, 6.0f, 1.0f, 0.0f}, /* LF 左前 */      \
        {3U, -1, 6.0f, 1.0f, 0.0f}, /* LB 左后 */      \
        {4U,  1, 6.0f, 1.0f, 0.0f}, /* RB 右后 */      \
    }

void Chassis_Init(void);
void Chassis_Stop(void);
void Chassis_RunPeriodic(void);
/*
一次 PID 控制迭代：对 4 个轮子逐一读实际转速 → 和目标转速做 PID → 输出电流 → DjiMotor_SetCurrent() → 打包发送 CAN 帧到 FDCAN1  
*/
void Chassis_ControlLoop(float dt_s);

void Chassis_SetWheelTargetRpm(float rf_rpm,
                               float lf_rpm,
                               float lb_rpm,
                               float rb_rpm);

void Chassis_SetVelocityRpm(float vx_rpm, float vy_rpm, float wz_rpm);
float Chassis_GetWheelTargetRpm(ChassisWheelIndex wheel);

extern volatile float g_chassis_target_rpm[CHASSIS_MOTOR_COUNT];
extern volatile float g_chassis_cmd_vx;
extern volatile float g_chassis_cmd_vy;
extern volatile float g_chassis_cmd_vw;
extern volatile uint32_t g_chassis_set_velocity_count;

#ifdef __cplusplus
}
#endif

#endif
