#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 低速最小电流补偿:
     * 用于 0.01m/s 这类很低速测试。低速时 PID 输出可能不足以克服静摩擦，
     * 导致轮子时转时停；开启后，在目标转速很小但非零时给一个最小电流。
     *
     * 调参建议:
     *   - MIN_DRIVE_CURRENT 从 300~800 试起，太小没效果，太大低速会冲
     *   - LOW_SPEED_RPM 应覆盖 0.01m/s 对应的约 26rpm
     *   - TARGET_DEADBAND_RPM 用来避免目标接近 0 时误补偿
     * */
#define CHASSIS_LOW_SPEED_COMP_ENABLE       1U
#define CHASSIS_LOW_SPEED_RPM               40.0f
#define CHASSIS_TARGET_DEADBAND_RPM         1.0f
#define CHASSIS_MIN_DRIVE_CURRENT           300.0f


#define CHASSIS_MOTOR_COUNT        4U
#define CHASSIS_CONTROL_PERIOD_MS  1U
#define CHASSIS_CURRENT_LIMIT      12000.0f
#define CHASSIS_INTEGRAL_LIMIT     30000.0f
#define CHASSIS_OFFLINE_TIMEOUT_MS 100U

/* 底盘物理参数，用于把上位机的 m/s、rad/s 转成 C620 反馈侧 rpm。
 * 轮径 0.14m；0.58m/0.50m 是前后轮距和左右轮距，控制里使用半长/半宽。 */
#define CHASSIS_WHEEL_DIAMETER_M      0.14f
#define CHASSIS_WHEEL_RADIUS_M        (CHASSIS_WHEEL_DIAMETER_M * 0.5f)
#define CHASSIS_LENGTH_M              0.58f
#define CHASSIS_WIDTH_M               0.50f
#define CHASSIS_HALF_LENGTH_M         (CHASSIS_LENGTH_M * 0.5f)
#define CHASSIS_HALF_WIDTH_M          (CHASSIS_WIDTH_M * 0.5f)
#define CHASSIS_MOTOR_REDUCTION_RATIO 19.0f
#define CHASSIS_PI                    3.1415926535f

/* 调试限速系数:
 * 上位机仍然按 m/s、rad/s 发送，底盘内部先乘该系数再解算轮速。
 * 只影响 Chassis_SetVelocity()，不影响直接设置 rpm 的 Chassis_SetVelocityRpm()。 */
#define CHASSIS_VX_SCALE    1.4f
#define CHASSIS_VY_SCALE    1.84f
#define CHASSIS_VW_SCALE    1.07f

/* 上电跑车测试:
 * 置 1 后，等四个底盘电机反馈都在线，再按下面的物理速度跑 1 秒后自动停止。这里的CHASSIS_BOOT_TEST_xyw是上电测试的时候测的东西，
 * BOOT_TEST 只用来选方向，值保持 0 或 1，实际速度由 CHASSIS_BOOT_TEST_VX_MPS/CHASSIS_BOOT_TEST_VY_MPS/CHASSIS_BOOT_TEST_WZ_RADPS 决定。
 * 这里的速度也会经过 CHASSIS_LINEAR/ANGULAR_VELOCITY_SCALE 限速。 */
#define CHASSIS_BOOT_TEST_ENABLE      0
#define CHASSIS_BOOT_TEST_DURATION_MS 20000000U
#define CHASSIS_BOOT_TEST_VX_MPS      1.0f
#define CHASSIS_BOOT_TEST_VY_MPS      1.0f
#define CHASSIS_BOOT_TEST_WZ_RADPS    1.0f

typedef enum {
    CHASSIS_WHEEL_RF = 0,  /* 右前 */
    CHASSIS_WHEEL_LF,      /* 左前 */
    CHASSIS_WHEEL_LB,      /* 左后 */
    CHASSIS_WHEEL_RB,      /* 右后 */
} ChassisWheelIndex;

typedef struct {
    uint8_t motor_id;      /*CHASSIS_BOOT_TEST_VY_MPS C620 电调 ID: 1~8，对应反馈 ID 0x201~0x208 */
    int8_t direction;      /* 方向系数: 1 或 -1 */
    float speed_kp;
    float speed_ki;
    float speed_kd;
} ChassisMotorConfig;

/* 四个轮子的实车配置，顺序必须和 ChassisWheelIndex 保持一致。 */
#define CHASSIS_MOTOR_CONFIG_INIT                       \
    {                                                   \
        {1U,  1, 10.0f, 80.0f, 0.03f}, /* RF 右前 */      \
        {2U, -1, 10.0f, 80.0f, 0.03f}, /* LF 左前 */      \
        {3U, -1, 10.0f, 80.0f, 0.03f}, /* LB 左后 */      \
        {4U,  1, 10.0f, 80.0f, 0.03f}, /* RB 右后 */      \
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

void Chassis_SetVelocity(float vx_mps, float vy_mps, float wz_radps);
void Chassis_SetVelocityRpm(float vx_rpm, float vy_rpm, float wz_rpm);
float Chassis_GetWheelTargetRpm(ChassisWheelIndex wheel);

extern volatile float g_chassis_target_rpm[CHASSIS_MOTOR_COUNT];
extern volatile float g_chassis_cmd_vx;
extern volatile float g_chassis_cmd_vy;
extern volatile float g_chassis_cmd_vw;
extern volatile float g_chassis_cmd_vx_mps;
extern volatile float g_chassis_cmd_vy_mps;
extern volatile float g_chassis_cmd_wz_radps;
extern volatile uint32_t g_chassis_set_velocity_count;

#ifdef __cplusplus
}
#endif

#endif
