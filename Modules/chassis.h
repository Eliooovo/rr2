#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CHASSIS_MOTOR_COUNT        4U
#define CHASSIS_CONTROL_PERIOD_MS  1U

/* 底盘电机 ID: FDCAN1 上的 DJI 3508 / C620，ID 1,2,3,4。 */
#define CHASSIS_MOTOR_ID_1         1U
#define CHASSIS_MOTOR_ID_2         2U
#define CHASSIS_MOTOR_ID_3         3U
#define CHASSIS_MOTOR_ID_4         4U

/* 根据实车安装方向调整。试车时某个轮子方向反了，把对应值改为 -1。 */
#define CHASSIS_MOTOR_DIR_1        1
#define CHASSIS_MOTOR_DIR_2        1
#define CHASSIS_MOTOR_DIR_3        1
#define CHASSIS_MOTOR_DIR_4        1

/* 上电默认不自动运动。需要四轮低速试转时改为 1。 */
#define CHASSIS_BOOT_TEST_ENABLE   0
#define CHASSIS_BOOT_TEST_RPM      300.0f

void Chassis_Init(void);
void Chassis_Stop(void);
void Chassis_RunPeriodic(void);
/*
一次 PID 控制迭代：对 4 个轮子逐一读实际转速 → 和目标转速做 PID → 输出电流 → DjiMotor_SetCurrent() → 打包发送 CAN 帧到 FDCAN1  
*/
void Chassis_ControlLoop(float dt_s);

void Chassis_SetWheelTargetRpm(float motor1_rpm,
                               float motor2_rpm,
                               float motor3_rpm,
                               float motor4_rpm);

void Chassis_SetVelocityRpm(float vx_rpm, float vy_rpm, float wz_rpm);
float Chassis_GetWheelTargetRpm(uint8_t wheel_index);

#ifdef __cplusplus
}
#endif

#endif
