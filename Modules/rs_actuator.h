#ifndef RS_ACTUATOR_H
#define RS_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* RS 执行器上电小角度测试。
 * 默认关闭；确认机械安全、方向和限位后，再将 ENABLE 改为 1。
 * 测试目标是相对上电后首次设定的零点，而不是协议原始坐标。 */
#define RS_ACTUATOR_BOOT_TEST_ENABLE       1U
#define RS_ACTUATOR_BOOT_TEST_DELAY_MS     1000U
#define RS_ACTUATOR_BOOT_TEST_TARGET_DEG   40.0f
#define RS_ACTUATOR_BOOT_TEST_SPEED_DEG_S  20.0f
#define RS_ACTUATOR_BOOT_TEST_DURATION_MS   3000U
#define RS_ACTUATOR_CSP_STATUS_NOT_CALLED   0xFFU

typedef enum {
    RS_ACTUATOR_BOOT_TEST_PHASE_DISABLED = 0,
    RS_ACTUATOR_BOOT_TEST_PHASE_WAIT_FEEDBACK,
    RS_ACTUATOR_BOOT_TEST_PHASE_WAIT_STABLE,
    RS_ACTUATOR_BOOT_TEST_PHASE_RUNNING,
    RS_ACTUATOR_BOOT_TEST_PHASE_DONE,
} RsActuatorBootTestPhase;

typedef enum {
    RS_ACTUATOR_0 = 0,
    RS_ACTUATOR_COUNT,
} RsActuatorId;

void RsActuator_Init(void);
void RsActuator_RunPeriodic(void);

uint8_t RsActuator_SetTargetDeg(RsActuatorId actuator, float position_deg);
void RsActuator_SetZeroToCurrent(RsActuatorId actuator);
void RsActuator_Disable(RsActuatorId actuator);

float RsActuator_GetTargetDeg(RsActuatorId actuator);
float RsActuator_GetPositionDeg(RsActuatorId actuator);
uint8_t RsActuator_IsOnline(RsActuatorId actuator, uint32_t now_ms);

/* Ozone/debugger 观察量，不参与控制逻辑。 */
extern volatile RsActuatorBootTestPhase g_rs_actuator_boot_test_phase;
extern volatile uint32_t g_rs_actuator_csp_success_count;
extern volatile uint32_t g_rs_actuator_csp_error_count;
extern volatile uint8_t g_rs_actuator_last_csp_status;
extern volatile float g_rs_actuator_last_csp_target_rad;
extern volatile float g_rs_actuator_last_csp_speed_rad_s;

#ifdef __cplusplus
}
#endif

#endif /* RS_ACTUATOR_H */
