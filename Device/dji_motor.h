#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DJI_MOTOR_FEEDBACK_ID_BASE 0x201U
#define DJI_MOTOR_FEEDBACK_ID_MIN  0x201U
#define DJI_MOTOR_FEEDBACK_ID_MAX  0x208U
#define DJI_MOTOR_ENCODER_RANGE    8192
#define DJI_MOTOR_ENCODER_HALF     4096
#define DJI_MOTOR_CURRENT_MAX      16384.0f

typedef enum {
    DJI_MOTOR_CONTROL_MODE_CURRENT = 0,
    DJI_MOTOR_CONTROL_MODE_SPEED,
    DJI_MOTOR_CONTROL_MODE_POSITION,
    DJI_MOTOR_CONTROL_MODE_NONE = 0xFF
} dji_motor_control_mode_t;

typedef enum {
    DJI_MOTOR_STATUS_OK = 0,
    DJI_MOTOR_STATUS_INVALID_ARGUMENT,
    DJI_MOTOR_STATUS_INVALID_CONFIG,
    DJI_MOTOR_STATUS_NOT_INITIALIZED,
    DJI_MOTOR_STATUS_DUPLICATE_INSTANCE,
    DJI_MOTOR_STATUS_ALREADY_GROUPED,
    DJI_MOTOR_STATUS_GROUP_MISMATCH,
    DJI_MOTOR_STATUS_FDCAN_TX_ERROR
} dji_motor_status_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
} dji_motor_pid_config_t;

typedef struct {
    uint8_t motor_id;             /**< C620 电调 ID，范围 1..8。 */
    uint32_t offline_timeout_ms;  /**< 反馈超时，必须大于控制周期。 */
    uint32_t control_period_ms;   /**< PID 运行周期，必须大于 0。 */
    float current_limit;          /**< C620 电流命令限幅，范围 (0, 16384]。 */
    float max_speed_rpm;          /**< 位置环输出转速限幅；不用位置模式时可为 0。 */
    dji_motor_pid_config_t speed_pid;
    dji_motor_pid_config_t position_pid;
} dji_motor_config_t;

typedef struct {
    int64_t encoder_count; /**< 以首帧为零点的连续累计编码器计数。 */
    double angle_deg;      /**< 以首帧为零点的连续累计角度，单位 deg。 */
    uint8_t valid;
} dji_motor_multi_turn_feedback_t;

typedef struct {
    uint16_t encoder;       /**< C620 单圈机械角度，范围 0..8191。 */
    int16_t speed_rpm;      /**< 电机轴转速，单位 rpm。 */
    int16_t given_current;  /**< C620 回传的转矩电流原始值。 */
    uint8_t temperature_c;  /**< 电调温度，单位摄氏度。 */
    dji_motor_multi_turn_feedback_t multi_turn;
} dji_motor_feedback_t;

typedef struct {
    volatile uint8_t online;
    uint8_t enabled;
    dji_motor_control_mode_t control_mode;
    volatile uint32_t feedback_count;
    volatile uint32_t last_update_ms;
    float target_current;
    float target_speed_rpm;
    double target_position_deg;
    double position_error_deg;
    float speed_correction_rpm; /**< 位置模式的速度输出修正，单位 rpm。 */
    float current_correction;   /**< 速度/位置模式的 C620 电流命令修正值。 */
} dji_motor_state_t;

typedef struct dji_motor_group dji_motor_group_t;

typedef struct {
    double integral;
    double last_error;
} dji_motor_pid_state_t;

typedef struct {
    uint16_t previous_encoder;
    int16_t current_command;
    uint32_t last_control_ms;
    uint8_t initialized;
    uint8_t control_timer_started;
    dji_motor_pid_state_t speed_pid;
    dji_motor_pid_state_t position_pid;
    dji_motor_group_t *group;
} dji_motor_internal_t;

typedef struct {
    dji_motor_config_t config;
    dji_motor_feedback_t feedback;
    dji_motor_state_t state;
    dji_motor_internal_t internal;
} dji_motor_t;

dji_motor_status_t dji_motor_init(dji_motor_t *motor);
dji_motor_status_t dji_motor_deinit(dji_motor_t *motor);

dji_motor_status_t dji_motor_disable(dji_motor_t *motor);
/**
 * 设置位置环输出的运行时速度修正。可在每个控制周期调用，不复位 PID。
 */
dji_motor_status_t dji_motor_set_speed_correction(
    dji_motor_t *motor,
    float speed_correction_rpm);
/**
 * 设置速度环输出的运行时电流修正。可在每个控制周期调用，不复位 PID。
 */
dji_motor_status_t dji_motor_set_current_correction(
    dji_motor_t *motor,
    float current_correction);
dji_motor_status_t dji_motor_current_control(dji_motor_t *motor,
                                             float current);
dji_motor_status_t dji_motor_speed_control(dji_motor_t *motor,
                                           float speed_rpm);
/**
 * 连续多圈位置控制。position_deg 以首帧反馈建立的软件零点为基准，
 * 可为任意有限 double 角度，不区分单圈和多圈。
 */
dji_motor_status_t dji_motor_position_control(dji_motor_t *motor,
                                              double position_deg);

/** 将当前编码器位置重新定义为连续位置 0 deg，并复位控制器。 */
dji_motor_status_t dji_motor_set_zero_to_current(dji_motor_t *motor);

/** 在短临界区内复制完整反馈，安全读取中断中更新的 double 连续角度。 */
dji_motor_status_t dji_motor_get_feedback(const dji_motor_t *motor,
                                          dji_motor_feedback_t *feedback);

/** 由 Group 在主循环调用，更新在线状态和当前控制模式的 PID。 */
dji_motor_status_t dji_motor_update(dji_motor_t *motor, uint32_t now_ms);

/** 由 Group 的 RX 分发器在中断中调用。 */
dji_motor_status_t dji_motor_handle_feedback(dji_motor_t *motor,
                                             const uint8_t data[8],
                                             uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* DJI_MOTOR_H */
