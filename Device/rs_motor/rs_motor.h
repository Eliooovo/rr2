#ifndef RS_MOTOR_H
#define RS_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "fdcan.h"

/** RobStride 参数地址，用于 rs_motor_write_parameter() 的 index 参数。 */
#define RS_PARAM_CUR_KP              0x7010U  /**< 电流环 Kp          */
#define RS_PARAM_CUR_KI              0x7011U  /**< 电流环 Ki          */
#define RS_PARAM_CUR_FILT_GAIN       0x7014U  /**< 电流滤波系数        */
#define RS_PARAM_CURRENT_LIMIT       0x7018U  /**< 速度/位置模式电流限制 */
#define RS_PARAM_LOC_KP              0x701EU  /**< 位置环 Kp (CSP核心) */
#define RS_PARAM_SPD_KP              0x701FU  /**< 速度环 Kp          */
#define RS_PARAM_SPD_KI              0x7020U  /**< 速度环 Ki          */
#define RS_PARAM_SPD_FILT_GAIN       0x7021U  /**< 速度滤波系数        */
#define RS_PARAM_DAMPER              0x702AU  /**< 阻尼开关 (uint8)   */

/** RobStride 私有协议支持的电机参数表编号。 */
typedef enum {
    RS_MOTOR_TYPE_0 = 0,
    RS_MOTOR_TYPE_1,
    RS_MOTOR_TYPE_2,
    RS_MOTOR_TYPE_3,
    RS_MOTOR_TYPE_4,
    RS_MOTOR_TYPE_5,
    RS_MOTOR_TYPE_6,
    RS_MOTOR_TYPE_COUNT
} rs_motor_type_t;

/** 电机反馈中的运行状态。 */
typedef enum {
    RS_MOTOR_RUN_STATE_REST = 0,
    RS_MOTOR_RUN_STATE_CALIBRATING = 1,
    RS_MOTOR_RUN_STATE_RUNNING = 2
} rs_motor_run_state_t;

/** 电机原生控制模式及 MCU 侧逻辑控制模式。 */
typedef enum {
    RS_MOTOR_CONTROL_MODE_MOTION = 0,
    RS_MOTOR_CONTROL_MODE_PP_POSITION = 1,
    RS_MOTOR_CONTROL_MODE_SPEED = 2,
    RS_MOTOR_CONTROL_MODE_CURRENT = 3,
    RS_MOTOR_CONTROL_MODE_ZERO = 4,
    RS_MOTOR_CONTROL_MODE_CSP_POSITION = 5,
    /**
     * MCU 侧连续多圈位置外环；电机的 0x7005 参数实际配置为速度模式。
     */
    RS_MOTOR_CONTROL_MODE_MULTI_TURN_POSITION = 6,
    RS_MOTOR_CONTROL_MODE_NONE = 0xFF
} rs_motor_control_mode_t;

typedef enum {
    RS_MOTOR_STATUS_OK = 0,
    RS_MOTOR_STATUS_INVALID_ARGUMENT,
    RS_MOTOR_STATUS_INVALID_CONFIG,
    RS_MOTOR_STATUS_NOT_INITIALIZED,
    RS_MOTOR_STATUS_DUPLICATE_INSTANCE,
    RS_MOTOR_STATUS_FDCAN_TX_ERROR
} rs_motor_status_t;

/* 简短兼容名，便于在应用代码中判断返回值。 */
#define RS_MOTOR_OK                    RS_MOTOR_STATUS_OK
#define RS_MOTOR_ERROR_PARAM           RS_MOTOR_STATUS_INVALID_ARGUMENT
#define RS_MOTOR_ERROR_CONFIG          RS_MOTOR_STATUS_INVALID_CONFIG
#define RS_MOTOR_ERROR_NOT_INITIALIZED RS_MOTOR_STATUS_NOT_INITIALIZED
#define RS_MOTOR_ERROR_DUPLICATE       RS_MOTOR_STATUS_DUPLICATE_INSTANCE
#define RS_MOTOR_ERROR_FDCAN_TX        RS_MOTOR_STATUS_FDCAN_TX_ERROR

typedef struct {
    uint8_t uncalibrated;
    uint8_t stall_overload;
    uint8_t magnetic_encoder;
    uint8_t over_temperature;
    uint8_t drive;
    uint8_t under_voltage;
} rs_motor_fault_t;

typedef struct {
    double angle_rad; /**< 以上电后首帧反馈位置为零点的连续累计角度，单位 rad。 */
    uint8_t valid;    /**< 已收到首帧反馈并建立软件零点。 */
} rs_motor_multi_turn_feedback_t;

typedef struct {
    float angle_rad;
    float speed_rad_s;
    float torque_nm;
    float temperature_c;

    rs_motor_run_state_t run_state;
    rs_motor_fault_t fault;
    rs_motor_multi_turn_feedback_t multi_turn;
} rs_motor_feedback_t;

typedef struct {
    uint8_t active;
    uint8_t paused_offline;
    uint8_t position_reached;
    double target_position_rad;
    double position_error_rad;
} rs_motor_multi_turn_state_t;

typedef struct {
    volatile uint8_t online;
    uint8_t enabled;
    rs_motor_control_mode_t control_mode;

    volatile uint32_t feedback_count;
    volatile uint32_t last_update_ms;
    rs_motor_multi_turn_state_t multi_turn;
} rs_motor_state_t;

typedef struct {
    float current_limit_a;
    float position_kp_s_1;
    /**
     * 到位容差，单位 rad；设为 0 时关闭到位检测并持续运行 P 外环。
     */
    float position_tolerance_rad;
    uint32_t control_period_ms;
} rs_motor_multi_turn_config_t;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t motor_id;              /**< 有效范围 1..0x7f。 */
    uint8_t master_id;
    rs_motor_type_t motor_type;
    uint32_t offline_timeout_ms;   /**< 必须大于 0。 */
    /**
     * 连续多圈位置外环配置。全零表示不启用，调用多圈接口时才校验。
     */
    rs_motor_multi_turn_config_t multi_turn;
} rs_motor_config_t;

typedef struct rs_motor rs_motor_t;

/**
 * 驱动私有数据。应用只应读取 config、feedback 和 state，不应修改此结构。
 */
typedef struct {
    float position_min;
    float position_max;
    float speed_min;
    float speed_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float torque_min;
    float torque_max;

    float previous_feedback_angle_rad;
    float multi_turn_speed_limit_rad_s;
    float csp_current_limit_a;
    float csp_speed_limit_rad_s;
    float pp_current_limit_a;
    float pp_speed_rad_s;
    float pp_acceleration_rad_s2;
    uint32_t multi_turn_last_control_ms;

    volatile uint32_t tx_error_count;
    volatile uint32_t last_hal_error;
    volatile uint32_t offline_transition_count;
    volatile uint32_t online_recovery_count;
    volatile uint32_t max_feedback_gap_ms;

    uint8_t initialized;
    uint8_t mode_applied;
    uint8_t csp_limited_parameters_applied;
    uint8_t csp_speed_limit_applied;
    uint8_t pp_parameters_applied;
    uint8_t pp_current_limit_applied;
    uint8_t multi_turn_control_timer_started;
    rs_motor_control_mode_t applied_control_mode;
    rs_motor_t *next;
} rs_motor_internal_t;

struct rs_motor {
    rs_motor_config_t config;
    rs_motor_feedback_t feedback;
    rs_motor_state_t state;
    rs_motor_internal_t internal;
};

rs_motor_status_t rs_motor_init(rs_motor_t *motor);
rs_motor_status_t rs_motor_deinit(rs_motor_t *motor);

rs_motor_status_t rs_motor_enable(rs_motor_t *motor);
rs_motor_status_t rs_motor_disable(rs_motor_t *motor);

/** 力矩、位置、速度、Kp、Kd 顺序与 RobStride 私有协议一致。 */
rs_motor_status_t rs_motor_motion_control(rs_motor_t *motor,
                                          float torque_nm,
                                          float position_rad,
                                          float speed_rad_s,
                                          float kp,
                                          float kd);

/** PP 模式：目标速度、加速度、目标位置。 */
rs_motor_status_t rs_motor_pp_position_control(rs_motor_t *motor,
                                               float speed_rad_s,
                                               float acceleration_rad_s2,
                                               float position_rad);

/** PP 模式：电流限制、目标速度、加速度、目标位置。 */
rs_motor_status_t rs_motor_pp_position_control_limited(
    rs_motor_t *motor,
    float current_limit_a,
    float speed_rad_s,
    float acceleration_rad_s2,
    float position_rad);

/** CSP 模式：速度限制、目标位置。 */
rs_motor_status_t rs_motor_csp_position_control(rs_motor_t *motor,
                                                float speed_limit_rad_s,
                                                float position_rad);

/** CSP 模式：电流限制、速度限制、目标位置。 */
rs_motor_status_t rs_motor_csp_position_control_limited(
    rs_motor_t *motor,
    float current_limit_a,
    float speed_limit_rad_s,
    float position_rad);

/** 速度模式：电流限制、加速度、目标速度。 */
rs_motor_status_t rs_motor_speed_control(rs_motor_t *motor,
                                         float current_limit_a,
                                         float acceleration_rad_s2,
                                         float speed_rad_s);

rs_motor_status_t rs_motor_current_control(rs_motor_t *motor, float current_a);

/** 向电机写入一个 float 参数（通信类型 0x12），index 使用 RS_PARAM_* 宏。 */
rs_motor_status_t rs_motor_write_parameter(rs_motor_t *motor,
                                           uint16_t index,
                                           float value);

/**
 * MCU 侧连续多圈位置控制：速度限制、加速度、连续位置目标。
 *
 * position_rad 使用首帧反馈建立的软件零点，可为任意有限 double 弧度值。
 */
rs_motor_status_t rs_motor_multi_turn_position_control(
    rs_motor_t *motor,
    float speed_limit_rad_s,
    float acceleration_rad_s2,
    double position_rad);

/**
 * 在短临界区内复制完整反馈，供主循环安全读取中断中更新的 double 多圈角度。
 */
rs_motor_status_t rs_motor_get_feedback(const rs_motor_t *motor,
                                        rs_motor_feedback_t *feedback);

/**
 * 处理一帧 FDCAN RX FIFO 消息。
 *
 * @return 1 表示帧已分发给一个实例，0 表示帧非法或没有匹配实例。
 */
uint8_t rs_motor_handle_rx(FDCAN_HandleTypeDef *hfdcan,
                           const FDCAN_RxHeaderTypeDef *header,
                           const uint8_t data[8],
                           uint32_t now_ms);

/**
 * 使用无符号时间差处理 HAL tick 回绕、更新在线状态并运行多圈位置外环。
 * 多圈模式下，调用方应以不慢于 config.multi_turn.control_period_ms 的周期调用。
 */
rs_motor_status_t rs_motor_update(rs_motor_t *motor, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* RS_MOTOR_H */
