#ifndef RS_MOTOR_H
#define RS_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "fdcan.h"

/* ==========================================================================
 * Lingzu RobStride motor driver - private CAN protocol
 *
 * Boundary:
 *   - CubeMX/Core owns FDCAN hardware init.
 *   - Bsp/bsp_fdcan owns actual CAN transmission through fdcanx_send_ext_data().
 *   - This file owns RobStride private protocol packing/parsing, object state,
 *     CSP position command helpers, and multi-turn feedback expansion.
 * ========================================================================== */

#define RS_MOTOR_COUNT                8U
#define RS_MOTOR_DEFAULT_HOST_ID      0xFDU
#define RS_MOTOR_DEFAULT_OFFLINE_MS   100U

#define RS_MOTOR_RAD_PER_DEG          0.017453292519943295f
#define RS_MOTOR_DEG_PER_RAD          57.29577951308232f

typedef enum {
    RS_MOTOR_MODEL_RS00 = 0,
    RS_MOTOR_MODEL_RS03,
    RS_MOTOR_MODEL_RS05,
    RS_MOTOR_MODEL_CUSTOM,
} RsMotorModel;

typedef enum {
    RS_MOTOR_PROTOCOL_PRIVATE = 0,
    RS_MOTOR_PROTOCOL_CANOPEN = 1,
    RS_MOTOR_PROTOCOL_MIT     = 2,
} RsMotorProtocol;

typedef enum {
    RS_MOTOR_CONTROL_MODE_MOTION       = 0,
    RS_MOTOR_CONTROL_MODE_PP_POSITION  = 1,
    RS_MOTOR_CONTROL_MODE_SPEED        = 2,
    RS_MOTOR_CONTROL_MODE_CURRENT      = 3,
    RS_MOTOR_CONTROL_MODE_ZERO         = 4,
    RS_MOTOR_CONTROL_MODE_CSP_POSITION = 5,
    RS_MOTOR_CONTROL_MODE_NONE         = 0xFF
} RsMotorControlMode;

/* Backward-compatible name used by older standard-frame helpers. */
typedef RsMotorControlMode RsMotorRunMode;

typedef enum {
    RS_MOTOR_RUN_STATE_REST = 0,
    RS_MOTOR_RUN_STATE_CALIBRATING = 1,
    RS_MOTOR_RUN_STATE_RUNNING = 2,
} RsMotorRunState;

typedef enum {
    RS_MOTOR_STATUS_OK = 0,
    RS_MOTOR_STATUS_INVALID_ARGUMENT,
    RS_MOTOR_STATUS_INVALID_CONFIG,
    RS_MOTOR_STATUS_NOT_INITIALIZED,
    RS_MOTOR_STATUS_DUPLICATE_INSTANCE,
    RS_MOTOR_STATUS_TX_ERROR,
} RsMotorStatus;

typedef struct {
    RsMotorModel model;

    /* Protocol feedback conversion range. This is not a mechanical limit. */
    float position_min_rad;
    float position_max_rad;
    float speed_min_rad_s;
    float speed_max_rad_s;
    float torque_min_nm;
    float torque_max_nm;
} RsMotorModelParams;

typedef struct {
    uint8_t uncalibrated;
    uint8_t stall_overload;
    uint8_t magnetic_encoder;
    uint8_t over_temperature;
    uint8_t drive;
    uint8_t under_voltage;
} RsMotorFault;

typedef struct {
    float raw_position_rad;      /* Single-span protocol position. */
    float total_position_rad;    /* Multi-turn expanded position before zero offset. */
    float position_rad;          /* total_position_rad - zero_offset_rad. */
    float angle_deg;
    float speed_rad_s;
    float speed_deg_s;
    float torque_nm;
    float temperature_c;
    RsMotorRunState run_state;
} RsMotorFeedback;

typedef struct {
    uint8_t online;
    uint8_t enabled;
    RsMotorControlMode control_mode;
    uint32_t feedback_count;
    uint32_t last_update_ms;
} RsMotorPublicState;

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t motor_id;              /* RobStride CAN node ID, valid 1..0x7f. */
    uint8_t host_id;               /* Controller/master ID used in private extended IDs. */
    RsMotorModel model;
    RsMotorProtocol protocol;
    RsMotorModelParams params;     /* Feedback conversion range for this instance. */
    uint32_t offline_timeout_ms;
} RsMotorConfig;

typedef struct rs_motor RsMotor;

typedef struct {
    uint8_t initialized;
    uint8_t mode_applied;
    RsMotorControlMode applied_control_mode;

    float last_raw_position_rad;
    int32_t wrap_count;
    float zero_offset_rad;

    float target_position_rad;
    float target_speed_rad_s;

    RsMotor *next;
} RsMotorInternal;

struct rs_motor {
    RsMotorConfig config;
    RsMotorInternal internal;
    RsMotorFeedback feedback;
    RsMotorPublicState state;
    RsMotorFault fault;
};

/* Legacy flattened state kept for existing modules and debugger watch windows. */
typedef struct {
    uint8_t id;
    uint8_t online;
    RsMotorModel model;

    float raw_position_rad;
    float last_raw_position_rad;
    int32_t wrap_count;
    float total_position_rad;
    float total_angle_deg;

    float zero_offset_rad;
    float position_rad;
    float angle_deg;

    float speed_rad_s;
    float speed_deg_s;
    float torque_nm;
    float temperature_c;

    uint8_t mode_state;
    uint8_t fault;
    uint8_t warning;

    float target_position_rad;
    float target_speed_rad_s;

    uint32_t update_count;
    uint32_t last_update_ms;
} RsMotorState;

typedef RsMotor rs_motor_t;
typedef RsMotorConfig rs_motor_config_t;
typedef RsMotorStatus rs_motor_status_t;

extern RsMotorState g_rs_motors[RS_MOTOR_COUNT];

void RsMotor_Init(void);

RsMotorStatus rs_motor_init(RsMotor *motor);
RsMotorStatus rs_motor_deinit(RsMotor *motor);
RsMotorStatus rs_motor_enable(RsMotor *motor);
RsMotorStatus rs_motor_disable(RsMotor *motor);
RsMotorStatus rs_motor_csp_position_control(RsMotor *motor,
                                            float speed_limit_rad_s,
                                            float position_rad);
RsMotorStatus rs_motor_update(RsMotor *motor, uint32_t now_ms);
uint8_t rs_motor_handle_rx(FDCAN_HandleTypeDef *hfdcan,
                           const FDCAN_RxHeaderTypeDef *header,
                           const uint8_t data[8],
                           uint32_t now_ms);

RsMotor *RsMotor_GetObject(uint8_t motor_id);
uint8_t RsMotor_ConfigureMotor(uint8_t motor_id, RsMotorModel model);
uint8_t RsMotor_SetCustomModelParams(uint8_t motor_id, const RsMotorModelParams *params);
RsMotorState *RsMotor_GetState(uint8_t motor_id);

uint8_t RsMotor_IsFeedbackId(uint16_t std_id);
uint8_t RsMotor_IsPrivateFeedbackId(uint32_t ext_id);
uint8_t RsMotor_SetHostId(uint8_t host_id);
uint8_t RsMotor_GetHostId(void);

uint8_t RsMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms);
uint8_t RsMotor_HandlePrivateFeedback(uint32_t ext_id, const uint8_t data[8], uint32_t now_ms);

void RsMotor_SetZeroToCurrent(uint8_t motor_id);
uint8_t RsMotor_SetTargetPositionDeg(uint8_t motor_id,
                                     float position_deg,
                                     float max_speed_deg_s);
uint8_t RsMotor_CspPositionControl(uint8_t motor_id,
                                   float position_rad,
                                   float speed_limit_rad_s);

/* Private-protocol frame builders. They only pack frames; transmission is done
 * by bsp_fdcan through fdcanx_send_ext_data(). */
uint8_t RsMotor_BuildPrivateEnableFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivateStopFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivateRunModeFrame(uint8_t motor_id, uint8_t run_mode,
                                         uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivateParamWriteFrame(uint8_t motor_id, uint16_t index, float value,
                                            uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivatePositionFrame(uint8_t motor_id, float position_rad,
                                          float limit_spd_rad_s,
                                          uint32_t *ext_id, uint8_t data[8]);

/* Legacy standard-frame helpers kept for compatibility. New RobStride code
 * should use the private-protocol CSP helpers above. */
uint8_t RsMotor_BuildEnableFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildStopFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildClearFaultFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildSetZeroFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildSetRunModeFrame(uint8_t motor_id, RsMotorRunMode mode,
                                     uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildSetProtocolFrame(uint8_t motor_id, RsMotorProtocol protocol,
                                      uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildPositionFrame(uint8_t motor_id, float position_deg,
                                   float max_speed_deg_s,
                                   uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildPositionFrameFromTarget(uint8_t motor_id,
                                             uint16_t *std_id, uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif /* RS_MOTOR_H */
