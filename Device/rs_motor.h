#ifndef RS_MOTOR_H
#define RS_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==========================================================================
 * 灵足 RobStride 电机驱动 - MIT 标准帧协议层
 *
 * 适用目标:
 *   - RS00 / RS05 已按手册填入反馈量程
 *   - RS03 预留同协议入口，具体速度/力矩量程需要拿到 RS03 手册后确认
 *
 * 设计边界:
 *   - 本文件只做 CAN 协议打包、反馈解析、状态保存和多圈展开
 *   - 不直接调用 HAL_FDCAN 发送，发送仍由 bsp_fdcan 的 fdcanx_send_data() 完成
 *   - 位置环 PID 在电机内部实现，上层只需要给位置目标和过程速度
 *
 * 使用流程示例:
 *   RsMotor_Init();
 *   RsMotor_ConfigureMotor(1, RS_MOTOR_MODEL_RS00);
 *
 *   // 上电后建议先切到位置模式，再使能
 *   RsMotor_BuildSetRunModeFrame(1, RS_MOTOR_RUN_MODE_POSITION, &tx_id, data);
 *   fdcanx_send_data(&hfdcan3, tx_id, data, 8);
 *   RsMotor_BuildEnableFrame(1, &tx_id, data);
 *   fdcanx_send_data(&hfdcan3, tx_id, data, 8);
 *
 *   // 发送 361 度目标。目标会以 float rad 发送，361 度不会被截断成 1 度。
 *   RsMotor_SetTargetPositionDeg(1, 361.0f, 90.0f);
 *   RsMotor_BuildPositionFrameFromTarget(1, &tx_id, data);
 *   fdcanx_send_data(&hfdcan3, tx_id, data, 8);
 * ========================================================================== */

#define RS_MOTOR_COUNT                8U
#define RS_MOTOR_DEFAULT_HOST_ID      0x0FDU

/* MIT 位置/速度控制帧的 11 位 CAN ID: bit10~8 是模式类型, bit7~0 是目标电机 ID。 */
#define RS_MOTOR_CAN_MODE_POSITION    0x1U
#define RS_MOTOR_CAN_MODE_SPEED       0x2U

#define RS_MOTOR_RAD_PER_DEG          0.017453292519943295f
#define RS_MOTOR_DEG_PER_RAD          57.29577951308232f

typedef enum {
    RS_MOTOR_MODEL_RS00 = 0,
    RS_MOTOR_MODEL_RS03,
    RS_MOTOR_MODEL_RS05,
    RS_MOTOR_MODEL_CUSTOM,
} RsMotorModel;

typedef enum {
    RS_MOTOR_RUN_MODE_MIT      = 0,
    RS_MOTOR_RUN_MODE_POSITION = 1,
    RS_MOTOR_RUN_MODE_SPEED    = 2,
} RsMotorRunMode;

typedef enum {
    RS_MOTOR_PROTOCOL_PRIVATE = 0,
    RS_MOTOR_PROTOCOL_CANOPEN = 1,
    RS_MOTOR_PROTOCOL_MIT     = 2,
} RsMotorProtocol;

typedef struct {
    RsMotorModel model;

    /* MIT 反馈量程。手册里的编码值会按这些范围线性还原为物理量。 */
    float position_min_rad;
    float position_max_rad;
    float speed_min_rad_s;
    float speed_max_rad_s;
    float torque_min_nm;
    float torque_max_nm;

    /* RS00 的 MIT 反馈 Byte6 带模式/故障/预警标志，温度只占 12bit。
     * RS05 手册中 Byte6~7 直接是 Temp*10，所以这里用配置区分。 */
    uint8_t feedback_has_status_bits;
} RsMotorModelParams;

typedef struct {
    uint8_t id;                 /* 电机 CAN ID，通常为 1~8 */
    uint8_t online;             /* 是否收到过反馈 */
    RsMotorModel model;

    float raw_position_rad;     /* 单段反馈位置，范围由型号参数决定，如 -12.57~12.57rad */
    float last_raw_position_rad;
    int32_t wrap_count;         /* 多圈展开计数，跨越反馈范围边界时 +/-1 */
    float total_position_rad;   /* 多圈连续位置，支持 361deg 这类超过单圈的反馈 */
    float total_angle_deg;

    float zero_offset_rad;      /* 软件零点，SetZeroToCurrent 后有效 */
    float position_rad;         /* total_position_rad - zero_offset_rad */
    float angle_deg;            /* position_rad 转成角度 */

    float speed_rad_s;
    float speed_deg_s;
    float torque_nm;
    float temperature_c;

    uint8_t mode_state;         /* 0=Reset, 1=Cali, 2=Motor；部分型号 MIT 反馈不提供 */
    uint8_t fault;              /* 1=有故障；部分型号 MIT 反馈不提供 */
    uint8_t warning;            /* 1=有预警；部分型号 MIT 反馈不提供 */

    float target_position_rad;  /* 最近一次缓存的位置目标 */
    float target_speed_rad_s;   /* 最近一次缓存的位置模式过程速度 */

    uint32_t update_count;
    uint32_t last_update_ms;
} RsMotorState;

extern RsMotorState g_rs_motors[RS_MOTOR_COUNT];

void RsMotor_Init(void);

uint8_t RsMotor_ConfigureMotor(uint8_t motor_id, RsMotorModel model);
uint8_t RsMotor_SetCustomModelParams(uint8_t motor_id, const RsMotorModelParams *params);
RsMotorState *RsMotor_GetState(uint8_t motor_id);

uint8_t RsMotor_IsFeedbackId(uint16_t std_id);
uint8_t RsMotor_IsPrivateFeedbackId(uint32_t ext_id);
uint8_t RsMotor_SetHostId(uint8_t host_id);
uint8_t RsMotor_GetHostId(void);

/* 在 FDCAN3 接收回调里调用。返回解析到的 motor_id，非灵足反馈返回 0。 */
uint8_t RsMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms);
uint8_t RsMotor_HandlePrivateFeedback(uint32_t ext_id, const uint8_t data[8], uint32_t now_ms);

void RsMotor_SetZeroToCurrent(uint8_t motor_id);
uint8_t RsMotor_SetTargetPositionDeg(uint8_t motor_id, float position_deg, float max_speed_deg_s);

/* 常用控制帧打包。成功返回 1，失败返回 0。 */
uint8_t RsMotor_BuildEnableFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildStopFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildClearFaultFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildSetZeroFrame(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildSetRunModeFrame(uint8_t motor_id, RsMotorRunMode mode,
                                     uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildSetProtocolFrame(uint8_t motor_id, RsMotorProtocol protocol,
                                      uint16_t *std_id, uint8_t data[8]);

uint8_t RsMotor_BuildPrivateEnableFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivateStopFrame(uint8_t motor_id, uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivateRunModeFrame(uint8_t motor_id, uint8_t run_mode,
                                         uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivateParamWriteFrame(uint8_t motor_id, uint16_t index, float value,
                                            uint32_t *ext_id, uint8_t data[8]);
uint8_t RsMotor_BuildPrivatePositionFrame(uint8_t motor_id, float position_rad,
                                          float limit_spd_rad_s, uint32_t *ext_id, uint8_t data[8]);

uint8_t RsMotor_BuildPositionFrame(uint8_t motor_id, float position_deg, float max_speed_deg_s,
                                   uint16_t *std_id, uint8_t data[8]);
uint8_t RsMotor_BuildPositionFrameFromTarget(uint8_t motor_id, uint16_t *std_id, uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif
