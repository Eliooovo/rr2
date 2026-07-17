#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DJI_MOTOR_COUNT             8U
#define DJI_MOTOR_FEEDBACK_ID_BASE  0x201U
#define DJI_MOTOR_FEEDBACK_ID_MIN   0x201U
#define DJI_MOTOR_FEEDBACK_ID_MAX   0x208U
#define DJI_MOTOR_CMD_ID_1_TO_4     0x200U
#define DJI_MOTOR_CMD_ID_5_TO_8     0x1FFU
#define DJI_MOTOR_ENCODER_RANGE     8192
#define DJI_MOTOR_ENCODER_HALF      4096
#define DJI_MOTOR_CURRENT_LIMIT     16384

typedef struct {
    uint8_t id;                /* 电调 ID，范围 1~8 */
    uint8_t online;            /* 收到过反馈后置 1 */
    uint16_t encoder;          /* C620 单圈编码器值，范围 0~8191 */
    uint16_t last_encoder;     /* 上一次反馈的单圈编码器值 */
    int16_t speed_rpm;         /* C620 反馈转速，单位 rpm */
    int16_t given_current;     /* C620 反馈电流值 */
    uint8_t temperature;       /* C620 反馈温度 */
    int32_t round_count;       /* 多圈累计圈数 */
    int32_t total_encoder;     /* 多圈累计编码器值 */
    float total_angle_deg;     /* 多圈累计角度，单位度 */
    uint32_t update_count;     /* 收到该电机反馈的次数 */
    uint32_t last_update_ms;   /* 最近一次反馈时间，由外部传入 */
} DjiMotorState;

extern DjiMotorState g_dji_motors[DJI_MOTOR_COUNT];

void DjiMotor_Init(void);
uint8_t DjiMotor_IsFeedbackId(uint16_t std_id);
uint8_t DjiMotor_GetMotorIdFromFeedbackId(uint16_t std_id);

/* 解析 0x201~0x208 反馈帧；返回解析到的电机 ID，非 DJI 反馈则返回 0。 */
uint8_t DjiMotor_HandleFeedback(uint16_t std_id, const uint8_t data[8], uint32_t now_ms);

DjiMotorState *DjiMotor_GetState(uint8_t motor_id);
void DjiMotor_ClearCurrents(void);

/* 设置单个电机电流命令，只更新缓存，不直接发送 CAN。 */
void DjiMotor_SetCurrent(uint8_t motor_id, int16_t current);
int16_t DjiMotor_GetCurrent(uint8_t motor_id);

/* 按 DJI 协议把 4 个电机电流打包成 8 字节数据。 */
uint8_t DjiMotor_BuildCurrentFrame(uint16_t cmd_id, uint8_t data[8]);
void DjiMotor_BuildAllCurrentFrames(uint8_t data_1_to_4[8], uint8_t data_5_to_8[8]);

#ifdef __cplusplus
}
#endif

#endif
