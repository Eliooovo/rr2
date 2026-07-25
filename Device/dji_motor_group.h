#ifndef DJI_MOTOR_GROUP_H
#define DJI_MOTOR_GROUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "dji_motor.h"
#include "fdcan.h"

#define DJI_MOTOR_GROUP_MAX_MOTORS 4U
#define DJI_MOTOR_CMD_ID_1_TO_4    0x200U
#define DJI_MOTOR_CMD_ID_5_TO_8    0x1FFU

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint16_t command_id; /**< 只能为 0x200 或 0x1ff。 */
    uint8_t motor_count; /**< 有效范围 1..4。 */
    dji_motor_t *motors[DJI_MOTOR_GROUP_MAX_MOTORS];
} dji_motor_group_config_t;

typedef struct {
    uint32_t tx_count;
    uint32_t last_tx_ms;
} dji_motor_group_state_t;

typedef struct {
    uint8_t initialized;
    dji_motor_group_t *next;
} dji_motor_group_internal_t;

struct dji_motor_group {
    dji_motor_group_config_t config;
    dji_motor_group_state_t state;
    dji_motor_group_internal_t internal;
};

dji_motor_status_t dji_motor_group_init(dji_motor_group_t *group);
dji_motor_status_t dji_motor_group_deinit(dji_motor_group_t *group);

/** 按电机 ID 放入协议固定槽位；未配置的槽位自动填零。 */
dji_motor_status_t dji_motor_group_build_frame(
    const dji_motor_group_t *group,
    uint8_t data[8]);

/** 更新组内所有电机控制器并发送一帧聚合电流命令。 */
dji_motor_status_t dji_motor_group_update(dji_motor_group_t *group,
                                          uint32_t now_ms);

/**
 * 从已注册 Group 中按 FDCAN + 反馈 ID 分发一帧。
 *
 * @return 1 表示已匹配并处理，0 表示帧非法或没有匹配实例。
 */
uint8_t dji_motor_group_handle_rx(FDCAN_HandleTypeDef *hfdcan,
                                  const FDCAN_RxHeaderTypeDef *header,
                                  const uint8_t data[8],
                                  uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* DJI_MOTOR_GROUP_H */
