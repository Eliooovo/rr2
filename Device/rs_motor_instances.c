#include "rs_motor_instances.h"

#include <stddef.h>

#include "fdcan.h"

/* All real RobStride motor objects live here. The protocol driver only knows
 * how a RobStride motor talks; this file decides which physical motors exist. */
static RsMotor s_rs00_id3_motor = {
    .config = {
        .hfdcan = &hfdcan3,
        .motor_id = 3U,
        .host_id = RS_MOTOR_DEFAULT_HOST_ID,
        .model = RS_MOTOR_MODEL_RS00,
        .protocol = RS_MOTOR_PROTOCOL_PRIVATE,
        .params = {
            .model = RS_MOTOR_MODEL_RS00,
            .position_min_rad = -12.57f,
            .position_max_rad = 12.57f,
            .speed_min_rad_s = -33.0f,
            .speed_max_rad_s = 33.0f,
            .torque_min_nm = -14.0f,
            .torque_max_nm = 14.0f,
        },
        .offline_timeout_ms = RS_MOTOR_DEFAULT_OFFLINE_MS,
    },
};

volatile RsMotorStatus g_rs_motor_instances_init_status =
    RS_MOTOR_STATUS_NOT_INITIALIZED;

void RsMotorInstances_Init(void)
{
    g_rs_motor_instances_init_status = rs_motor_init(&s_rs00_id3_motor);
}

RsMotor *RsMotorInstances_Get(RsMotorInstanceId instance)
{
    if (instance == RS_MOTOR_INSTANCE_RS00_ID3) {
        return &s_rs00_id3_motor;
    }
    return NULL;
}
