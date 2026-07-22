#ifndef RS_MOTOR_INSTANCES_H
#define RS_MOTOR_INSTANCES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs_motor.h"

typedef enum {
    RS_MOTOR_INSTANCE_RS00_ID3 = 0,
    RS_MOTOR_INSTANCE_COUNT,
} RsMotorInstanceId;

void RsMotorInstances_Init(void);
RsMotor *RsMotorInstances_Get(RsMotorInstanceId instance);

extern volatile RsMotorStatus g_rs_motor_instances_init_status;

#ifdef __cplusplus
}
#endif

#endif /* RS_MOTOR_INSTANCES_H */
