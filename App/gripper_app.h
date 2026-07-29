/**
 * @file    gripper_app.h
 * @brief   RS 电机夹爪两位置控制 App 配置与周期接口
 */

#ifndef GRIPPER_APP_H
#define GRIPPER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 用户可配置项。 */
#define GRIPPER_APP_CTRL_PERIOD_MS 10U
#define GRIPPER_APP_POS_CLOSE_RAD   0.0f
#define GRIPPER_APP_POS_OPEN_RAD   (-0.9f)
#define GRIPPER_APP_KP             20.0f
#define GRIPPER_APP_KD              0.5f

void GripperApp_Init(void);
void GripperApp_RunPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_APP_H */
