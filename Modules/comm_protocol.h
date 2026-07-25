#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

/*
 * CubeMX 管理的 usbd_cdc_if.c 已在用户区包含本头文件并调用旧回调名。
 * 保留这一层只为兼容生成文件；应用代码统一使用 comm_app.h。
 */
#include "comm_app.h"

static inline void Comm_OnUsbReceived(const uint8_t *data, uint32_t len)
{
    CommApp_OnUsbReceived(data, len);
}

#endif /* COMM_PROTOCOL_H */
