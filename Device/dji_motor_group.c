#include "dji_motor_group.h"

#include <stddef.h>
#include <string.h>

static dji_motor_group_t *s_group_list;

static uint8_t dji_motor_group_command_is_valid(uint16_t command_id)
{
    return (command_id == DJI_MOTOR_CMD_ID_1_TO_4 ||
            command_id == DJI_MOTOR_CMD_ID_5_TO_8) ? 1U : 0U;
}

static uint8_t dji_motor_group_id_matches(uint16_t command_id,
                                          uint8_t motor_id)
{
    if (command_id == DJI_MOTOR_CMD_ID_1_TO_4) {
        return (motor_id >= 1U && motor_id <= 4U) ? 1U : 0U;
    }
    if (command_id == DJI_MOTOR_CMD_ID_5_TO_8) {
        return (motor_id >= 5U && motor_id <= 8U) ? 1U : 0U;
    }
    return 0U;
}

static dji_motor_group_t *dji_motor_group_find_pointer(
    const dji_motor_group_t *group)
{
    dji_motor_group_t *current = s_group_list;

    while (current != NULL) {
        if (current == group) {
            return current;
        }
        current = current->internal.next;
    }
    return NULL;
}

static dji_motor_status_t dji_motor_group_require_initialized(
    const dji_motor_group_t *group)
{
    if (group == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (group->internal.initialized == 0U ||
        dji_motor_group_find_pointer(group) == NULL) {
        return DJI_MOTOR_STATUS_NOT_INITIALIZED;
    }
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_group_init(dji_motor_group_t *group)
{
    dji_motor_group_t *current;

    if (group == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (dji_motor_group_find_pointer(group) != NULL ||
        group->internal.initialized != 0U) {
        return DJI_MOTOR_STATUS_DUPLICATE_INSTANCE;
    }
    if (group->config.hfdcan == NULL ||
        dji_motor_group_command_is_valid(group->config.command_id) == 0U ||
        group->config.motor_count == 0U ||
        group->config.motor_count > DJI_MOTOR_GROUP_MAX_MOTORS) {
        return DJI_MOTOR_STATUS_INVALID_CONFIG;
    }

    current = s_group_list;
    while (current != NULL) {
        if (current->config.hfdcan == group->config.hfdcan &&
            current->config.command_id == group->config.command_id) {
            return DJI_MOTOR_STATUS_DUPLICATE_INSTANCE;
        }
        current = current->internal.next;
    }

    for (uint8_t i = 0U; i < group->config.motor_count; ++i) {
        dji_motor_t *motor = group->config.motors[i];

        if (motor == NULL || motor->internal.initialized == 0U) {
            return DJI_MOTOR_STATUS_INVALID_CONFIG;
        }
        if (motor->internal.group != NULL) {
            return DJI_MOTOR_STATUS_ALREADY_GROUPED;
        }
        if (dji_motor_group_id_matches(group->config.command_id,
                                       motor->config.motor_id) == 0U) {
            return DJI_MOTOR_STATUS_GROUP_MISMATCH;
        }
        for (uint8_t j = 0U; j < i; ++j) {
            if (group->config.motors[j] == motor ||
                group->config.motors[j]->config.motor_id ==
                    motor->config.motor_id) {
                return DJI_MOTOR_STATUS_DUPLICATE_INSTANCE;
            }
        }
    }

    memset(&group->state, 0, sizeof(group->state));
    memset(&group->internal, 0, sizeof(group->internal));
    group->internal.initialized = 1U;
    group->internal.next = s_group_list;
    s_group_list = group;
    for (uint8_t i = 0U; i < group->config.motor_count; ++i) {
        group->config.motors[i]->internal.group = group;
    }
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_group_deinit(dji_motor_group_t *group)
{
    dji_motor_group_t **link;

    if (group == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    link = &s_group_list;
    while (*link != NULL && *link != group) {
        link = &(*link)->internal.next;
    }
    if (*link == NULL || group->internal.initialized == 0U) {
        return DJI_MOTOR_STATUS_NOT_INITIALIZED;
    }

    *link = group->internal.next;
    for (uint8_t i = 0U; i < group->config.motor_count; ++i) {
        if (group->config.motors[i] != NULL &&
            group->config.motors[i]->internal.group == group) {
            group->config.motors[i]->internal.group = NULL;
        }
    }
    memset(&group->state, 0, sizeof(group->state));
    memset(&group->internal, 0, sizeof(group->internal));
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_group_build_frame(
    const dji_motor_group_t *group,
    uint8_t data[8])
{
    dji_motor_status_t status = dji_motor_group_require_initialized(group);
    uint8_t first_motor_id;

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }
    if (data == NULL) {
        return DJI_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    memset(data, 0, 8U);
    first_motor_id = (group->config.command_id == DJI_MOTOR_CMD_ID_1_TO_4) ?
                     1U : 5U;
    for (uint8_t i = 0U; i < group->config.motor_count; ++i) {
        const dji_motor_t *motor = group->config.motors[i];
        uint8_t slot = (uint8_t)(motor->config.motor_id - first_motor_id);
        uint16_t current = (uint16_t)motor->internal.current_command;

        data[slot * 2U] = (uint8_t)(current >> 8U);
        data[slot * 2U + 1U] = (uint8_t)current;
    }
    return DJI_MOTOR_STATUS_OK;
}

dji_motor_status_t dji_motor_group_update(dji_motor_group_t *group,
                                          uint32_t now_ms)
{
    dji_motor_status_t status = dji_motor_group_require_initialized(group);
    dji_motor_status_t first_error = DJI_MOTOR_STATUS_OK;
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8];

    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }

    for (uint8_t i = 0U; i < group->config.motor_count; ++i) {
        status = dji_motor_update(group->config.motors[i], now_ms);
        if (status != DJI_MOTOR_STATUS_OK) {
            group->config.motors[i]->internal.current_command = 0;
            if (first_error == DJI_MOTOR_STATUS_OK) {
                first_error = status;
            }
        }
    }
    status = dji_motor_group_build_frame(group, data);
    if (status != DJI_MOTOR_STATUS_OK) {
        return status;
    }

    header.Identifier = group->config.command_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(group->config.hfdcan,
                                      &header,
                                      data) != HAL_OK) {
        return DJI_MOTOR_STATUS_FDCAN_TX_ERROR;
    }
    group->state.tx_count++;
    group->state.last_tx_ms = now_ms;
    return first_error;
}

uint8_t dji_motor_group_handle_rx(FDCAN_HandleTypeDef *hfdcan,
                                  const FDCAN_RxHeaderTypeDef *header,
                                  const uint8_t data[8],
                                  uint32_t now_ms)
{
    dji_motor_group_t *group;
    uint8_t motor_id;

    if (hfdcan == NULL || header == NULL || data == NULL) {
        return 0U;
    }
    if (header->IdType != FDCAN_STANDARD_ID ||
        header->RxFrameType != FDCAN_DATA_FRAME ||
        header->DataLength != FDCAN_DLC_BYTES_8 ||
        header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->Identifier < DJI_MOTOR_FEEDBACK_ID_MIN ||
        header->Identifier > DJI_MOTOR_FEEDBACK_ID_MAX) {
        return 0U;
    }

    motor_id = (uint8_t)(header->Identifier -
                         DJI_MOTOR_FEEDBACK_ID_BASE + 1U);
    group = s_group_list;
    while (group != NULL) {
        if (group->config.hfdcan == hfdcan) {
            for (uint8_t i = 0U; i < group->config.motor_count; ++i) {
                dji_motor_t *motor = group->config.motors[i];

                if (motor->config.motor_id == motor_id) {
                    return (dji_motor_handle_feedback(motor,
                                                      data,
                                                      now_ms) ==
                            DJI_MOTOR_STATUS_OK) ? 1U : 0U;
                }
            }
        }
        group = group->internal.next;
    }
    return 0U;
}
