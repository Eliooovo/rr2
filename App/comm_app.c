/**
 * @file    comm_app.c
 * @brief   USB CDC 固定帧解析和全局邮箱反馈发送
 */

#include "comm_app.h"

#include <string.h>

#include "main.h"
#include "usbd_cdc_if.h"

#define COMM_APP_FLOAT_COUNT   11U
#define COMM_APP_PACKET_SIZE   (1U + COMM_APP_FLOAT_COUNT * 4U + 1U)
#define COMM_APP_COMMAND_HEAD  0xAAU
#define COMM_APP_COMMAND_TAIL  0x55U
#define COMM_APP_FEEDBACK_HEAD 0xAAU
#define COMM_APP_FEEDBACK_TAIL 0x55U
#define COMM_APP_TX_BUFFER_COUNT 2U

#if COMM_APP_RX_BUFFER_SIZE <= COMM_APP_PACKET_SIZE
#error "COMM_APP_RX_BUFFER_SIZE must be larger than one complete packet"
#endif

volatile comm_app_command_t g_comm_app_command;
volatile comm_app_feedback_t g_comm_app_feedback;

static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_buffer[COMM_APP_RX_BUFFER_SIZE];
static uint8_t s_parse_buffer[COMM_APP_PACKET_SIZE];
static uint16_t s_parse_index;
static uint8_t
    s_feedback_packet[COMM_APP_TX_BUFFER_COUNT][COMM_APP_PACKET_SIZE];
static uint8_t s_active_tx_buffer;
static uint32_t s_last_feedback_ms;

static void CommApp_RxPush(uint8_t byte)
{
    uint16_t next =
        (uint16_t)((s_rx_head + 1U) % COMM_APP_RX_BUFFER_SIZE);

    if (next == s_rx_tail) {
        s_rx_tail =
            (uint16_t)((s_rx_tail + 1U) % COMM_APP_RX_BUFFER_SIZE);
    }

    s_rx_buffer[s_rx_head] = byte;
    __DMB();
    s_rx_head = next;
}

static uint8_t CommApp_RxPop(uint8_t *byte)
{
    uint16_t tail = s_rx_tail;

    if (tail == s_rx_head) {
        return 0U;
    }

    __DMB();
    *byte = s_rx_buffer[tail];
    s_rx_tail =
        (uint16_t)((tail + 1U) % COMM_APP_RX_BUFFER_SIZE);
    return 1U;
}

static float CommApp_ReadFloatLe(const uint8_t data[4])
{
    uint8_t native_bytes[sizeof(float)];
    float value;

    native_bytes[0] = data[0];
    native_bytes[1] = data[1];
    native_bytes[2] = data[2];
    native_bytes[3] = data[3];
    (void)memcpy(&value, native_bytes, sizeof(value));
    return value;
}

static void CommApp_WriteFloatLe(uint8_t data[4], float value)
{
    uint8_t native_bytes[sizeof(float)];

    (void)memcpy(native_bytes, &value, sizeof(value));
    data[0] = native_bytes[0];
    data[1] = native_bytes[1];
    data[2] = native_bytes[2];
    data[3] = native_bytes[3];
}

static void CommApp_UnpackCommand(
    const uint8_t packet[COMM_APP_PACKET_SIZE])
{
    float fields[COMM_APP_FLOAT_COUNT];
    uint32_t next_sequence = g_comm_app_command.sequence + 1U;

    for (uint8_t i = 0U; i < COMM_APP_FLOAT_COUNT; ++i) {
        fields[i] =
            CommApp_ReadFloatLe(&packet[1U + (uint16_t)i * 4U]);
    }

    g_comm_app_command.chassis_vx_m_s = fields[0];
    g_comm_app_command.chassis_vy_m_s = fields[1];
    g_comm_app_command.chassis_wz_rad_s = fields[2];
    g_comm_app_command.lift_front_position_m = fields[3];
    g_comm_app_command.lift_rear_position_m = fields[4];
    g_comm_app_command.kfs_lift_position_m = fields[5];
    g_comm_app_command.kfs_root_rotate_rad = fields[6];
    g_comm_app_command.kfs_tip_rotate_rad = fields[7];
    g_comm_app_command.kfs_grip_position_m = fields[8];
    g_comm_app_command.weapon_rotate_rad = fields[9];
    g_comm_app_command.weapon_grip_position_m = fields[10];
    __DMB();
    g_comm_app_command.sequence = next_sequence;
    g_comm_app_command.valid = 1U;
}

static void CommApp_ParseRx(void)
{
    uint8_t byte;

    while (CommApp_RxPop(&byte) != 0U) {
        if (s_parse_index == 0U && byte != COMM_APP_COMMAND_HEAD) {
            continue;
        }

        s_parse_buffer[s_parse_index] = byte;
        s_parse_index++;

        if (s_parse_index >= COMM_APP_PACKET_SIZE) {
            if (s_parse_buffer[0] == COMM_APP_COMMAND_HEAD &&
                s_parse_buffer[COMM_APP_PACKET_SIZE - 1U] ==
                    COMM_APP_COMMAND_TAIL) {
                CommApp_UnpackCommand(s_parse_buffer);
            }
            s_parse_index = 0U;
        }
    }
}

static void CommApp_PackFeedback(
    uint8_t packet[COMM_APP_PACKET_SIZE])
{
    float fields[COMM_APP_FLOAT_COUNT] = {0.0f};

    if (g_comm_app_feedback.chassis_valid != 0U) {
        fields[0] = g_comm_app_feedback.chassis_vx_m_s;
        fields[1] = g_comm_app_feedback.chassis_vy_m_s;
        fields[2] = g_comm_app_feedback.chassis_wz_rad_s;
    }
    if (g_comm_app_feedback.lift_valid != 0U) {
        fields[3] = g_comm_app_feedback.lift_front_position_m;
        fields[4] = g_comm_app_feedback.lift_rear_position_m;
    }
    if (g_comm_app_feedback.kfs_lift_valid != 0U) {
        fields[5] = g_comm_app_feedback.kfs_lift_position_m;
    }
    if (g_comm_app_feedback.kfs_root_rotate_valid != 0U) {
        fields[6] = g_comm_app_feedback.kfs_root_rotate_rad;
    }

    packet[0] = COMM_APP_FEEDBACK_HEAD;
    for (uint8_t i = 0U; i < COMM_APP_FLOAT_COUNT; ++i) {
        CommApp_WriteFloatLe(&packet[1U + (uint16_t)i * 4U],
                             fields[i]);
    }
    packet[COMM_APP_PACKET_SIZE - 1U] = COMM_APP_FEEDBACK_TAIL;
}

static void CommApp_SendFeedbackPeriodic(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t next_tx_buffer;

    if ((uint32_t)(now_ms - s_last_feedback_ms) <
        COMM_APP_FEEDBACK_PERIOD_MS) {
        return;
    }

    next_tx_buffer =
        (uint8_t)((s_active_tx_buffer + 1U) %
                  COMM_APP_TX_BUFFER_COUNT);
    CommApp_PackFeedback(s_feedback_packet[next_tx_buffer]);
    if (CDC_Transmit_HS(s_feedback_packet[next_tx_buffer],
                        COMM_APP_PACKET_SIZE) == USBD_OK) {
        s_active_tx_buffer = next_tx_buffer;
        s_last_feedback_ms = now_ms;
    }
}

void CommApp_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_parse_index = 0U;
    s_active_tx_buffer = 0U;
    s_last_feedback_ms = 0U;

    g_comm_app_command.chassis_vx_m_s = 0.0f;
    g_comm_app_command.chassis_vy_m_s = 0.0f;
    g_comm_app_command.chassis_wz_rad_s = 0.0f;
    g_comm_app_command.lift_front_position_m = 0.0f;
    g_comm_app_command.lift_rear_position_m = 0.0f;
    g_comm_app_command.kfs_lift_position_m = 0.0f;
    g_comm_app_command.kfs_root_rotate_rad = 0.0f;
    g_comm_app_command.kfs_tip_rotate_rad = 0.0f;
    g_comm_app_command.kfs_grip_position_m = 0.0f;
    g_comm_app_command.weapon_rotate_rad = 0.0f;
    g_comm_app_command.weapon_grip_position_m = 0.0f;
    g_comm_app_command.sequence = 0U;
    g_comm_app_command.valid = 0U;

    g_comm_app_feedback.kfs_lift_position_m = 0.0f;
    g_comm_app_feedback.kfs_root_rotate_rad = 0.0f;
    g_comm_app_feedback.kfs_tip_rotate_rad = 0.0f;
    g_comm_app_feedback.kfs_grip_position_m = 0.0f;
    g_comm_app_feedback.weapon_rotate_rad = 0.0f;
    g_comm_app_feedback.weapon_grip_position_m = 0.0f;
    g_comm_app_feedback.kfs_lift_valid = 0U;
    g_comm_app_feedback.kfs_root_rotate_valid = 0U;
}

void CommApp_RunPeriodic(void)
{
    CommApp_ParseRx();
    CommApp_SendFeedbackPeriodic();
}

void CommApp_OnUsbReceived(const uint8_t *data, uint32_t len)
{
    if (data == NULL) {
        return;
    }

    for (uint32_t i = 0U; i < len; ++i) {
        CommApp_RxPush(data[i]);
    }
}
