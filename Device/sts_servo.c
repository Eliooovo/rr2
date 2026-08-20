/**
 * @file    sts_servo.c
 * @brief   STS 系列舵机驱动，基于 SCSLib SMS_STS 协议层。
 *
 * 该文件同时提供 SCSLib 需要的 5 个硬件抽象函数
 *（readSCS / writeSCS / writeByteSCS / rFlushSCS / wFlushSCS），
 * 替代官方 SCSerail.c，因此不能将 SCSerail.c 一并编译。
 */

#include "sts_servo.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- SCSLib 官方头文件 --------------------------------------------------- */
#include "INST.h"
#include "SCS.h"
#include "SMS_STS.h"

#define STS_SERVO_UART_TX_TIMEOUT_MS 5U
#define STS_SERVO_UART_RX_TIMEOUT_MS 5U

/* ========================================================================
 * 硬件抽象层 —— 供 SCSLib 全局调用
 * ======================================================================== */

/** 当前操作的 UART 句柄，由 sts_servo_init() 设置。 */
static UART_HandleTypeDef *g_huart;

/** 发送缓冲，供 writeSCS / writeByteSCS 逐字节累积。 */
static uint8_t g_tx_buf[128];
static uint8_t g_tx_len;
static volatile uint32_t s_uart_rx_error_count;
static volatile HAL_StatusTypeDef s_last_uart_rx_status = HAL_OK;
static volatile uint32_t s_last_uart_rx_error_code = HAL_UART_ERROR_NONE;
static volatile uint32_t s_uart_tx_error_count;
static volatile HAL_StatusTypeDef s_last_uart_tx_status = HAL_OK;

/**
 * @brief 从 UART 阻塞读取 nLen 字节。
 * @return 成功读取的字节数，失败返回 0。
 */
int readSCS(uint8_t *nDat, int nLen)
{
    s_last_uart_rx_status = HAL_UART_Receive(g_huart,
                                             nDat,
                                             nLen,
                                             STS_SERVO_UART_RX_TIMEOUT_MS);
    if (s_last_uart_rx_status != HAL_OK) {
        s_last_uart_rx_error_code = g_huart->ErrorCode;
        s_uart_rx_error_count++;
        return 0;
    }
    s_last_uart_rx_error_code = HAL_UART_ERROR_NONE;
    return nLen;
}

/**
 * @brief 将数据追加到发送缓冲。
 * @return 当前缓冲长度。
 */
int writeSCS(uint8_t *nDat, int nLen)
{
    while (nLen--) {
        if (g_tx_len < sizeof(g_tx_buf)) {
            g_tx_buf[g_tx_len] = *nDat;
            g_tx_len++;
            nDat++;
        }
    }
    return g_tx_len;
}

/**
 * @brief 将单字节追加到发送缓冲。
 * @return 当前缓冲长度。
 */
int writeByteSCS(unsigned char bDat)
{
    if (g_tx_len < sizeof(g_tx_buf)) {
        g_tx_buf[g_tx_len] = bDat;
        g_tx_len++;
    }
    return g_tx_len;
}

/**
 * @brief 清除上一次事务残留的接收数据和 UART 错误。
 */
void rFlushSCS(void)
{
    if (g_huart == NULL) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(g_huart,
                          UART_CLEAR_PEF | UART_CLEAR_FEF |
                          UART_CLEAR_NEF | UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(g_huart, UART_RXDATA_FLUSH_REQUEST);
}

/**
 * @brief 将缓冲数据通过 UART 发出。
 */
void wFlushSCS(void)
{
    if (g_tx_len) {
        s_last_uart_tx_status = HAL_UART_Transmit(g_huart,
                                                  g_tx_buf,
                                                  g_tx_len,
                                                  STS_SERVO_UART_TX_TIMEOUT_MS);
        if (s_last_uart_tx_status != HAL_OK) {
            s_uart_tx_error_count++;
        }
        g_tx_len = 0;
    }
}

/* ========================================================================
 * 辅助函数
 * ======================================================================== */

/** STS3215 编码器分辨率：4096 步/圈。 */
#define STS_ENCODER_STEPS 4096.0f

/**
 * @brief 弧度 → 舵机原始位置值 (0 ~ 4095)。
 */
static inline int16_t rad_to_raw(float pos_rad)
{
    float raw_f = pos_rad * STS_ENCODER_STEPS / (2.0f * (float)M_PI);
    if (raw_f < 0.0f) {
        raw_f += STS_ENCODER_STEPS;
    }
    if (raw_f >= STS_ENCODER_STEPS) {
        raw_f -= STS_ENCODER_STEPS;
    }
    return (int16_t)raw_f;
}

/**
 * @brief 舵机原始位置值 (0 ~ 4095) → 弧度。
 */
static inline float raw_to_rad(int16_t raw)
{
    return (float)raw * 2.0f * (float)M_PI / STS_ENCODER_STEPS;
}

/* ========================================================================
 * 公开 API
 * ======================================================================== */

sts_servo_status_t sts_servo_init(sts_servo_t *servo)
{
    if (!servo) {
        return STS_SERVO_ERROR_PARAM;
    }
    if (!servo->config.huart) {
        return STS_SERVO_ERROR_CONFIG;
    }
    if (servo->config.id == 0 || servo->config.id > 253) {
        return STS_SERVO_ERROR_CONFIG;
    }
    if (servo->config.offline_timeout_ms == 0U) {
        return STS_SERVO_ERROR_CONFIG;
    }

    /* 设置全局 UART 句柄，供硬件抽象层使用。 */
    g_huart = servo->config.huart;

    /* 清空内部状态。 */
    memset(&servo->feedback, 0, sizeof(servo->feedback));
    memset(&servo->state, 0, sizeof(servo->state));
    servo->internal.tx_len = 0;
    servo->internal.last_update_ms = 0;

    /* 协议层默认：小端，写指令开启应答。 */
    setEnd(0);
    setLevel(1);

    /* Ping 检测在线。 */
    if (Ping(servo->config.id) == -1) {
        servo->internal.initialized = 0;
        servo->state.online = 0;
        return STS_SERVO_ERROR_TIMEOUT;
    }

    servo->internal.last_update_ms = HAL_GetTick();
    servo->state.online = 1U;
    servo->internal.initialized = 1;
    return STS_SERVO_OK;
}

sts_servo_status_t sts_servo_enable(sts_servo_t *servo)
{
    if (!servo || !servo->internal.initialized) {
        return STS_SERVO_ERROR_NOT_INIT;
    }

    g_huart = servo->config.huart;

    if (!writeByte(servo->config.id, SMS_STS_TORQUE_ENABLE, 1)) {
        return STS_SERVO_ERROR_TIMEOUT;
    }

    servo->state.enabled = 1;
    return STS_SERVO_OK;
}

sts_servo_status_t sts_servo_disable(sts_servo_t *servo)
{
    if (!servo || !servo->internal.initialized) {
        return STS_SERVO_ERROR_NOT_INIT;
    }

    g_huart = servo->config.huart;

    if (!writeByte(servo->config.id, SMS_STS_TORQUE_ENABLE, 0)) {
        return STS_SERVO_ERROR_TIMEOUT;
    }

    servo->state.enabled = 0;
    return STS_SERVO_OK;
}

sts_servo_status_t sts_servo_set_position(sts_servo_t *servo,
                                          float pos_rad,
                                          uint16_t speed,
                                          uint8_t acc)
{
    if (!servo || !servo->internal.initialized) {
        return STS_SERVO_ERROR_NOT_INIT;
    }

    g_huart = servo->config.huart;

    int16_t raw = rad_to_raw(pos_rad);

    if (WritePosEx(servo->config.id, raw, speed, acc) != 1) {
        return STS_SERVO_ERROR_TIMEOUT;
    }

    return STS_SERVO_OK;
}

sts_servo_status_t sts_servo_set_position_raw(sts_servo_t *servo,
                                              int16_t raw,
                                              uint16_t speed,
                                              uint8_t acc)
{
    if (!servo || !servo->internal.initialized) {
        return STS_SERVO_ERROR_NOT_INIT;
    }

    g_huart = servo->config.huart;

    if (WritePosEx(servo->config.id, raw, speed, acc) != 1) {
        return STS_SERVO_ERROR_TIMEOUT;
    }

    return STS_SERVO_OK;
}

sts_servo_status_t sts_servo_get_feedback(sts_servo_t *servo,
                                          sts_servo_feedback_t *fb)
{
    if (!servo || !servo->internal.initialized) {
        return STS_SERVO_ERROR_NOT_INIT;
    }
    if (!fb) {
        return STS_SERVO_ERROR_PARAM;
    }

    g_huart = servo->config.huart;

    /* 批量读取 56~70 共 15 字节。 */
    uint8_t buf[15];
    int ret = Read(servo->config.id, SMS_STS_PRESENT_POSITION_L, buf, sizeof(buf));
    if (ret != sizeof(buf)) {
        return STS_SERVO_ERROR_TIMEOUT;
    }

    /* 解析各字段（小端序，SCS2Host 负责转换）。 */
    int16_t pos_raw   = (int16_t)SCS2Host(buf[0], buf[1]);
    int16_t speed_raw = (int16_t)SCS2Host(buf[2], buf[3]);
    int16_t load_raw  = (int16_t)SCS2Host(buf[4], buf[5]);
    int16_t current_raw = (int16_t)SCS2Host(buf[13], buf[14]);

    fb->pos_rad        = raw_to_rad(pos_raw);
    fb->speed_rad_s    = (float)speed_raw * 2.0f * (float)M_PI / 60.0f;
    fb->load           = load_raw;
    fb->current_raw    = current_raw;
    fb->voltage_x10    = buf[6];
    fb->temperature_c  = buf[7];
    fb->status         = buf[9];
    fb->moving         = buf[10];

    /* 同步写入对象内部 feedback。 */
    memcpy(&servo->feedback, fb, sizeof(*fb));
    servo->internal.last_update_ms = HAL_GetTick();
    servo->state.online = 1U;

    return STS_SERVO_OK;
}

sts_servo_status_t sts_servo_update(sts_servo_t *servo, uint32_t now_ms)
{
    if (!servo || !servo->internal.initialized) {
        return STS_SERVO_ERROR_NOT_INIT;
    }

    /* 从最近一次成功通信起超时才标记离线。 */
    uint32_t elapsed;
    if (now_ms >= servo->internal.last_update_ms) {
        elapsed = now_ms - servo->internal.last_update_ms;
    } else {
        /* HAL tick 回绕。 */
        elapsed = (0xFFFFFFFFUL - servo->internal.last_update_ms) + now_ms + 1;
    }

    if (elapsed >= servo->config.offline_timeout_ms) {
        servo->state.online = 0;
    }

    return STS_SERVO_OK;
}
