/**
 * @file    tof200c_app.c
 * @brief   TOF200C 设备配置、周期处理和最新反馈快照。
 */

#include "tof200c_app.h"

#include "i2c.h"
#include "main.h"
#include "tof200c.h"

static tof200c_t g_tof200c = {
    .config = {
        .hi2c = &hi2c2,
        .xshut_port = TOF_XSHUT_GPIO_Port,
        .xshut_pin = TOF_XSHUT_Pin,
        .int_port = TOF_INT_GPIO_Port,
        .int_pin = TOF_INT_Pin,
        .i2c_address_7bit = TOF200C_DEFAULT_I2C_ADDRESS_7BIT,
        .profile = TOF200C_PROFILE_HIGH_ACCURACY,
        .stale_timeout_ms = 100U,
    },
};

/* 保留原调试符号，便于在 GDB/IDE 中继续观察。 */
volatile tof200c_status_t g_tof200c_init_status =
    TOF200C_STATUS_NOT_INITIALIZED;
volatile tof200c_status_t g_tof200c_read_status =
    TOF200C_STATUS_NOT_INITIALIZED;
volatile tof200c_feedback_t g_tof200c_latest;

static uint32_t s_last_process_ms;
static uint8_t s_init_attempted;

void Tof200cApp_Init(void)
{
    if (s_init_attempted != 0U) {
        return;
    }

    g_tof200c_init_status = tof200c_init(&g_tof200c);
    s_last_process_ms = 0U;
    s_init_attempted = 1U;
}

void Tof200cApp_RunPeriodic(void)
{
    tof200c_feedback_t latest;
    uint32_t now_ms;

    if (s_init_attempted == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_process_ms) >=
        TOF200C_APP_PROCESS_PERIOD_MS) {
        tof200c_process(&g_tof200c);
        s_last_process_ms = now_ms;
    }

    g_tof200c_read_status = tof200c_get_latest(&g_tof200c, &latest);
    if (g_tof200c_read_status == TOF200C_STATUS_OK ||
        g_tof200c_read_status == TOF200C_STATUS_STALE_DATA) {
        g_tof200c_latest = latest;
    }
}

void Tof200cApp_OnExtiCallback(uint16_t gpio_pin)
{
    tof200c_on_exti_callback(&g_tof200c, gpio_pin);
}

void Tof200cApp_OnI2cMemRxComplete(I2C_HandleTypeDef *hi2c)
{
    tof200c_on_i2c_mem_rx_complete(&g_tof200c, hi2c);
}

void Tof200cApp_OnI2cMemTxComplete(I2C_HandleTypeDef *hi2c)
{
    tof200c_on_i2c_mem_tx_complete(&g_tof200c, hi2c);
}

void Tof200cApp_OnI2cError(I2C_HandleTypeDef *hi2c)
{
    tof200c_on_i2c_error(&g_tof200c, hi2c);
}
