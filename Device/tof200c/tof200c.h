#ifndef TOF200C_H_
#define TOF200C_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "vl53l0x_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TOF200C_DEFAULT_I2C_ADDRESS_7BIT 0x29U

typedef enum {
  TOF200C_PROFILE_STANDARD = 0,
  TOF200C_PROFILE_HIGH_ACCURACY,
  TOF200C_PROFILE_LONG_RANGE,
  TOF200C_PROFILE_HIGH_SPEED
} tof200c_profile_t;

typedef enum {
  TOF200C_STATUS_OK = 0,
  TOF200C_STATUS_INVALID_ARGUMENT,
  TOF200C_STATUS_NOT_INITIALIZED,
  TOF200C_STATUS_NO_DATA,
  TOF200C_STATUS_STALE_DATA,
  TOF200C_STATUS_BUSY,
  TOF200C_STATUS_DEVICE_NOT_FOUND,
  TOF200C_STATUS_PAL_ERROR,
  TOF200C_STATUS_HAL_ERROR,
  TOF200C_STATUS_INTERRUPT_CLEAR_ERROR,
  TOF200C_STATUS_OFFLINE
} tof200c_status_t;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  GPIO_TypeDef *xshut_port;
  uint16_t xshut_pin;
  GPIO_TypeDef *int_port;
  uint16_t int_pin;
  uint8_t i2c_address_7bit;
  tof200c_profile_t profile;
  uint32_t stale_timeout_ms;
} tof200c_config_t;

typedef struct {
  uint16_t distance_mm;
  uint32_t signal_rate_mcps_q16_16;
  uint32_t ambient_rate_mcps_q16_16;
  uint16_t effective_spad_count_q8_8;
  uint8_t device_range_status;
  uint8_t range_status;
  bool range_valid;
  uint32_t timestamp_ms;
  uint32_t sample_count;
} tof200c_feedback_t;

typedef enum {
  TOF200C_TRANSFER_IDLE = 0,
  TOF200C_TRANSFER_READING_RESULT,
  TOF200C_TRANSFER_CLEAR_ASSERT,
  TOF200C_TRANSFER_CLEAR_RELEASE,
  TOF200C_TRANSFER_CHECK_INTERRUPT
} tof200c_transfer_state_t;

typedef enum {
  TOF200C_CONNECTION_UNINITIALIZED = 0,
  TOF200C_CONNECTION_STARTING,
  TOF200C_CONNECTION_ONLINE,
  TOF200C_CONNECTION_OFFLINE,
  TOF200C_CONNECTION_RECOVERING
} tof200c_connection_t;

typedef enum {
  TOF200C_OFFLINE_REASON_NONE = 0,
  TOF200C_OFFLINE_REASON_I2C_ERROR,
  TOF200C_OFFLINE_REASON_TRANSFER_TIMEOUT,
  TOF200C_OFFLINE_REASON_DATA_TIMEOUT,
  TOF200C_OFFLINE_REASON_SENSOR_INIT_ERROR,
  TOF200C_OFFLINE_REASON_INTERRUPT_CLEAR_ERROR
} tof200c_offline_reason_t;

typedef struct {
  volatile int32_t last_pal_error;
  volatile uint32_t last_hal_error;
  volatile tof200c_offline_reason_t offline_reason;
  volatile tof200c_transfer_state_t failed_transfer_state;
  volatile uint32_t i2c_busy_count;
  volatile uint32_t i2c_error_count;
  volatile uint32_t interrupt_clear_error_count;
  volatile uint32_t offline_count;
  volatile uint32_t recovery_count;
  volatile uint32_t transfer_timeout_count;
} tof200c_fault_t;

typedef struct {
  volatile tof200c_connection_t connection;
  volatile uint32_t changed_at_ms;
} tof200c_state_t;

typedef struct {
  VL53L0X_Dev_t pal_dev;
  uint8_t result_buffer[12];
  uint8_t clear_value;
  uint8_t interrupt_status;
  volatile tof200c_transfer_state_t transfer_state;
  volatile bool data_pending;
  uint8_t clear_attempts;
  uint32_t transfer_started_ms;
  uint32_t measurement_started_ms;
  uint32_t sample_count_at_start;
  uint32_t recovery_after_ms;
  uint32_t recovery_backoff_ms;
} tof200c_internal_t;

typedef struct {
  tof200c_config_t config;
  tof200c_internal_t internal;
  tof200c_state_t state;
  volatile tof200c_feedback_t feedback;
  tof200c_fault_t fault;
} tof200c_t;

/**
 * Blocking one-time initialization. It resets the sensor through XSHUT,
 * performs the ST PAL calibration and starts continuous ranging.
 */
tof200c_status_t tof200c_init(tof200c_t *device);

/**
 * Immediately copies the latest complete sample. No I2C access or wait occurs.
 */
tof200c_status_t tof200c_get_latest(const tof200c_t *device,
                                    tof200c_feedback_t *feedback);

/**
 * Normal operation is non-blocking. An offline recovery attempt may briefly
 * block while the ST PAL initialization and calibration are performed.
 * Call this function continuously from the main loop.
 */
void tof200c_process(tof200c_t *device);

/**
 * Blocking explicit recovery and shutdown operations.
 */
tof200c_status_t tof200c_recover(tof200c_t *device);
tof200c_status_t tof200c_deinit(tof200c_t *device);

/**
 * Forward the corresponding STM32 HAL callbacks to these functions.
 */
void tof200c_on_exti_callback(tof200c_t *device, uint16_t gpio_pin);
void tof200c_on_i2c_mem_rx_complete(tof200c_t *device,
                                    I2C_HandleTypeDef *hi2c);
void tof200c_on_i2c_mem_tx_complete(tof200c_t *device,
                                    I2C_HandleTypeDef *hi2c);
void tof200c_on_i2c_error(tof200c_t *device, I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif
