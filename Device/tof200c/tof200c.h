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

/*
 * 传感器恢复状态机步骤。
 *
 * 把原本一次完成的启动序列拆分成离散状态，供开机初始化和上层
 * 显式调用 tof200c_recover() 时复用。App 主循环不会自动启动它。
 *
 * 两步"等待"状态（XSHUT_LOW_WAIT、XSHUT_HIGH_WAIT）只检查时间，不阻塞。
 * 其余"动作"状态执行一个 VL53L0X API 调用；ST API 内部使用同步 I2C，
 * 通信异常时可能阻塞至平台超时。因此仅允许用于主循环启动前或显式恢复。
 *
 * 流程：
 *   IDLE → XSHUT_LOW_WAIT(30ms) → XSHUT_HIGH_WAIT(30ms)
 *        → READ_MODEL_ID → DATA_INIT → STATIC_INIT
 *        → APPLY_PROFILE → CONFIG_GPIO → START_MEASUREMENT → DONE
 *        → IDLE（恢复完成，切换到 ONLINE）
 */
typedef enum {
  TOF200C_RECOVERY_IDLE = 0,            /* 未在恢复 */
  TOF200C_RECOVERY_XSHUT_LOW_WAIT,      /* XSHUT 拉低（传感器断电关机）后等待 30ms */
  TOF200C_RECOVERY_XSHUT_HIGH_WAIT,     /* XSHUT 拉高（传感器上电启动）后等待 30ms */
  TOF200C_RECOVERY_READ_MODEL_ID,       /* 读 Model ID 确认设备存在 */
  TOF200C_RECOVERY_DATA_INIT,           /* VL53L0X_DataInit */
  TOF200C_RECOVERY_STATIC_INIT,         /* VL53L0X_StaticInit（含/跳过 SPAD 管理） */
  TOF200C_RECOVERY_APPLY_PROFILE,       /* 应用测距 Profile */
  TOF200C_RECOVERY_CONFIG_GPIO,         /* 配置 GPIO 中断输出 */
  TOF200C_RECOVERY_START_MEASUREMENT,   /* 清除中断掩码，启动连续测距 */
  TOF200C_RECOVERY_DONE                 /* 恢复成功 → 切换到 ONLINE */
} tof200c_recovery_step_t;

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
  /* ---- 开机/显式恢复状态机 ---- */
  tof200c_recovery_step_t recovery_step;/* 当前恢复步骤（IDLE = 未在恢复） */
  uint32_t recovery_step_start_ms;      /* 当前步骤开始时刻 */
  bool recovery_is_light;               /* true=轻量恢复跳过SPAD校准, false=全量 */
  bool reset_i2c;                       /* 是否在 XSHUT 后重置 I2C 外设 */
  bool count_recovery;                  /* 是否计入 recovery_count */
  /* ---- 校准缓存（首次全量初始化后保存，供轻量恢复复用） ---- */
  uint8_t cached_vhv_settings;          /* VHV 校准值 */
  uint8_t cached_phase_cal;             /* 相位校准值 */
  uint32_t cached_ref_spad_count;       /* 参考 SPAD 数量 */
  uint8_t cached_is_aperture_spads;     /* 是否使用孔径 SPAD */
  bool calibration_cached;              /* 上述缓存是否有效 */
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
 * Normal online operation uses interrupt-driven I2C and is non-blocking.
 * Once the device becomes offline, this function leaves it offline and
 * does not start automatic recovery from the main loop.
 */
void tof200c_process(tof200c_t *device);

/**
 * Blocking explicit recovery and shutdown operations. The application does
 * not call tof200c_recover() automatically; normal recovery therefore needs
 * a controller restart unless a higher layer explicitly requests it.
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
