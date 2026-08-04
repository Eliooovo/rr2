#include "tof200c.h"

#include <string.h>

#include "vl53l0x_api.h"
#include "vl53l0x_device.h"

#define TOF200C_RESULT_REGISTER 0x14U
#define TOF200C_INTERRUPT_STATUS_REGISTER 0x13U
#define TOF200C_INTERRUPT_CLEAR_REGISTER 0x0BU
#define TOF200C_MODEL_ID 0xEEAAU
#define TOF200C_XSHUT_DELAY_MS 30U
#define TOF200C_TRANSFER_TIMEOUT_MS 20U
#define TOF200C_DATA_TIMEOUT_MS 500U
#define TOF200C_RECOVERY_RETRY_MS 5000U
#define TOF200C_RECOVERY_BACKOFF_BASE_MS 5000U
#define TOF200C_RECOVERY_BACKOFF_MAX_MS 60000U
#define TOF200C_MAX_CLEAR_ATTEMPTS 3U

typedef struct {
  FixPoint1616_t signal_limit_mcps;
  FixPoint1616_t sigma_limit_mm;
  uint32_t timing_budget_us;
  uint8_t pre_range_vcsel_period;
  uint8_t final_range_vcsel_period;
} tof200c_profile_config_t;

static const tof200c_profile_config_t k_profile_configs[] = {
    [TOF200C_PROFILE_STANDARD] =
        {
            .signal_limit_mcps = 16384U,
            .sigma_limit_mm = 1179648U,
            .timing_budget_us = 33000U,
            .pre_range_vcsel_period = 14U,
            .final_range_vcsel_period = 10U,
        },
    [TOF200C_PROFILE_HIGH_ACCURACY] =
        {
            .signal_limit_mcps = 16384U,
            .sigma_limit_mm = 1179648U,
            .timing_budget_us = 200000U,
            .pre_range_vcsel_period = 14U,
            .final_range_vcsel_period = 10U,
        },
    [TOF200C_PROFILE_LONG_RANGE] =
        {
            .signal_limit_mcps = 6554U,
            .sigma_limit_mm = 3932160U,
            .timing_budget_us = 33000U,
            .pre_range_vcsel_period = 18U,
            .final_range_vcsel_period = 14U,
        },
    [TOF200C_PROFILE_HIGH_SPEED] =
        {
            .signal_limit_mcps = 16384U,
            .sigma_limit_mm = 2097152U,
            .timing_budget_us = 20000U,
            .pre_range_vcsel_period = 14U,
            .final_range_vcsel_period = 10U,
        },
};

static uint32_t tof200c_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void tof200c_exit_critical(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static bool tof200c_pin_is_single(uint16_t pin)
{
  return (pin != 0U) && ((pin & (uint16_t)(pin - 1U)) == 0U);
}

static bool tof200c_config_is_valid(const tof200c_config_t *config)
{
  return (config != NULL) && (config->hi2c != NULL) &&
         (config->xshut_port != NULL) && (config->int_port != NULL) &&
         tof200c_pin_is_single(config->xshut_pin) &&
         tof200c_pin_is_single(config->int_pin) &&
         (config->i2c_address_7bit != 0U) &&
         (config->i2c_address_7bit <= 0x7FU) &&
         (config->profile <= TOF200C_PROFILE_HIGH_SPEED);
}

static tof200c_status_t tof200c_set_pal_error(tof200c_t *device,
                                              VL53L0X_Error pal_error)
{
  device->fault.last_pal_error = (int32_t)pal_error;
  return TOF200C_STATUS_PAL_ERROR;
}

static bool tof200c_time_reached(uint32_t now_ms, uint32_t target_ms)
{
  return (int32_t)(now_ms - target_ms) >= 0;
}

static bool tof200c_time_elapsed(uint32_t now_ms, uint32_t start_ms,
                                uint32_t timeout_ms)
{
  return (uint32_t)(now_ms - start_ms) > timeout_ms;
}

static void tof200c_set_connection(tof200c_t *device,
                                   tof200c_connection_t connection)
{
  uint32_t primask = tof200c_enter_critical();

  device->state.connection = connection;
  device->state.changed_at_ms = HAL_GetTick();
  tof200c_exit_critical(primask);
}

static void tof200c_mark_offline(tof200c_t *device,
                                 tof200c_offline_reason_t reason,
                                 tof200c_transfer_state_t failed_transfer)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t primask = tof200c_enter_critical();
  tof200c_connection_t previous = device->state.connection;

  if ((previous == TOF200C_CONNECTION_ONLINE) ||
      (previous == TOF200C_CONNECTION_STARTING)) {
    device->fault.offline_count++;
  }
  device->fault.offline_reason = reason;
  device->fault.failed_transfer_state = failed_transfer;
  device->internal.transfer_state = TOF200C_TRANSFER_IDLE;
  device->internal.data_pending = false;
  if (device->internal.recovery_backoff_ms == 0U) {
    device->internal.recovery_backoff_ms = TOF200C_RECOVERY_BACKOFF_BASE_MS;
  } else {
    uint32_t next = device->internal.recovery_backoff_ms * 2U;
    device->internal.recovery_backoff_ms =
        (next > TOF200C_RECOVERY_BACKOFF_MAX_MS) ? TOF200C_RECOVERY_BACKOFF_MAX_MS
                                                  : next;
  }
  device->internal.recovery_after_ms = now_ms + device->internal.recovery_backoff_ms;
  device->state.connection = TOF200C_CONNECTION_OFFLINE;
  device->state.changed_at_ms = now_ms;
  tof200c_exit_critical(primask);
}

static uint8_t tof200c_map_range_status(uint8_t device_range_status)
{
  uint8_t status = (uint8_t)((device_range_status & 0x78U) >> 3);

  if ((status == 0U) || (status == 5U) || (status == 7U) ||
      (status >= 12U)) {
    return 255U;
  }
  if ((status >= 1U) && (status <= 3U)) {
    return 5U;
  }
  if ((status == 6U) || (status == 9U)) {
    return 4U;
  }
  if ((status == 8U) || (status == 10U)) {
    return 3U;
  }
  if (status == 4U) {
    return 2U;
  }
  return 0U;
}

static uint16_t tof200c_read_be_u16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void tof200c_publish_result(tof200c_t *device)
{
  const uint8_t *raw = device->internal.result_buffer;
  tof200c_feedback_t feedback;
  uint32_t primask;

  feedback.distance_mm = tof200c_read_be_u16(&raw[10]);
  feedback.signal_rate_mcps_q16_16 =
      (uint32_t)tof200c_read_be_u16(&raw[6]) << 9;
  feedback.ambient_rate_mcps_q16_16 =
      (uint32_t)tof200c_read_be_u16(&raw[8]) << 9;
  feedback.effective_spad_count_q8_8 = tof200c_read_be_u16(&raw[2]);
  feedback.device_range_status = raw[0];
  feedback.range_status = tof200c_map_range_status(raw[0]);
  feedback.range_valid = (feedback.range_status == 0U);
  feedback.timestamp_ms = HAL_GetTick();

  primask = tof200c_enter_critical();
  feedback.sample_count = device->feedback.sample_count + 1U;
  device->feedback.distance_mm = feedback.distance_mm;
  device->feedback.signal_rate_mcps_q16_16 =
      feedback.signal_rate_mcps_q16_16;
  device->feedback.ambient_rate_mcps_q16_16 =
      feedback.ambient_rate_mcps_q16_16;
  device->feedback.effective_spad_count_q8_8 =
      feedback.effective_spad_count_q8_8;
  device->feedback.device_range_status = feedback.device_range_status;
  device->feedback.range_status = feedback.range_status;
  device->feedback.range_valid = feedback.range_valid;
  device->feedback.timestamp_ms = feedback.timestamp_ms;
  device->feedback.sample_count = feedback.sample_count;
  tof200c_exit_critical(primask);
}

static void tof200c_record_async_start_failure(tof200c_t *device,
                                                HAL_StatusTypeDef hal_status)
{
  tof200c_transfer_state_t failed_transfer =
      device->internal.transfer_state;

  device->fault.last_hal_error = HAL_I2C_GetError(device->config.hi2c);
  if (hal_status == HAL_BUSY) {
    device->fault.i2c_busy_count++;
  } else {
    device->fault.i2c_error_count++;
  }
  tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_I2C_ERROR,
                       failed_transfer);
}

static void tof200c_begin_clear_assert(tof200c_t *device)
{
  HAL_StatusTypeDef hal_status;

  device->internal.clear_attempts++;
  device->internal.clear_value = 1U;
  device->internal.transfer_state = TOF200C_TRANSFER_CLEAR_ASSERT;
  device->internal.transfer_started_ms = HAL_GetTick();
  hal_status = HAL_I2C_Mem_Write_IT(
      device->config.hi2c, device->internal.pal_dev.I2cDevAddr,
      TOF200C_INTERRUPT_CLEAR_REGISTER, I2C_MEMADD_SIZE_8BIT,
      &device->internal.clear_value, 1U);
  if (hal_status != HAL_OK) {
    tof200c_record_async_start_failure(device, hal_status);
  }
}

static void tof200c_start_result_read(tof200c_t *device)
{
  HAL_StatusTypeDef hal_status;
  uint32_t primask;

  primask = tof200c_enter_critical();
  if ((device->internal.transfer_state != TOF200C_TRANSFER_IDLE) ||
      (device->state.connection != TOF200C_CONNECTION_ONLINE)) {
    tof200c_exit_critical(primask);
    return;
  }
  device->internal.transfer_state = TOF200C_TRANSFER_READING_RESULT;
  device->internal.data_pending = false;
  device->internal.transfer_started_ms = HAL_GetTick();
  tof200c_exit_critical(primask);

  hal_status = HAL_I2C_Mem_Read_IT(
      device->config.hi2c, device->internal.pal_dev.I2cDevAddr,
      TOF200C_RESULT_REGISTER, I2C_MEMADD_SIZE_8BIT,
      device->internal.result_buffer, sizeof(device->internal.result_buffer));
  if (hal_status != HAL_OK) {
    tof200c_record_async_start_failure(device, hal_status);
  }
}

static VL53L0X_Error tof200c_apply_profile(tof200c_t *device)
{
  const tof200c_profile_config_t *profile =
      &k_profile_configs[device->config.profile];
  VL53L0X_DEV pal_dev = &device->internal.pal_dev;
  VL53L0X_Error status;

  status = VL53L0X_SetDeviceMode(pal_dev,
                                 VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  if (status == VL53L0X_ERROR_NONE) {
    status = VL53L0X_SetLimitCheckEnable(
        pal_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1U);
  }
  if (status == VL53L0X_ERROR_NONE) {
    status = VL53L0X_SetLimitCheckEnable(
        pal_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1U);
  }
  if (status == VL53L0X_ERROR_NONE) {
    status = VL53L0X_SetLimitCheckValue(
        pal_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        profile->sigma_limit_mm);
  }
  if (status == VL53L0X_ERROR_NONE) {
    status = VL53L0X_SetLimitCheckValue(
        pal_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        profile->signal_limit_mcps);
  }
  if (status == VL53L0X_ERROR_NONE) {
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
        pal_dev, profile->timing_budget_us);
  }
  if (status == VL53L0X_ERROR_NONE) {
    status = VL53L0X_SetVcselPulsePeriod(
        pal_dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
        profile->pre_range_vcsel_period);
  }
  if (status == VL53L0X_ERROR_NONE) {
    status = VL53L0X_SetVcselPulsePeriod(
        pal_dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
        profile->final_range_vcsel_period);
  }
  return status;
}

static tof200c_status_t tof200c_start_sensor(tof200c_t *device,
                                             bool reset_i2c,
                                             bool count_recovery)
{
  const tof200c_config_t *config = &device->config;
  VL53L0X_DEV pal_dev;
  VL53L0X_Error pal_status;
  HAL_StatusTypeDef hal_status;
  uint16_t model_id = 0U;
  uint8_t vhv_settings = 0U;
  uint8_t phase_cal = 0U;
  uint32_t ref_spad_count = 0U;
  uint8_t is_aperture_spads = 0U;

  tof200c_set_connection(
      device, count_recovery ? TOF200C_CONNECTION_RECOVERING
                             : TOF200C_CONNECTION_STARTING);
  device->internal.transfer_state = TOF200C_TRANSFER_IDLE;
  device->internal.data_pending = false;
  device->internal.clear_attempts = 0U;
  memset(&device->internal.pal_dev, 0, sizeof(device->internal.pal_dev));
  pal_dev = &device->internal.pal_dev;
  pal_dev->I2cDevAddr = (uint8_t)(TOF200C_DEFAULT_I2C_ADDRESS_7BIT << 1);
  pal_dev->comms_type = 1U;
  pal_dev->comms_speed_khz = 400U;
  pal_dev->hi2c = config->hi2c;

  HAL_GPIO_WritePin(config->xshut_port, config->xshut_pin, GPIO_PIN_RESET);
  HAL_Delay(TOF200C_XSHUT_DELAY_MS);

  if (reset_i2c) {
    hal_status = HAL_I2C_DeInit(config->hi2c);
    if (hal_status == HAL_OK) {
      hal_status = HAL_I2C_Init(config->hi2c);
    }
    if (hal_status != HAL_OK) {
      device->fault.last_hal_error = HAL_I2C_GetError(config->hi2c);
      device->fault.i2c_error_count++;
      tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_I2C_ERROR,
                           TOF200C_TRANSFER_IDLE);
      return TOF200C_STATUS_HAL_ERROR;
    }
  }

  HAL_GPIO_WritePin(config->xshut_port, config->xshut_pin, GPIO_PIN_SET);
  HAL_Delay(TOF200C_XSHUT_DELAY_MS);

  pal_status =
      VL53L0X_RdWord(pal_dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
  if ((pal_status != VL53L0X_ERROR_NONE) || (model_id != TOF200C_MODEL_ID)) {
    device->fault.last_pal_error = (int32_t)pal_status;
    device->fault.last_hal_error = HAL_I2C_GetError(config->hi2c);
    HAL_GPIO_WritePin(config->xshut_port, config->xshut_pin, GPIO_PIN_RESET);
    tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_SENSOR_INIT_ERROR,
                         TOF200C_TRANSFER_IDLE);
    return TOF200C_STATUS_DEVICE_NOT_FOUND;
  }

  if (config->i2c_address_7bit != TOF200C_DEFAULT_I2C_ADDRESS_7BIT) {
    pal_status =
        VL53L0X_SetDeviceAddress(pal_dev,
                                 (uint8_t)(config->i2c_address_7bit << 1));
    if (pal_status != VL53L0X_ERROR_NONE) {
      device->fault.last_pal_error = (int32_t)pal_status;
      device->fault.last_hal_error = HAL_I2C_GetError(config->hi2c);
      HAL_GPIO_WritePin(config->xshut_port, config->xshut_pin, GPIO_PIN_RESET);
      tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_SENSOR_INIT_ERROR,
                           TOF200C_TRANSFER_IDLE);
      return TOF200C_STATUS_PAL_ERROR;
    }
    pal_dev->I2cDevAddr = (uint8_t)(config->i2c_address_7bit << 1);
  }

  pal_status = VL53L0X_DataInit(pal_dev);
  if (pal_status == VL53L0X_ERROR_NONE) {
    pal_status = VL53L0X_StaticInit(pal_dev);
  }
  if (pal_status == VL53L0X_ERROR_NONE) {
    pal_status =
        VL53L0X_PerformRefCalibration(pal_dev, &vhv_settings, &phase_cal);
  }
  if (pal_status == VL53L0X_ERROR_NONE) {
    pal_status = VL53L0X_PerformRefSpadManagement(
        pal_dev, &ref_spad_count, &is_aperture_spads);
  }
  if (pal_status == VL53L0X_ERROR_NONE) {
    pal_status = tof200c_apply_profile(device);
  }
  if (pal_status == VL53L0X_ERROR_NONE) {
    pal_status = VL53L0X_SetGpioConfig(
        pal_dev, 0U, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
        VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
        VL53L0X_INTERRUPTPOLARITY_LOW);
  }
  if (pal_status == VL53L0X_ERROR_NONE) {
    pal_status = VL53L0X_ClearInterruptMask(pal_dev, 0U);
  }
  if (pal_status == VL53L0X_ERROR_NONE) {
    pal_status = VL53L0X_StartMeasurement(pal_dev);
  }
  if (pal_status != VL53L0X_ERROR_NONE) {
    device->fault.last_pal_error = (int32_t)pal_status;
    device->fault.last_hal_error = HAL_I2C_GetError(config->hi2c);
    HAL_GPIO_WritePin(config->xshut_port, config->xshut_pin, GPIO_PIN_RESET);
    tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_SENSOR_INIT_ERROR,
                         TOF200C_TRANSFER_IDLE);
    return TOF200C_STATUS_PAL_ERROR;
  }

  device->internal.measurement_started_ms = HAL_GetTick();
  device->internal.sample_count_at_start = device->feedback.sample_count;
  device->fault.offline_reason = TOF200C_OFFLINE_REASON_NONE;
  if (count_recovery) {
    device->fault.recovery_count++;
    device->internal.recovery_backoff_ms = 0U;
  }
  tof200c_set_connection(device, TOF200C_CONNECTION_ONLINE);
  if (HAL_GPIO_ReadPin(config->int_port, config->int_pin) == GPIO_PIN_RESET) {
    device->internal.data_pending = true;
  }
  return TOF200C_STATUS_OK;
}

tof200c_status_t tof200c_init(tof200c_t *device)
{
  tof200c_config_t config;

  if (device == NULL) {
    return TOF200C_STATUS_INVALID_ARGUMENT;
  }

  config = device->config;
  if (!tof200c_config_is_valid(&config)) {
    device->state.connection = TOF200C_CONNECTION_UNINITIALIZED;
    return TOF200C_STATUS_INVALID_ARGUMENT;
  }

  memset(device, 0, sizeof(*device));
  device->config = config;
  device->state.connection = TOF200C_CONNECTION_UNINITIALIZED;
  device->state.changed_at_ms = HAL_GetTick();
  return tof200c_start_sensor(device, false, false);
}

tof200c_status_t tof200c_get_latest(const tof200c_t *device,
                                    tof200c_feedback_t *feedback)
{
  uint32_t primask;

  if ((device == NULL) || (feedback == NULL)) {
    return TOF200C_STATUS_INVALID_ARGUMENT;
  }
  if (device->state.connection == TOF200C_CONNECTION_UNINITIALIZED) {
    return TOF200C_STATUS_NOT_INITIALIZED;
  }

  primask = tof200c_enter_critical();
  feedback->distance_mm = device->feedback.distance_mm;
  feedback->signal_rate_mcps_q16_16 =
      device->feedback.signal_rate_mcps_q16_16;
  feedback->ambient_rate_mcps_q16_16 =
      device->feedback.ambient_rate_mcps_q16_16;
  feedback->effective_spad_count_q8_8 =
      device->feedback.effective_spad_count_q8_8;
  feedback->device_range_status = device->feedback.device_range_status;
  feedback->range_status = device->feedback.range_status;
  feedback->range_valid = device->feedback.range_valid;
  feedback->timestamp_ms = device->feedback.timestamp_ms;
  feedback->sample_count = device->feedback.sample_count;
  tof200c_exit_critical(primask);

  if (device->state.connection != TOF200C_CONNECTION_ONLINE) {
    return TOF200C_STATUS_OFFLINE;
  }
  if (feedback->sample_count == 0U) {
    return TOF200C_STATUS_NO_DATA;
  }
  if ((device->config.stale_timeout_ms != 0U) &&
      ((uint32_t)(HAL_GetTick() - feedback->timestamp_ms) >
       device->config.stale_timeout_ms)) {
    return TOF200C_STATUS_STALE_DATA;
  }
  return TOF200C_STATUS_OK;
}

void tof200c_process(tof200c_t *device)
{
  uint32_t now_ms;
  uint32_t sample_reference_ms;
  tof200c_transfer_state_t failed_transfer;

  if (device == NULL) {
    return;
  }

  now_ms = HAL_GetTick();
  if (device->state.connection == TOF200C_CONNECTION_OFFLINE) {
    if (tof200c_time_reached(now_ms, device->internal.recovery_after_ms)) {
      (void)tof200c_start_sensor(device, true, true);
    }
    return;
  }
  if (device->state.connection != TOF200C_CONNECTION_ONLINE) {
    return;
  }

  if ((device->internal.transfer_state != TOF200C_TRANSFER_IDLE) &&
      tof200c_time_elapsed(now_ms, device->internal.transfer_started_ms,
                           TOF200C_TRANSFER_TIMEOUT_MS)) {
    failed_transfer = device->internal.transfer_state;
    device->fault.transfer_timeout_count++;
    tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_TRANSFER_TIMEOUT,
                         failed_transfer);
    return;
  }

  sample_reference_ms =
      (device->feedback.sample_count == device->internal.sample_count_at_start)
          ? device->internal.measurement_started_ms
          : device->feedback.timestamp_ms;
  if (tof200c_time_elapsed(now_ms, sample_reference_ms,
                           TOF200C_DATA_TIMEOUT_MS)) {
    tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_DATA_TIMEOUT,
                         device->internal.transfer_state);
    return;
  }

  if (HAL_GPIO_ReadPin(device->config.int_port, device->config.int_pin) ==
      GPIO_PIN_RESET) {
    device->internal.data_pending = true;
  }
  if (device->internal.data_pending &&
      (device->internal.transfer_state == TOF200C_TRANSFER_IDLE)) {
    tof200c_start_result_read(device);
  }
}

tof200c_status_t tof200c_recover(tof200c_t *device)
{
  if (device == NULL) {
    return TOF200C_STATUS_INVALID_ARGUMENT;
  }
  if (!tof200c_config_is_valid(&device->config)) {
    return TOF200C_STATUS_INVALID_ARGUMENT;
  }
  if (device->state.connection == TOF200C_CONNECTION_UNINITIALIZED) {
    return tof200c_init(device);
  }
  return tof200c_start_sensor(device, true, true);
}

tof200c_status_t tof200c_deinit(tof200c_t *device)
{
  VL53L0X_Error pal_status = VL53L0X_ERROR_NONE;
  tof200c_connection_t previous_connection;
  tof200c_transfer_state_t previous_transfer;
  uint32_t primask;

  if (device == NULL) {
    return TOF200C_STATUS_INVALID_ARGUMENT;
  }
  if (device->state.connection == TOF200C_CONNECTION_UNINITIALIZED) {
    return TOF200C_STATUS_NOT_INITIALIZED;
  }

  primask = tof200c_enter_critical();
  previous_connection = device->state.connection;
  previous_transfer = device->internal.transfer_state;
  device->state.connection = TOF200C_CONNECTION_UNINITIALIZED;
  device->state.changed_at_ms = HAL_GetTick();
  device->internal.transfer_state = TOF200C_TRANSFER_IDLE;
  device->internal.data_pending = false;
  tof200c_exit_critical(primask);

  if ((previous_connection == TOF200C_CONNECTION_ONLINE) &&
      (previous_transfer == TOF200C_TRANSFER_IDLE)) {
    pal_status = VL53L0X_StopMeasurement(&device->internal.pal_dev);
  }
  HAL_GPIO_WritePin(device->config.xshut_port, device->config.xshut_pin,
                    GPIO_PIN_RESET);
  if (pal_status != VL53L0X_ERROR_NONE) {
    return tof200c_set_pal_error(device, pal_status);
  }
  return TOF200C_STATUS_OK;
}

void tof200c_on_exti_callback(tof200c_t *device, uint16_t gpio_pin)
{
  if ((device == NULL) ||
      (device->state.connection != TOF200C_CONNECTION_ONLINE) ||
      (gpio_pin != device->config.int_pin)) {
    return;
  }

  device->internal.data_pending = true;
}

void tof200c_on_i2c_mem_rx_complete(tof200c_t *device,
                                    I2C_HandleTypeDef *hi2c)
{
  if ((device == NULL) || (hi2c != device->config.hi2c) ||
      (device->state.connection != TOF200C_CONNECTION_ONLINE)) {
    return;
  }

  if (device->internal.transfer_state == TOF200C_TRANSFER_READING_RESULT) {
    tof200c_publish_result(device);
    device->internal.clear_attempts = 0U;
    tof200c_begin_clear_assert(device);
    return;
  }

  if (device->internal.transfer_state != TOF200C_TRANSFER_CHECK_INTERRUPT) {
    return;
  }

  if ((device->internal.interrupt_status & 0x07U) == 0U) {
    device->internal.transfer_state = TOF200C_TRANSFER_IDLE;
    device->internal.data_pending = false;
    if (HAL_GPIO_ReadPin(device->config.int_port, device->config.int_pin) ==
        GPIO_PIN_RESET) {
      device->internal.data_pending = true;
    }
    return;
  }

  if (device->internal.clear_attempts < TOF200C_MAX_CLEAR_ATTEMPTS) {
    tof200c_begin_clear_assert(device);
    return;
  }

  device->fault.interrupt_clear_error_count++;
  tof200c_mark_offline(device,
                       TOF200C_OFFLINE_REASON_INTERRUPT_CLEAR_ERROR,
                       TOF200C_TRANSFER_CHECK_INTERRUPT);
}

void tof200c_on_i2c_mem_tx_complete(tof200c_t *device,
                                    I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef hal_status;

  if ((device == NULL) || (hi2c != device->config.hi2c) ||
      (device->state.connection != TOF200C_CONNECTION_ONLINE)) {
    return;
  }

  if (device->internal.transfer_state == TOF200C_TRANSFER_CLEAR_ASSERT) {
    device->internal.clear_value = 0U;
    device->internal.transfer_state = TOF200C_TRANSFER_CLEAR_RELEASE;
    device->internal.transfer_started_ms = HAL_GetTick();
    hal_status = HAL_I2C_Mem_Write_IT(
        device->config.hi2c, device->internal.pal_dev.I2cDevAddr,
        TOF200C_INTERRUPT_CLEAR_REGISTER, I2C_MEMADD_SIZE_8BIT,
        &device->internal.clear_value, 1U);
    if (hal_status != HAL_OK) {
      tof200c_record_async_start_failure(device, hal_status);
    }
    return;
  }

  if (device->internal.transfer_state == TOF200C_TRANSFER_CLEAR_RELEASE) {
    device->internal.transfer_state = TOF200C_TRANSFER_CHECK_INTERRUPT;
    device->internal.transfer_started_ms = HAL_GetTick();
    hal_status = HAL_I2C_Mem_Read_IT(
        device->config.hi2c, device->internal.pal_dev.I2cDevAddr,
        TOF200C_INTERRUPT_STATUS_REGISTER, I2C_MEMADD_SIZE_8BIT,
        &device->internal.interrupt_status, 1U);
    if (hal_status != HAL_OK) {
      tof200c_record_async_start_failure(device, hal_status);
    }
  }
}

void tof200c_on_i2c_error(tof200c_t *device, I2C_HandleTypeDef *hi2c)
{
  tof200c_transfer_state_t failed_transfer;

  if ((device == NULL) || (hi2c != device->config.hi2c) ||
      (device->state.connection != TOF200C_CONNECTION_ONLINE) ||
      (device->internal.transfer_state == TOF200C_TRANSFER_IDLE)) {
    return;
  }

  failed_transfer = device->internal.transfer_state;
  device->fault.last_hal_error = HAL_I2C_GetError(hi2c);
  device->fault.i2c_error_count++;
  tof200c_mark_offline(device, TOF200C_OFFLINE_REASON_I2C_ERROR,
                       failed_transfer);
}
