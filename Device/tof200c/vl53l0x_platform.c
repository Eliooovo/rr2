#include "vl53l0x_platform.h"

#define VL53L0X_PLATFORM_TIMEOUT_MS 100U

static VL53L0X_Error vl53l0x_hal_status(HAL_StatusTypeDef status)
{
  return (status == HAL_OK) ? VL53L0X_ERROR_NONE
                            : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index,
                                  uint8_t *data, uint32_t count)
{
  if ((dev == NULL) || (dev->hi2c == NULL) || (data == NULL) ||
      (count == 0U) || (count > VL53L0X_MAX_I2C_XFER_SIZE)) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }

  return vl53l0x_hal_status(HAL_I2C_Mem_Write(
      dev->hi2c, dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, data,
      (uint16_t)count, VL53L0X_PLATFORM_TIMEOUT_MS));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index,
                                 uint8_t *data, uint32_t count)
{
  if ((dev == NULL) || (dev->hi2c == NULL) || (data == NULL) ||
      (count == 0U) || (count > VL53L0X_MAX_I2C_XFER_SIZE)) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }

  return vl53l0x_hal_status(HAL_I2C_Mem_Read(
      dev->hi2c, dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, data,
      (uint16_t)count, VL53L0X_PLATFORM_TIMEOUT_MS));
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data)
{
  return VL53L0X_WriteMulti(dev, index, &data, 1U);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data)
{
  uint8_t bytes[2] = {(uint8_t)(data >> 8), (uint8_t)data};
  return VL53L0X_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data)
{
  uint8_t bytes[4] = {
      (uint8_t)(data >> 24), (uint8_t)(data >> 16),
      (uint8_t)(data >> 8), (uint8_t)data};
  return VL53L0X_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index,
                                  uint8_t and_data, uint8_t or_data)
{
  uint8_t value;
  VL53L0X_Error status = VL53L0X_RdByte(dev, index, &value);

  if (status == VL53L0X_ERROR_NONE) {
    value = (uint8_t)((value & and_data) | or_data);
    status = VL53L0X_WrByte(dev, index, value);
  }
  return status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data)
{
  return VL53L0X_ReadMulti(dev, index, data, 1U);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data)
{
  uint8_t bytes[2];
  VL53L0X_Error status;

  if (data == NULL) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  status = VL53L0X_ReadMulti(dev, index, bytes, sizeof(bytes));
  if (status == VL53L0X_ERROR_NONE) {
    *data = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
  }
  return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data)
{
  uint8_t bytes[4];
  VL53L0X_Error status;

  if (data == NULL) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  status = VL53L0X_ReadMulti(dev, index, bytes, sizeof(bytes));
  if (status == VL53L0X_ERROR_NONE) {
    *data = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
            ((uint32_t)bytes[2] << 8) | bytes[3];
  }
  return status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev)
{
  (void)dev;
  HAL_Delay(1U);
  return VL53L0X_ERROR_NONE;
}
