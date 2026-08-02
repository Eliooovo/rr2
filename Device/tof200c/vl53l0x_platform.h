#ifndef VL53L0X_PLATFORM_H_
#define VL53L0X_PLATFORM_H_

#include "main.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform_log.h"

typedef struct {
  VL53L0X_DevData_t Data;
  uint8_t I2cDevAddr;
  uint8_t comms_type;
  uint16_t comms_speed_khz;
  I2C_HandleTypeDef *hi2c;
} VL53L0X_Dev_t;

typedef VL53L0X_Dev_t *VL53L0X_DEV;

#define VL53L0X_MAX_I2C_XFER_SIZE 64U
#define PALDevDataGet(Dev, field) ((Dev)->Data.field)
#define PALDevDataSet(Dev, field, data) ((Dev)->Data.field = (data))

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index,
                                  uint8_t *data, uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index,
                                 uint8_t *data, uint32_t count);
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data);
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data);
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data);
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index,
                                  uint8_t and_data, uint8_t or_data);
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data);
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data);
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data);
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev);

#endif
