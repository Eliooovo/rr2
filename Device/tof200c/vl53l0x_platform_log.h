#ifndef VL53L0X_PLATFORM_LOG_H_
#define VL53L0X_PLATFORM_LOG_H_

#include <string.h>

enum {
  TRACE_MODULE_NONE = 0x0,
  TRACE_MODULE_API = 0x1,
  TRACE_MODULE_PLATFORM = 0x2,
  TRACE_MODULE_ALL = 0x7fffffff
};

#define VL53L0X_ErrLog(...) ((void)0)
#define _LOG_FUNCTION_START(module, fmt, ...) ((void)0)
#define _LOG_FUNCTION_END(module, status, ...) ((void)0)
#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...) ((void)0)
#define VL53L0X_COPYSTRING(destination, source) strcpy((destination), (source))

#endif
