/*********************************************************************
* SEGGER RTT project configuration for rr2.
**********************************************************************
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS    (1)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS  (1)
#define BUFFER_SIZE_UP                    (1024)
#define BUFFER_SIZE_DOWN                  (16)
#define SEGGER_RTT_PRINTF_BUFFER_SIZE     (64u)

#define SEGGER_RTT_MODE_DEFAULT           SEGGER_RTT_MODE_NO_BLOCK_SKIP

/*
 * The RTT control block and buffers are linked into STM32H723 DTCM.
 * DTCM is not cached, so no cache-line alias is required.  Section placement
 * must remain enabled; setting CPU_CACHE_LINE_SIZE to a non-zero value makes
 * the official implementation use its cache-alignment path instead.
 */
#define SEGGER_RTT_CPU_CACHE_LINE_SIZE    (0)
#define SEGGER_RTT_UNCACHED_OFF           (0)
#define SEGGER_RTT_SECTION                ".segger_rtt"
#define SEGGER_RTT_BUFFER_SECTION         ".segger_rtt"
#define SEGGER_RTT_ALIGNMENT              (32)
#define SEGGER_RTT_BUFFER_ALIGNMENT       (32)

#endif
