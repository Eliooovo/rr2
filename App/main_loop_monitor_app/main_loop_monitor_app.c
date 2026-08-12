#include "main_loop_monitor_app.h"

#include <stdbool.h>
#include <stdint.h>

#include "SEGGER_RTT.h"
#include "stm32h7xx.h"

#define MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ 1000U
#define MAIN_LOOP_MONITOR_APP_RTT_BUFFER_INDEX       0U
#define MAIN_LOOP_MONITOR_APP_DWT_UNLOCK_KEY         0xC5ACCE55UL
#define MAIN_LOOP_MONITOR_APP_US_PER_SECOND          1000000ULL
#define MAIN_LOOP_MONITOR_APP_Q32_ONE                (1ULL << 32U)

typedef struct {
    uint32_t period_limit_cycles;
    uint32_t period_limit_us;
    uint64_t cycles_to_us_q32;
    uint32_t cycle_started_at;
    uint32_t last_cycle_us;
    uint32_t max_cycle_us;
    uint32_t timeout_count;
    bool timed_out;
    bool initialized;
} main_loop_monitor_app_state_t;

static main_loop_monitor_app_state_t s_state;

static uint32_t MainLoopMonitorApp_CyclesToUs(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * s_state.cycles_to_us_q32) >>
                      32U);
}

void MainLoopMonitorApp_Init(void)
{
    uint64_t period_limit_cycles;

    s_state.initialized = false;
    s_state.last_cycle_us = 0U;
    s_state.max_cycle_us = 0U;
    s_state.timeout_count = 0U;
    s_state.timed_out = false;

    SEGGER_RTT_Init();
    SEGGER_RTT_WriteString(
        MAIN_LOOP_MONITOR_APP_RTT_BUFFER_INDEX,
        "[rr2] RTT ready\r\n");

    if (MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ == 0U ||
        MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ > SystemCoreClock) {
        SEGGER_RTT_WriteString(
            MAIN_LOOP_MONITOR_APP_RTT_BUFFER_INDEX,
            "[ERROR] main loop monitor invalid frequency\r\n");
        return;
    }

    period_limit_cycles =
        ((uint64_t)SystemCoreClock +
         (uint64_t)MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ - 1ULL) /
        (uint64_t)MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ;
    s_state.period_limit_cycles = (uint32_t)period_limit_cycles;
    s_state.period_limit_us =
        (uint32_t)((MAIN_LOOP_MONITOR_APP_US_PER_SECOND +
                    (uint64_t)MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ -
                    1ULL) /
                   (uint64_t)MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ);
    s_state.cycles_to_us_q32 =
        ((MAIN_LOOP_MONITOR_APP_US_PER_SECOND *
          MAIN_LOOP_MONITOR_APP_Q32_ONE) +
         (uint64_t)SystemCoreClock - 1ULL) /
        (uint64_t)SystemCoreClock;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = MAIN_LOOP_MONITOR_APP_DWT_UNLOCK_KEY;
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U) {
        SEGGER_RTT_WriteString(
            MAIN_LOOP_MONITOR_APP_RTT_BUFFER_INDEX,
            "[ERROR] DWT cycle counter unavailable\r\n");
        return;
    }

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        SEGGER_RTT_WriteString(
            MAIN_LOOP_MONITOR_APP_RTT_BUFFER_INDEX,
            "[ERROR] DWT cycle counter enable failed\r\n");
        return;
    }

    s_state.cycle_started_at = DWT->CYCCNT;
    s_state.initialized = true;
}

void MainLoopMonitorApp_RunPeriodic(void)
{
    uint32_t elapsed_cycles;

    if (!s_state.initialized) {
        return;
    }

    elapsed_cycles = DWT->CYCCNT - s_state.cycle_started_at;
    s_state.last_cycle_us = MainLoopMonitorApp_CyclesToUs(elapsed_cycles);
    s_state.timed_out = elapsed_cycles > s_state.period_limit_cycles;

    if (s_state.last_cycle_us > s_state.max_cycle_us) {
        s_state.max_cycle_us = s_state.last_cycle_us;
    }

    if (s_state.timed_out) {
        s_state.timeout_count++;
        SEGGER_RTT_printf(
            MAIN_LOOP_MONITOR_APP_RTT_BUFFER_INDEX,
            "[WARN] main loop timeout: elapsed=%u us, limit=%u us (%u Hz), count=%u\r\n",
            s_state.last_cycle_us,
            s_state.period_limit_us,
            MAIN_LOOP_MONITOR_APP_EXPECTED_FREQUENCY_HZ,
            s_state.timeout_count);
    }

    s_state.cycle_started_at = DWT->CYCCNT;
}
