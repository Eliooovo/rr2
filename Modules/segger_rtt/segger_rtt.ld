/* Keep SEGGER RTT outside the Cortex-M7 D-Cache in STM32H723 DTCM. */
SECTIONS
{
  .segger_rtt (NOLOAD) :
  {
    . = ALIGN(32);
    KEEP(*(.segger_rtt))
    KEEP(*(.segger_rtt.*))
    . = ALIGN(32);
  } >DTCMRAM
}
