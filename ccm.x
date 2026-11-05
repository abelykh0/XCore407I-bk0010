/* C linker-script equivalent: place uninitialized CCM storage separately,
   after cortex-m-rt has defined its SRAM .data/.bss/stack boundaries. */
SECTIONS
{
  .ccmram (NOLOAD) : ALIGN(4)
  {
    *(.ccmram .ccmram.*);
    . = ALIGN(4);
  } > CCM
}
