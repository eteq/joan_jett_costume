MEMORY
{
  /* Leave 8k for the default bootloader on the Feather M0 */
  FLASH (rx) : ORIGIN = 0x00000000 + 8K, LENGTH = 256K - 8K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 31K
  PANDUMP: ORIGIN = 0x20007C00, LENGTH = 1K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

_panic_dump_start = ORIGIN(PANDUMP);
_panic_dump_end   = ORIGIN(PANDUMP) + LENGTH(PANDUMP);
