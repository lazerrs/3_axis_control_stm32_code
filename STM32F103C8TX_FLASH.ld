/* STM32F103C8T6 64K FLASH, 20K RAM */

ENTRY(Reset_Handler)

/* Define memory regions */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Define stack size */
_Min_Heap_Size = 0x200;      /* 512 bytes */
_Min_Stack_Size = 0x400;     /* 1KB stack */

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Vector table */
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .ARM.extab :
  {
    *(.ARM.extab*)
  } >FLASH

  .ARM.exidx :
  {
    *(.ARM.exidx*)
  } >FLASH

    .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT >FLASH

  /* Define source of initialized data in FLASH */
  _sidata = LOADADDR(.data);


  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >RAM

  /* Define stack at top of RAM */
  _estack = ORIGIN(RAM) + LENGTH(RAM);

  /* Heap and stack region */
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(4);
  } >RAM
}
