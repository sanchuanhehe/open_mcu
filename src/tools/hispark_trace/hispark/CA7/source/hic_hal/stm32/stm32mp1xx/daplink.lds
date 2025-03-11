/* create for stmp32mp157, A7_CODE run in sysram */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH("arm")
ENTRY(_start)

SYSRAM_ADDR = 0x2FFC0000;
SRAM2_ADDR  = 0x10020000;
SRAM4_ADDR  = 0x10050000;
START_ADDR  = SYSRAM_ADDR + 0xA500;

MEMORY
{
    A7_CODE(RX)     : ORIGIN = START_ADDR, LENGTH = 158K
    A7_SRAM(RW)     : ORIGIN = START_ADDR + 158K, LENGTH = 56K	
    MSGQ_SHM(RW)    : ORIGIN = SRAM4_ADDR, LENGTH = 20K
    STACK(RW)       : ORIGIN = SRAM4_ADDR + 20K, LENGTH = 12K
    CFGRAM(RW)      : ORIGIN = SRAM4_ADDR + 32K, LENGTH = 1K
    CFGROM(RW)      : ORIGIN = SRAM4_ADDR + 33K, LENGTH = 1K
    HEAP(RW)        : ORIGIN = SRAM4_ADDR + 34K, LENGTH = 4K	
	
    VARQ_SHM(RW)    : ORIGIN = SRAM2_ADDR, LENGTH = 120K
}

__HEAP_START__  = ORIGIN(HEAP);
__HEAP_END__    = ORIGIN(HEAP)+LENGTH(HEAP);
SECTIONS
{
    . = ALIGN(4);
    .text.entry :
    {
        KEEP(*(.text.entry))
    } > A7_CODE

	.text :
	{
		*(.text*)
    *(.ram.text*)
	} > A7_CODE

	.rodata :
    {
        . = ALIGN(4);
        *(SORT_BY_ALIGNMENT(.rodata*))
        *(.got*)
    } > A7_CODE

	.data :
    {
        . = ALIGN(4);
        __data_load = LOADADDR(.data);
        __data_start = .;
        *(SORT_BY_ALIGNMENT(.data*))
		. = ALIGN(4);
        __data_end = .;
    } > A7_SRAM AT > A7_CODE

	.bss :
	{
		. = ALIGN(4);
		__bss_start = .;
		*(.bss*)
		. = ALIGN(4);
		__bss_end = .;
	} > A7_SRAM

    .stack (NOLOAD) :
    {
        . = ALIGN(4);
        __SYSTEM_STACK_BEGIN__ = ORIGIN(STACK);
        KEEP(*(.stacks))
        __SYSTEM_STACK_END__ = ORIGIN(STACK) + LENGTH(STACK);
    } > STACK
    STACK_TOP = __SYSTEM_STACK_END__;

	.cfgram :
	{
		. = ALIGN(4);
        __cfgram_load = LOADADDR(.cfgram);
		__cfgram_start = .;
		*(cfgram)
		. = ALIGN(4);
		__cfgram_end = .;
	} > CFGRAM AT > A7_CODE

	.cfgrom :
	{
		. = ALIGN(4);
        __cfgrom_load = LOADADDR(.cfgrom);
		__cfgrom_start = .;
		*(cfgrom)
		. = ALIGN(4);
		__cfgrom_end = .;
	} > CFGROM AT > A7_CODE

  .msgq :
  {
    . = ALIGN(4);
	__msgq_start = .;
    *(.msgq_section)
    . = ALIGN(4);
	__msgq_end = .;
  } > MSGQ_SHM
  g_msgQStart = __msgq_start;

  .varq :
  {
    . = ALIGN(4);
	__varq_start = .;
    *(.varq_section)
    . = ALIGN(4);
	__varq_end = .;
  } > VARQ_SHM
  g_vargStart = __varq_start;

    .interp : { *(.interp*) }
}
