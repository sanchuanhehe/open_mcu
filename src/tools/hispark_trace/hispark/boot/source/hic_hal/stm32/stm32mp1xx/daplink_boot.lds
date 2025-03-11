/* create for stmp32mp157, A7_CODE run in sysram */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH("arm")
ENTRY(_start)

TEXT_START = 0x2FFC2500;
RAM_START  = 0x10000000;

MEMORY
{
	A7_CODE(RX) : ORIGIN = TEXT_START, LENGTH = 16K
	A7_SRAM(RW) : ORIGIN = RAM_START, LENGTH = 4K
	STACK(RW)   : ORIGIN = RAM_START + 4K, LENGTH = 16K
}

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
}
