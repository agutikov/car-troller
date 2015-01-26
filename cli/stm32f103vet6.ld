
ENTRY(reset_isr)

MEMORY
{
    FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM (rxw)     : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
	.text :
	{
		. = ALIGN(4);
		_text = .;
		KEEP(*(.isr_vector))
		*(.text)
		*(.text*)
		. = ALIGN(4);
		_etext = .;
		_rodata = .;
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
		_erodata = .;
	} > FLASH

	_data_size = 4K;
	_bss_size = 8K;
	_stack_size = 4K;
	_heap_size = LENGTH(SRAM) - _data_size - _bss_size - _stack_size;

	.data : AT (ADDR(.text) + SIZEOF(.text))
	{
		. = ALIGN(4);
		_data = .;
		*(.data)
		*(.data*)
		. = _data + _data_size;
		. = ALIGN(4);
		_edata = .;
	} > SRAM

	_data_at_flash = ADDR(.text) + SIZEOF(.text);
	_edata_at_flash = _data_at_flash + SIZEOF(.data);

	.bss :
	{
		. = ALIGN(4);
		_bss = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = _bss + _bss_size;
		. = ALIGN(4);
		_ebss = .;
	} > SRAM

	.heap :
	{
		. = ALIGN(4);
		_heap = .;
		. = . + _heap_size;
		. = ALIGN(4);
		_eheap = .;
	} > SRAM

	/*
	_estack = ORIGIN(SRAM) + LENGTH(SRAM);
	_stack = _estack - _stack_size;
	*/
	.stack :
	{
		. = ALIGN(4);
		_stack = .;
		. = . + _stack_size;
		. = ALIGN(4);
		_estack = .;
	} > SRAM
}


