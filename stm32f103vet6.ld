
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

	.data : AT (ADDR(.text) + SIZEOF(.text))
	{
		. = ALIGN(4);
		_data = .;
		*(.data)
		*(.data*)
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
		. = ALIGN(4);
		_ebss = .;
	} > SRAM
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);
_stack = _estack - 4K;
