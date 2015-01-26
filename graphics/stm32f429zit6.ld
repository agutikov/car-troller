
ENTRY(reset_isr)

MEMORY
{
    FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 2M

    CCM (rxw)     : ORIGIN = 0x10000000, LENGTH = 64K
/*
Continuous SRAM 112k, SRAM 16k and SRAM 64k
*/
    SRAM (rxw)    : ORIGIN = 0x20000000, LENGTH = 192K
/*
SDRAM connected to FMC Bank 6 (SDRAM Bank 2)
*/
    SDRAM (rxw)   : ORIGIN = 0xD0000000, LENGTH = 8M
}

/*
TODO: stack > CCM, data + bss > SRAM, heap > SDRAM
*/

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

	.heap :
	{
		. = ALIGN(4);
		_heap = .;
		. = . + LENGTH(SDRAM);
		. = ALIGN(4);
		_eheap = .;
	} > SDRAM

	.stack :
	{
		. = ALIGN(4);
		_stack = .;
		. = . + LENGTH(CCM);
		. = ALIGN(4);
		_estack = .;
	} > CCM
}


