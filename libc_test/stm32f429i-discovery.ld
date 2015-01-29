
ENTRY(Reset_Handler)

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
    .isr_vector :
    {
	. = ALIGN(4);
        KEEP(*(.isr_vector))
	. = ALIGN(4);
    } >FLASH

    /* for some STRx devices, the beginning of the startup code is stored in the .flashtext section, which goes to FLASH */
    .flashtext :
    {
	. = ALIGN(4);
        *(.flashtext)            /* Startup code */
	. = ALIGN(4);
    } > FLASH

	.text :
	{
		. = ALIGN(4);
		_text = .;
		*(.text)
		*(.text*)
		. = ALIGN(4);
		_etext = .;
		_rodata = .;
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
		_erodata = .;
		*(.glue_7)
        *(.glue_7t)
        *(.vfp11_veneer)
        *(.v4_bx)
        *(.ARM.extab* .gnu.linkonce.armextab.*)
	} > FLASH

	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > FLASH

	.data : AT (ADDR(.ARM.exidx) + SIZEOF(.ARM.exidx))
	{
		. = ALIGN(4);
		_data = .;
		_sdata = .;
		*(.data)
		*(.data*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
		_edata = .;
	} > SRAM

	_data_at_flash = ADDR(.text) + SIZEOF(.text);
	_sidata = ADDR(.text) + SIZEOF(.text);
	_edata_at_flash = _data_at_flash + SIZEOF(.data);

	.bss :
	{
		. = ALIGN(4);
		_bss = .;
		_sbss = .;
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


