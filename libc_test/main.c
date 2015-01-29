
#include <stdint.h>
#include <string.h>
#include <stdio.h>


void _init (void)
{
}


extern uint32_t _text;
extern uint32_t _etext;

extern uint32_t _rodata;
extern uint32_t _erodata;

extern uint32_t _data_at_flash;
extern uint32_t _edata_at_flash;

extern uint32_t _data;
extern uint32_t _edata;

extern uint32_t _bss;
extern uint32_t _ebss;

extern uint32_t _heap;
extern uint32_t _eheap;

extern uint32_t _stack;
extern uint32_t _estack;



typedef struct section {
	uint32_t *start;
	uint32_t *end;
} section_t;

const section_t flash_sections[] = {
	{&_text, &_etext},
	{&_rodata, &_erodata},
	{&_data_at_flash, &_edata_at_flash}
};

const section_t sram_sections[] = {
	{&_data, &_edata},
	{&_bss, &_ebss},
	{&_heap, &_eheap},
	{&_stack, &_estack}
};

void wait (uint32_t xz)
{
	for(uint32_t i = 0; i < xz*180000; i++)
		asm volatile ("nop");
}


static uint8_t *_heap_end = (uint8_t*) &_ebss;

caddr_t _sbrk_r (int incr)
{
	uint8_t* prev_heap_end = _heap_end;

	if (_heap_end + incr >= (uint8_t*)&_eheap) {
//		panic(10);
	}

	_heap_end += incr;

	return (caddr_t) prev_heap_end;
}





void main( void )
{

	int i = 0;
	int b = 345678;
	
	char buffer[128] = {0};
	
	snprintf(buffer, sizeof(buffer), "i=%d, b=%d", i, b);
	
	
	
	
	while(1);
	
}





