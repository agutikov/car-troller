

#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include <stdint.h>
#include <sys/types.h>

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

extern void main(void);
extern void panic (int delay);

static uint8_t *_heap_end = (uint8_t*)&_heap;

caddr_t _sbrk_r (int incr)
{
	if (_heap_end + incr >= (uint8_t*)&_heap_end) {
		// TODO: global terminal for printf
		panic(2000);
	}

	_heap_end += incr;

	return (caddr_t) _heap_end;
}



extern void usart1_tx_dma_isr(void);
extern void usart1_isr (void);

/*
 * Only Reset ISR may be ((noreturn)) and ((naked)).
 * If execution should proceed after IRQ handling -
 * ISR _MUST_ return.
 */
void reset_isr(void) __attribute__ ((noreturn)) __attribute__ ((naked));

void reset_isr(void)
{
	register uint32_t *src = &_erodata;
	register uint32_t *dest = &_data;
	register uint32_t *end = &_edata;

	/* Copy the data segment initializers from flash to SRAM. */
	while (dest < end) {
		register uint32_t tmp = *src;
		*dest = tmp;
		dest++;
		src++;
	}

	/* Zero fill the bss segment. */
	dest = &_bss;
	end = &_ebss;
	while (dest < end) {
		register uint32_t tmp = 0;
		*dest = tmp;
		dest++;
	}

	SystemInit();

	/* Call the application's entry point. */
	main();

	while(1);
}

typedef struct frame {
	uint32_t R0;
	uint32_t R1;
	uint32_t R2;
	uint32_t R3;
	uint32_t R12;
	uint32_t return_address;
	uint32_t PSR;
	uint32_t LR;
} frame_t;

void hard_fault_isr(void)
{
//	uint32_t anchor = 0x00ABBA00;
//	frame_t* frame = (frame_t*) (((uint8_t*)&anchor) - sizeof(frame_t));
//	while (1);

	panic(300);
}

void nmi_isr(void)
{
	panic(1000);
}

#if 0
void shared_isr(void) __attribute__ ((noreturn)) __attribute__ ((naked));

extern driver_isr_t driver_isr_vector[];

/*
 * Shared isr detects active IRQ and call related user's isr.
 * User can change isr - array of user's isrs locates in sram.
 */
void shared_isr(void)
{
	uint8_t irq_id = (SCB->ICSR & 0xFF) - 15;
	driver_isr_vector[irq_id](irq_id);
}
#endif


typedef void (*isr_t) (void);

__attribute__ ((section(".isr_vector")))
isr_t __isr_vector[] =
{
	(isr_t) (&_estack), // The initial stack pointer

	reset_isr,		/*!#0 cortex-m3 reset interrupt begin code of this */
	nmi_isr,		/*!#1 cortex-m3 non maskable interrupt */
	hard_fault_isr,		/*!#2 cortex-m3 hardware fault interrupt */
	0,			/*!#3 cortex-m3 memory management interrupt */
	0,			/*!#4 cortex-m3 bus fault interrupt */
	0,			/*!#5 cortex-m3 usage fault interrupt */
	0,			/*!#6 reserved */
	0,			/*!#7 reserved */
	0,			/*!#8 reserved */
	0,			/*!#9 reserved */
	0,			/*!#10 cortex-m3 system service interrupt */
	0,			/*!#11 cortex-m3 debug monitor interrupt */
	0,			/*!#12 reserved */
	0,			/*!#13 cortex-m3 penable request for system service interrupt */
	0,			/*!#14 cortex-m3 system tick timer interrupt */
	0,			/*!%0 window watchdog interrupt */
	0,			/*!%1 PVD through EXTI line detection interrupt */
	0,			/*!%2 tamper adn timestamp through EXTI interrupt */
	0,			/*!%3 RTC wakeup through EXTI interrupt */
	0,			/*!%4 flash global interrupt */
	0,			/*!%5 RCC global interrupt */
	0,			/*!%6 EXTI line0 interrupt */
	0,			/*!%7 EXTI line1 interrupt */
	0,			/*!%8 EXTI line2 interrupt */
	0,			/*!%9 EXTI line3 interrupt */
	0,			/*!%10 EXTI line4 interrupt */
	usart1_tx_dma_isr,			/*!%11 DMA1 channel 1 global interrupt */
	0,			/*!%12 DMA1 channel 2 global interrupt */
	0,			/*!%13 DMA1 channel 3 global interrupt */
	0,			/*!%14 DMA1 channel 4 global interrupt */
	0,			/*!%15 DMA1 channel 5 global interrupt */
	0,			/*!%16 DMA1 channel 6 global interrupt */
	0,			/*!%17 DMA1 channel 7 global interrupt */
	0,			/*!%18 ADC1 global interrupt */
	0,			/*!%19 reserved */
	0,			/*!%20 reserved */
	0,			/*!%21 reserved */
	0,			/*!%22 reserved */
	0,			/*!%23 EXTI line[9:5] interrupts */
	0,			/*!%24 TIM1 break and TIM15 global interrupt */
	0,			/*!%25 TIM1 update and TIM16 global interrupt */
	0,			/*!%26 TIM1 trigger and commutation and TIM17 global interrupt */
	0,			/*!%27 TIM1 capture compare interrupt */
	0,			/*!%28 TIM2 global interrupt */
	0,			/*!%29 TIM3 global interrupt */
	0,			/*!%30 TIM4 global interrupt */
	0,			/*!%31 I2C1 event interrupt */
	0,			/*!%32 I2C1 error interrupt */
	0,			/*!%33 I2C2 event interrupt */
	0,			/*!%34 I2C2 error interrupt */
	0,			/*!%35 SPI1 global interrupt */
	0,			/*!%36 SPI2 global interrupt */
	usart1_isr,			/*!%37 USART1 global interrupt */
	0,			/*!%38 USART2 global interrupt */
	0,			/*!%39 USART3 global interrupt */
	0,			/*!%40 EXTI line[15:10] interrupts */
	0,			/*!%41 RTC alarm through EXTI line interrupt */
	0,			/*!%42 HDMI-CEC interrupt */
	0,			/*!%43 reserved */
	0,			/*!%44 reserved */
	0,			/*!%45 reserved */
	0,			/*!%46 reserved */
	0,			/*!%47 reserved */
	0,			/*!%48 reserved */
	0,			/*!%49 reserved */
	0,			/*!%50 reserved */
	0,			/*!%51 reserved */
	0,			/*!%52 reserved */
	0,			/*!%53 reserved */
	0,			/*!%54 TIM6 and DAC underrun interrupt */
	0			/*!%55 TIM7 interrupt */
};

