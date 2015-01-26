#ifndef _USART_DRV_H_
#define _USART_DRV_H_

#include <stdint.h>
#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "misc.h"

#include "ring_buffer.h"

/*
 * USART driver configuration
 *
 * chip:
 * - rcc: dma clocks and usart clocks
 * - usart interrupts
 * - usart regs
 * - dma interrupts
 * - dma regs
 *
 * board:
 * - gpio lines
 */
typedef struct usart_config {

	// RCC: gpio port clock, usart clock, dma clock,
	// RCC alternative: AHB, APB1, APB2 clocks
	uint32_t AHB_clocks;
	uint32_t APB1_clocks;
	uint32_t APB2_clocks;

	GPIO_TypeDef* usart_gpio_port;
	uint32_t usart_gpio_tx_line;
	uint32_t usart_gpio_rx_line;

	USART_TypeDef* usart_regs;
	IRQn_Type usart_irqn;

	DMA_Channel_TypeDef* dma_channel_regs;
	IRQn_Type dma_irqn;

	// channel 1 (in datasheet) - > idx 1
	uint32_t dma_channel_idx;

} usart_config_t;

/*
 * Runtime USART device data:
 *
 * buffers
 * usart registers
 * tx dma channal
 *
 */
typedef struct usart_ {
	ring_buffer_t tx;
	uint32_t last_tx_dma;
	ring_buffer_t rx;

	USART_TypeDef* usart_regs;
	DMA_Channel_TypeDef* dma_channel_regs;

	DMA_InitTypeDef DMA_InitStructure;

	// see usart_config_t
	uint32_t dma_channel_idx;

	int term_newline_count;

} usart_t;

void usart_init (usart_t* usart, const usart_config_t* cfg,
		void* tx_buffer, uint32_t tx_buffer_size,
		void* rx_buffer, uint32_t rx_buffer_size);

void usart_enable (usart_t* usart);
void usart_disable (usart_t* usart);

int usart_send (usart_t* usart, const void *ptr, uint32_t size);
int usart_recv (usart_t* usart, void *ptr, uint32_t size);

#define CR ((uint8_t)13)
#define LF ((uint8_t)10)

int term_send (usart_t* term, const void* ptr, uint32_t size);
static inline void term_putstr (usart_t* term, const void* str)
{
	term_send(term, str, strlen(str));
}
int term_recv (usart_t* term, void* ptr, uint32_t size);
int term_getline (usart_t* term, void* ptr, uint32_t size);

#define PRINTF_BUFFER_SIZE 512

void term_printf (usart_t* term, const char* format, ...);

void usart_isr (usart_t* usart);
void usart_handle_tx_dma_irq (usart_t* usart);



#endif
