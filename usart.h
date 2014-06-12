#ifndef _USART_DRV_H_
#define _USART_DRV_H_

#include <stdint.h>

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
	uint32_t dma_tc_flag;

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
	int tx_started;
	ring_buffer_t rx;

	USART_TypeDef* usart_regs;
	DMA_Channel_TypeDef* dma_channel_regs;

	DMA_InitTypeDef DMA_InitStructure;
	uint32_t dma_tc_flag;

} usart_t;

void usart_init (usart_t* usart, const usart_config_t* cfg,
		void* tx_buffer, uint32_t tx_buffer_size,
		void* rx_buffer, uint32_t rx_buffer_size);

void usart_enable (usart_t* usart);
void usart_disable (usart_t* usart);

int usart_send (usart_t* usart, const void *ptr, uint32_t size);

int usart_recv (usart_t* usart, void *ptr, uint32_t size);

void usart_isr (usart_t* usart);
void usart_handle_tx_dma_irq (usart_t* usart);

#endif
