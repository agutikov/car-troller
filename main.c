
/*

	A9 - usart1 tx
	A10 - usart1 rx








 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32f10x_rcc.h"

#include "usart.h"


#define USART_NUMBER 3
#define USART_BUFF_SIZE	512

uint8_t usart_buffers[USART_NUMBER][2][USART_BUFF_SIZE];
usart_t usart_devices[USART_NUMBER];
usart_config_t usart_configs[USART_NUMBER] = {
	{
//		.AHB_clocks = RCC_AHBPeriph_DMA1,
		.APB1_clocks = 0,
//		.APB2_clocks = RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,
		.APB2_clocks = RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,

		.usart_gpio_port = GPIOA,
		.usart_gpio_tx_line = GPIO_Pin_9,
		.usart_gpio_rx_line = GPIO_Pin_10,

		.usart_regs = USART1,
		.usart_irqn = USART1_IRQn,

//		.dma_channel_regs = DMA1_Channel1,
//		.dma_irqn = DMA1_Channel1_IRQn,
//		.dma_channel_idx = 1
	},
	{

	},
	{

	}
};

void usart1_isr (void)
{
	usart_isr(&usart_devices[0]);
}
void usart1_tx_dma_isr(void)
{
	usart_handle_tx_dma_irq(&usart_devices[0]);
}

#define LED_NUMBER  4

uint32_t leds[LED_NUMBER][2] = {
		{(uint32_t)GPIOC, GPIO_Pin_6},
		{(uint32_t)GPIOC, GPIO_Pin_7},
		{(uint32_t)GPIOD, GPIO_Pin_13},
		{(uint32_t)GPIOD, GPIO_Pin_6}
};
void leds_init (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
void led_on (int led)
{
	if (led < LED_NUMBER)
		GPIO_WriteBit((GPIO_TypeDef *)leds[led][0], leds[led][1], 1);
}
void led_off (int led)
{
	if (led < LED_NUMBER)
		GPIO_WriteBit((GPIO_TypeDef *)leds[led][0], leds[led][1], 0);
}
void led_num (int n)
{
	for (int i = 0; i < LED_NUMBER; i++) {
		if (n & (1 << i))
			GPIO_WriteBit((GPIO_TypeDef *)leds[i][0], leds[i][1], 1);
		else
			GPIO_WriteBit((GPIO_TypeDef *)leds[i][0], leds[i][1], 0);
	}
}

void wait (uint32_t xz)
{
	for(uint32_t i = 0; i < xz*1000; i++)
		__ASM volatile ("nop");
}

void panic (int delay)
{
	while(1) {
		led_num(1);
		wait(delay);
		led_num(2);
		wait(delay);
		led_num(4);
		wait(delay);
		led_num(8);
		wait(delay);
	}
}

int argc = 0;
const char* argv[32] = {0};

void parse_args (char* cmd, uint32_t cmd_length)
{
	argc = 0;
	memset(argv, 0, sizeof(argv));

	int state = 0;

	for (int i = 0; i < cmd_length; i++, cmd++) {
		if (*cmd == ' ' || *cmd == '\t' || *cmd == '\n') {
			if (state) {
				argc++;
				*cmd = 0;
				state = 0;
			}
		} else {
			if (!state) {
				argv[argc] = cmd;
				state = 1;
			}
		}
	}
}

const char* help = "help, h, ? - print this message\n";

int cmd_exec (int argc, const char* argv[], usart_t* term)
{
	if (!strcmp(argv[0], "help") || argv[0][0] == '?' || argv[0][0] == 'h') {
		term_putstr(term, help);
		return 0;
	}


	return 1;
}


char cmd_buffer[USART_BUFF_SIZE];

/*
 * TODO:
 * - timer irq with screen control
 * - buttons
 * - buzzer pwm
 * - reostat adc
 *
 */

void main( void )
{
	leds_init();
	led_num(0);

	usart_t* usart_1 = &usart_devices[0];

	usart_init(usart_1, &usart_configs[0],
			usart_buffers[0][0], USART_BUFF_SIZE,
			usart_buffers[0][1], USART_BUFF_SIZE);

	usart_enable(usart_1);

	term_putstr(usart_1, "Egor Volvo Car-Troller\n");

#if 0
	char buffer[100];
	snprintf(buffer, sizeof(buffer), "%d\n", 123);
#endif

#if 0
	char* tail = 0;
	char numstr[4] = {'1', '2', '3', 0};
	int a = strtol(numstr, &tail, 10);

	if (a == 123) {
		led_num(1);
	} else {
		led_num(5);
	}
#endif

	int recv;

	term_putstr(usart_1, "~ # ");
	while (1) {

		recv = term_getline(usart_1, cmd_buffer, sizeof(cmd_buffer));

		if (recv > 0) {

			if (cmd_buffer[0] != LF) {

				parse_args(cmd_buffer, recv);
#if 1
				for (int i = 0; i < argc; i++) {
					term_putstr(usart_1, argv[i]);
					term_putstr(usart_1, "\n");
				}
				term_putstr(usart_1, "\n");
#endif
				int result = cmd_exec(argc, argv, usart_1);

				if (result < 0) {
	//				term_printf(usart_1, "ERROR: error code %d (0x%X)\n", result, result);
				} else if (result == 1) {
					term_putstr(usart_1, "ERROR: command not implemented\n");
				}

			}

			term_putstr(usart_1, "~ # ");

//			term_putstr(usart_1, cmd_buffer);
//			term_send(usart_1, cmd_buffer, recv);
//			term_printf(usart_1, "recvd %d bytes\n", recv);

		} else {
		}


	}

}
