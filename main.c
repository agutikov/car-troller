
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

		.dma_channel_regs = DMA1_Channel1,
		.dma_irqn = DMA1_Channel1_IRQn,
		.dma_tc_flag = DMA1_FLAG_TC1
	},
	{

	},
	{

	}
};


void usart1_isr(void) __attribute__ ((noreturn)) __attribute__ ((naked));
void usart1_tx_dma_isr(void) __attribute__ ((noreturn)) __attribute__ ((naked));
void usart1_isr (void)
{
	usart_isr(&usart_devices[0]);
}
void usart1_tx_dma_isr(void)
{
//	usart_handle_tx_dma_irq(&usart_devices[0]);
}


char buffer[128];

uint32_t leds[4][2] = {
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
void led_en (int led)
{
	if (led < 4)
		GPIO_WriteBit((GPIO_TypeDef *)leds[led][0], leds[led][1], 1);
}
void led_dis (int led)
{
	if (led < 4)
		GPIO_WriteBit((GPIO_TypeDef *)leds[led][0], leds[led][1], 0);
}
void led_num (int n)
{
	for (int i = 0; i < 4; i++) {
		if (n & (1 << i))
			GPIO_WriteBit((GPIO_TypeDef *)leds[i][0], leds[i][1], 1);
		else
			GPIO_WriteBit((GPIO_TypeDef *)leds[i][0], leds[i][1], 0);
	}
}

void main( void )
{
	leds_init();

	usart_t* usart_1 = &usart_devices[0];

	usart_init(usart_1, &usart_configs[0],
			usart_buffers[0][0], USART_BUFF_SIZE,
			usart_buffers[0][1], USART_BUFF_SIZE);

	usart_enable(usart_1);
#if 0
	uint8_t byte;
	int rcvd = 0;

	while (1) {

		if (!rcvd && USART_GetFlagStatus(usart_1->usart_regs, USART_FLAG_RXNE) != RESET) {
			byte = USART_ReceiveData(usart_1->usart_regs);
			rcvd = 1;
		}


		if (rcvd && USART_GetFlagStatus(usart_1->usart_regs, USART_FLAG_TXE) != RESET) {
			USART_SendData(usart_1->usart_regs, byte);
			rcvd = 0;
		}
	}
#endif




	usart_send(usart_1, "start\n", 6);


	while (1) {
		continue;


		int recv = usart_recv(usart_1, buffer, sizeof(buffer));
		if (recv) {
			usart_send(usart_1, "1234567890\n", 11);
			usart_send(usart_1, buffer, recv);
		}


	}
}




