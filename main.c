
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_can.h"
#include "stm32f10x_cec.h"
#include "stm32f10x_crc.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_sdio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_wwdg.h"
#include "misc.h"



void uart_init (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
}

void send_ch (uint8_t ch)
{

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	
	USART_SendData(USART1, ch);

}

void send_str (const char* str)
{
	if (!str) {
		send_str("send_str(): NULL pointer\n\r");
	}
	for (const char* ptr = str; *ptr != 0; ptr++)
		send_ch(*ptr);
}

void main( void )
{
	uart_init();

	send_str("Hello!\n\r");

	while (1) {



	}




#if 0
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

	uint8_t C6 = 0;
	uint8_t C7 = 0;
	uint8_t D6 = 0;
	uint8_t D13 = 0;

	while (1) {

		for (int count_C6 = 0; count_C6 < 10; count_C6++) {
			C6 = !C6;
			GPIO_WriteBit(GPIOC, GPIO_Pin_6, C6);

			for (int count_C7 = 0; count_C7 < 10; count_C7++) {
				C7 = !C7;
				GPIO_WriteBit(GPIOC, GPIO_Pin_7, C7);

				for (int count_D13 = 0; count_D13 < 10; count_D13++) {
					D13 = !D13;
					GPIO_WriteBit(GPIOD, GPIO_Pin_13, D13);

					for (int count_D6 = 0; count_D6 < 10; count_D6++) {
						D6 = !D6;
						GPIO_WriteBit(GPIOD, GPIO_Pin_6, D6);

						for (int _i = 0; _i < 100000; _i++) {
							__ASM volatile ("nop");
						}

					}
				}
			}
		}


	}
#endif

}






void assert_param (void* ptr)
{
}


