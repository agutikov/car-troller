
/*

	pl2303 usart:
	A9 - usart1 tx
	A10 - usart1 rx

	E0 - buzzer - no avaliable TIM pwm, software pwm by TIM2 irq

	C6, C7, D13, D6 - LEDs GPIO

	E2, E3, E4, E5 - keys GPIO EXTI

	C0, C1 - reostat, ADC channel 10,11

	C8 - rti screen control GPIO + TIM3 used


 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"

#include "usart.h"

#include <ctype.h>
#include <limits.h>

/*
 * Convert a string to a long integer.
 *
 * Ignores `locale' stuff. Assumes that the upper and lower case
 * alphabets and digits are each contiguous.
 */
long strtol (const char *nptr, char **endptr, int base)
{
	const char *s = nptr;
	unsigned long acc;
	unsigned char c;
	unsigned long cutoff;
	int neg = 0, any, cutlim;

	/*
	* Skip white space and pick up leading +/- sign if any.
	* If base is 0, allow 0x for hex and 0 for octal, else
	* assume decimal; if base is already 16, allow 0x.
	*/

	do {
		c = *s++;
	} while (isspace(c));

	if (c == '-') {
		neg = 1;
		c = *s++;
	} else if (c == '+')
		c = *s++;


	if ((base == 0 || base == 16) &&
			c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}

	if (base == 0)
		base = c == '0' ? 8 : 10;

	/*
	* Compute the cutoff value between legal numbers and illegal
	* numbers. That is the largest legal value, divided by the
	* base. An input number that is greater than this value, if
	* followed by a legal input character, is too big. One that
	* is equal to this value may be valid or not; the limit
	* between valid and invalid numbers is then based on the last
	* digit. For instance, if the range for longs is
	* [-2147483648..2147483647] and the input base is 10,
	* cutoff will be set to 214748364 and cutlim to either
	* 7 (neg==0) or 8 (neg==1), meaning that if we have accumulated
	* a value > 214748364, or equal but the next digit is > 7 (or 8),
	* the number is too big, and we will return a range error.
	*
	* Set any if any `digits' consumed; make it negative to indicate
	* overflow.
	*/

	cutoff = neg ? -(unsigned long)LONG_MIN : LONG_MAX;
	cutlim = cutoff % (unsigned long)base;
	cutoff /= (unsigned long)base;

	for (acc = 0, any = 0;; c = *s++) {

		if (!isascii(c))
			break;
		if (isdigit(c))
			c -= '0';
		else if (isalpha(c))
			c -= isupper(c) ? 'A' - 10 : 'a' - 10;
		else
			break;

		if (c >= base)
			break;

		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
			any = -1;
		else {
			any = 1;
			acc *= base;
			acc += c;
		}
	}

	if (any < 0) {
		acc = neg ? LONG_MIN : LONG_MAX;
	} else if (neg)
		acc = -acc;

	if (endptr != 0)
		*((const char **)endptr) = any ? s - 1 : nptr;

	return (acc);
}

int i2str (char* b, int i, int base, int field)
{
	char const digit[] = "0123456789ABCDEF";
	int len = 0;

	if (base == 0)
		base = 10;

	if (i < 0) {
		*b++ = '-';
		len++;
		i = -i;
	}

	int shifter = i;

	do { // Move to where representation ends
		++b;
		len++;
		shifter = shifter/base;
	} while (shifter);

	int f = field - len;

	if (f > 0) {
		b += f;
		len += f;
	}

	*b = '\0';

	do { // Move back, inserting digits as u go
		*--b = digit[i%base];
		i = i/base;
	} while (i);

	if (f > 0) {
		while (--f)
			*--b = '0';
	}

	return len;
}


void wait (uint32_t xz)
{
	for(uint32_t i = 0; i < xz*1000; i++)
		__ASM volatile ("nop");
}


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

usart_t* term_1 = &usart_devices[0];

void printstr (const char* str)
{
	term_putstr(term_1, str);
}
void printnum (int i, int base)
{
	char buffer[32];
	i2str(buffer, i, base, 0);
	printstr(buffer);
}
uint8_t hex_first (uint8_t byte)
{
	byte >>= 4;
	return byte >= 0xA ? byte + 'A' - 10 : byte + '0';
}
uint8_t hex_last (uint8_t byte)
{
	byte &= 0x0F;
	return byte >= 0xA ? byte + 'A' - 10 : byte + '0';
}
void printhex (void* vptr, uint32_t size)
{
	uint8_t* ptr = vptr;
	term_putstr(term_1, "{ ");

	char delim[] = ", ";
	char buffer[4] = "0x--";
	while (size-- > 0) {
		buffer[2] = hex_first(*ptr);
		buffer[3] = hex_last(*ptr);
		ptr++;
		usart_send(term_1, buffer, sizeof(buffer));
		if (size > 0) {
			usart_send(term_1, delim, sizeof(delim));
		}
	}
	term_putstr(term_1, " }\n");
}

void usart1_isr (void)
{
	usart_isr(&usart_devices[0]);
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
void led_set (int led, int val)
{
	if (led < LED_NUMBER)
		GPIO_WriteBit((GPIO_TypeDef *)leds[led][0], leds[led][1], val);
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

#define RTI_BIT_US	    415
#define RTI_PERIOD_BITS  (100000 / RTI_BIT_US)

typedef enum BIT_ORDER {
	LSB_FIRST = 0,
	MSB_FIRST = 1
} BIT_ORDER_T;
BIT_ORDER_T rti_bit_order = MSB_FIRST;

// Copy whole head and tail into message.
uint8_t rti_head[] = {0x50, 0xE0};
uint8_t rti_tail[] = {0x80, 0x10, 0x87};

// Choose one of modes and one of brightnesses.
uint8_t rti_mode[] = {0x2D, // enable
			0x05, // disable
			0x40, 0x45, 0x4C, 0x46}; // from another project
uint8_t rti_brightness[] = {0xF8, // disable
				0x08, // min
				0x29, // mid
				0xE9, // max
				0x20, 0x61, 0x62, 0x23, 0x64, // from another project
				0x25, 0x26, 0x67, 0x68, 0x29,
				0x2A, 0x2C, 0x6B, 0x6D, 0x6E,
				0x2F};

/* Lack of syncronization will break nothing,
   rti monitor is resistant to wrong messages. */
uint8_t rti_msg[7];
int rti_bit_counter = 0;

int rti_period_counter = 0;

void rti_msg_reset()
{
	memcpy(rti_msg, rti_head, 2);
	memcpy(&rti_msg[4], rti_tail, 3);
	rti_msg[2] = rti_mode[0];
	rti_msg[3] = rti_brightness[0];
}
void rti_print_msg()
{
	printstr("RTI message: ");
	printhex(rti_msg, sizeof(rti_msg));
}

void rti_init (void)
{
	rti_msg_reset();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// prescaler makes one '1' in period equal 1 usecond
	uint16_t PrescalerValue = (SystemCoreClock / 1000000) - 1;
	TIM_TimeBaseStructure.TIM_Period = RTI_BIT_US;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
// Without "helpers" definitely faster ;)
//	TIM3->PSC = (SystemCoreClock / 1000000) - 1;
//	TIM3->ARR = 415;

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	TIM3->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM3_IRQn);

	TIM_Cmd(TIM3, ENABLE);
//	TIM3->CR1 |= TIM_CR1_CEN;

	printstr("RTI configured and enabled\n");
	printstr("message length: ");
	printnum(sizeof(rti_msg), 0);
	printstr(" bytes\n");
	rti_print_msg();
}

void rti_set_mode (unsigned int mode_id)
{
	if (mode_id < sizeof(rti_mode)) {
		rti_msg[2] = rti_mode[mode_id];
	}
}
void rti_set_brightness (unsigned int brightness_id)
{
	if (brightness_id < sizeof(rti_brightness)) {
		rti_msg[3] = rti_brightness[brightness_id];
	}
}

void rti_tick (void)
{
	if (!rti_period_counter) {
		rti_period_counter = RTI_PERIOD_BITS;
		rti_bit_counter = 0;
	} else {
		if (rti_bit_counter < sizeof(rti_msg)*8) {
			uint8_t bit;

			bit = rti_msg[rti_bit_counter/8];

			if (rti_bit_order == LSB_FIRST)
				bit &= 0x01 << (rti_bit_counter % 8);
			else // if (rti_bit_order == MSB_FIRST)
				bit &= 0x80 >> (rti_bit_counter % 8);

			GPIO_WriteBit(GPIOC, GPIO_Pin_8, bit);
			rti_bit_counter++;
		}
	}
}



void timer3_isr (void)
{
	TIM3->SR &= ~TIM_SR_UIF;
	rti_tick();
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

void reostat_adc_init (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// power for light sensor
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOC, GPIO_Pin_0, 1);

	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	/* Enable ADC1 clock so that we can talk to it */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Put everything back to power-on defaults */
	ADC_DeInit(ADC1);

	ADC_InitTypeDef  ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
	/* ADC1 and ADC2 operate independently */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	/* Disable the scan conversion so we do one at a time */
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	/* Don't do continuous conversions - do them on demand */
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	/* Start conversin by software, not an external trigger */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	/* Conversions are 12 bit - put them in the lower 12 bits of the result */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	/* Say how many channels would be used by the sequencer */
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	/* Now do the setup */
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC1))
	{}
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while (ADC_GetCalibrationStatus(ADC1))
	{}

	printstr("Reostat ADC for PC0 and PC1 configured\n");
}

uint16_t reostat_get_value (uint8_t channel)
{
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
	{};

	uint16_t value = ADC_GetConversionValue(ADC1);

	return value;
}

int sample_counter = 0;
uint16_t sensor_sample[100] = {0};

void light_sensor_sample_fill (void)
{
	for (int j = 0; j < 10; j++)
		for (int i = 0; i < sizeof(sensor_sample)/sizeof(sensor_sample[0]); i++) {
			sensor_sample[i] = reostat_get_value(10);
		}
}

void light_sensor_tick (int print)
{
	sensor_sample[sample_counter] = reostat_get_value(10);
	sample_counter++;
	if (sample_counter == sizeof(sensor_sample)/sizeof(sensor_sample[0])) {
		sample_counter = 0;
	}

	if (print) {
		uint32_t avg = 0;
		for (int i = 0; i < sizeof(sensor_sample)/sizeof(sensor_sample[0]); i++) {
			avg += sensor_sample[i];
		}
		avg /= sizeof(sensor_sample)/sizeof(sensor_sample[0]);

		printstr("sensor value = ");
		printnum(avg, 0);
		printstr("\n");
	}
}

void reostat_print (void)
{
	//TODO: where to find mapping ADC channels onto GPIO ?
	uint16_t adc0 = reostat_get_value(10);
	printstr("\nADC 0: ");
	printnum(adc0, 0);

//	uint16_t adc1 = reostat_get_value(11);
//	printstr("\nADC 1: ");
//	printnum(adc1, 0);

	printstr("\n");
}

void buzzer_init (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	TIM2->PSC = (SystemCoreClock / 44000) - 1;
	TIM2->ARR = 44000/440;

	NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->CR1 |= TIM_CR1_CEN;

	printstr("Buzzer\n");
}
void buzzer_set_freq (uint16_t f)
{
	TIM2->ARR = 44000/f;
}
void buzzer_enable ()
{
	TIM2->DIER |= TIM_DIER_UIE;
}
void buzzer_disable ()
{
	TIM2->DIER &= ~TIM_DIER_UIE;
}

int buzzer_state = 0;
void buzzer_tick ()
{
	GPIO_WriteBit(GPIOE, GPIO_Pin_0, buzzer_state);

	buzzer_state = !buzzer_state;
}

void beep (uint16_t f, uint16_t time)
{
	if (f) {
		TIM2->ARR = 44000/f;
		TIM2->DIER |= TIM_DIER_UIE;
	}
	wait(time*4);
	if (f) {
		TIM2->DIER &= ~TIM_DIER_UIE;
	}
}

typedef struct note {
	uint16_t freq;
	uint16_t dur;
} note_t;
void buzzer_play (note_t* notes, uint32_t length, uint32_t speed) {
	TIM2->DIER |= TIM_DIER_UIE;
	for (int i = 0; i < length; i++) {
		if (notes->freq) {
			TIM2->ARR = 44000/notes[i].freq;
			wait(notes[i].dur*speed);
		} else {
			TIM2->DIER &= ~TIM_DIER_UIE;
			wait(notes[i].dur*speed);
			TIM2->DIER |= TIM_DIER_UIE;
		}
		printnum(i, 0);
		printstr("\n");
	}
	TIM2->DIER &= ~TIM_DIER_UIE;
}

void timer2_isr (void)
{
	TIM2->SR &= ~TIM_SR_UIF;
	buzzer_tick();
}

int argc = 0;
char* argv[32] = {0};

void split_args (char* cmd, uint32_t cmd_length)
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


note_t star_wars[] = {
	{440, 500}, {440, 500}, {440, 500}, {349, 350}, {523, 150},
	{440, 500}, {349, 350}, {523, 150}, {440, 1000},
	{659, 500}, {659, 500}, {659, 500}, {698, 350}, {523, 150},
	{415, 500}, {349, 350}, {523, 150}, {440, 1000},
	{880, 500}, {440, 350}, {440, 150}, {880, 500}, {830, 250}, {784, 250},
	{740, 125}, {698, 125}, {740, 250},

	{0, 250},

	{455, 250}, {622, 500}, {587, 250}, {554, 250},
	{523, 125}, {466, 125}, {523, 250},

	{0, 250},

	{349, 125}, {415, 500}, {349, 375}, {440, 125},
	{523, 500}, {440, 375}, {523, 125}, {659, 1000},
	{880, 500}, {440, 350}, {440, 150}, {880, 500}, {830, 250}, {784, 250},
	{740, 125}, {698, 125}, {740, 250},

	{0, 250},

	{455, 250}, {622, 500}, {587, 250}, {554, 250},
	{523, 125}, {466, 125}, {523, 250},

	{0, 250},

	{349, 250}, {415, 500}, {349, 375}, {523, 125},
	{440, 500}, {349, 375}, {261, 125}, {440, 1000},

	{0, 100},
};

const char* help = "help, h, ? - print this message\n";

//TODO: structure of commands - tree with placeholders for arguments

int cmd_exec (int argc, char* argv[], usart_t* term)
{
	if (!strcmp(argv[0], "help") || argv[0][0] == '?' || argv[0][0] == 'h') {
		term_putstr(term, help);
		return 0;
	}

	if (!strcmp(argv[0], "rti") && argc > 1) {
		if (!strcmp(argv[1], "reset")) {
			rti_msg_reset();
			rti_print_msg();
			return 0;
		}
		if (!strcmp(argv[1], "brightness") && argc == 3) {
			int brightness_id = strtol(argv[2], 0, 0);
			rti_set_brightness(brightness_id);
			return 0;
		}
		if (!strcmp(argv[1], "mode") && argc == 3) {
			int mode_id = strtol(argv[2], 0, 0);
			rti_set_mode(mode_id);
			return 0;
		}
		if (!strcmp(argv[1], "show")) {
			rti_print_msg();
			return 0;
		}
		if (!strcmp(argv[1], "msg") && argc == 2 + sizeof(rti_msg)) {
			for (int i = 0; i < sizeof(rti_msg); i++) {
				rti_msg[i] = strtol(argv[2 + i], 0, 16);
			}
			rti_print_msg();
			return 0;
		}
	}

	if (!strcmp(argv[0], "AT")) {
		for (int i = 0; i < strlen(argv[1]); i++) {
			argv[1][i] = (char)tolower((int)argv[1][i]);
		}
		if (!strcmp(argv[1], "modeswitch") && argc == 3) {

			return 0;
		}
	}

	if (!strcmp(argv[0], "adc")) {
		if (argc == 1) {
			reostat_print();
			return 0;
		}
		if (!strcmp(argv[1], "cont") && argc == 4) {
			int counter = strtol(argv[2], 0, 0);
			int period = strtol(argv[3], 0, 0);
			while (counter--) {
				reostat_print();
				wait(period);
			}
			return 0;
		}
	}
	if (!strcmp(argv[0], "light")) {
		light_sensor_sample_fill();
		int counter = strtol(argv[1], 0, 0);
		int period = strtol(argv[2], 0, 0);
		while (counter--) {
			light_sensor_tick(1);
			wait(period);
		}
		return 0;
	}
	if (!strcmp(argv[0], "error")) {
		return -100500;
	}
	if (!strcmp(argv[0], "buzzer")) {
		buzzer_set_freq(500);
		buzzer_enable();
		wait(1000);
		buzzer_set_freq(1000);
		wait(1000);
		buzzer_set_freq(300);
		wait(1000);
		buzzer_disable();

		return 0;
	}
	if (!strcmp(argv[0], "starwars")) {
		buzzer_play(star_wars, sizeof(star_wars)/sizeof(star_wars[0]), 10);

		return 0;
	}

	return 1;
}

char cmd_buffer[USART_BUFF_SIZE];

/*
 * TODO:
 * - buzzer pwm
 * - buttons
 * - rtc
 * - system timer - for custom timers with callbacks and counters
 * - configuration for GPIO, NVIC, DMA - in one block, not in each driver
 * - print configuration during initialization
 *
 * - usart1 mode:
 * 		- text terminal
 * 		- binary protocol for communication with client
 * 		- lisp interpreter
 *
 * - terminal: history, edit, autocomplete?
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

	term_putstr(usart_1, "\n\n>>>>>>>> Egor Volvo Car-Troller <<<<<<<<<<\n");

	buzzer_init();
	rti_init();
	reostat_adc_init();

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

#if 0
	char numstr[4] = {'1', '2', '3', 0};
	int a = atoi(numstr);
#endif

	int recv;

	term_putstr(usart_1, "~ # ");
	while (1) {

		recv = term_getline(usart_1, cmd_buffer, sizeof(cmd_buffer));

		if (recv > 0) {
//			printhex(cmd_buffer, recv);

			if (cmd_buffer[0] != LF) {

				split_args(cmd_buffer, recv);
#if 0
				for (int i = 0; i < argc; i++) {
					term_putstr(usart_1, argv[i]);
					term_putstr(usart_1, "\n");
				}
				term_putstr(usart_1, "\n");
#endif
				int result = cmd_exec(argc, argv, usart_1);

				if (result < 0) {
					printstr("ERROR: error code ");
					printnum(result, 0);
					printstr("\n");
//					term_printf(usart_1, "ERROR: error code %d (0x%X)\n", result, result);
				} else if (result == 1) {
//					term_putstr(usart_1, "ERROR: command not implemented\n");
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
