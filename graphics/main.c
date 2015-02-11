
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>

#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_ts.h"
#include "stm32f429i_discovery_io.h"
#include "stm32f429i_discovery_lcd.h"

#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma2d.h"

#include "images.c"


extern void Touchscreen_Calibration(void);
extern uint16_t Calibration_GetX(uint16_t x);

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


#define LCD_FRAME_BUFFER_SIZE (320*240*4)

extern uint32_t __framebuffers;

uint32_t framebuffer_back_0 = (uint32_t)&__framebuffers;
uint32_t framebuffer_front_0 = (uint32_t)&__framebuffers + (320*240*4);
uint32_t framebuffer_back_1 = (uint32_t)&__framebuffers + 2*(320*240*4);
uint32_t framebuffer_front_1 = (uint32_t)&__framebuffers + 3*(320*240*4);


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
	for(uint32_t i = 0; i < xz*18000; i++)
		asm("nop");
}

void panic (int delay)
{
	while(1) {
		BSP_LED_Off(LED3);
		BSP_LED_Off(LED4);
		wait(delay);
		BSP_LED_On(LED3);
		BSP_LED_Off(LED4);
		wait(delay);
		BSP_LED_Off(LED3);
		BSP_LED_On(LED4);
		wait(delay);
		BSP_LED_On(LED3);
		BSP_LED_On(LED4);
		wait(delay);
	}
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

typedef struct argb32 {
	uint8_t a;
	uint8_t r;
	uint8_t g;
	uint8_t b;
} argb32_t;

void BSP_LCD_DrawBitmap_Ex(uint32_t X, uint32_t Y, uint8_t *pBmp)
{
  uint32_t index = 0, width = 0, height = 0, bitpixel = 0;
  uint32_t address;

  /* Get bitmap data address offset */
  index = *(__IO uint16_t *) (pBmp + 10);
  index |= (*(__IO uint16_t *) (pBmp + 12)) << 16;

  /* Read bitmap width */
  width = *(uint16_t *) (pBmp + 18);
  width |= (*(uint16_t *) (pBmp + 20)) << 16;

  /* Read bitmap height */
  height = *(uint16_t *) (pBmp + 22);
  height |= (*(uint16_t *) (pBmp + 24)) << 16;

  /* Read bit/pixel */
  bitpixel = *(uint16_t *) (pBmp + 28);

  /* Set Address */
  address = framebuffer_front_0 + (((BSP_LCD_GetXSize()*Y) + X)*(4));

  /* bypass the bitmap header */
  pBmp += (index + (width * (height - 1) * (bitpixel/8)));

  /* Convert picture to ARGB8888 pixel format */
  for(index=0; index < height; index++)
  {


	/* Pixel format conversion */
	for (int pixel = 0; pixel < width; pixel++) {

		argb32_t back = ((argb32_t*)address)[pixel];
		argb32_t front = ((argb32_t*)pBmp)[pixel];

		back.a = 255;

		uint32_t tmp;

		tmp = back.r*(256 - front.r)*(256 - front.a)/32/128;
		if (tmp > 255) tmp = 255;
		back.r = tmp;

		tmp = back.g*(256 - front.g)*(256 - front.a)/32/128;
		if (tmp > 255) tmp = 255;
		back.g = tmp;

		tmp = back.b*(256 - front.b)*(256 - front.a)/32/128;
		if (tmp > 255) tmp = 255;
		back.b = tmp;

		((argb32_t*)address)[pixel] = back;
	}


	  /* Increment the source and destination buffers */
	  address+=  ((BSP_LCD_GetXSize() - width + width)*4);
	  pBmp -= width*(bitpixel/8);
  }
}

#define RELAY_PORT 	GPIOG

#define RELAY_0_PIN		GPIO_PIN_11
#define RELAY_1_PIN		GPIO_PIN_12
#define RELAY_2_PIN		GPIO_PIN_13
#define RELAY_3_PIN		GPIO_PIN_14

#define RELAY_PORT_CLK_ENABLE 	__GPIOG_CLK_ENABLE

void PushButton_Action(int button_id)
{
	switch (button_id) {
	case 0:
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_1_PIN | RELAY_2_PIN | RELAY_3_PIN, 0);
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_0_PIN, 1);
		break;
	case 1:
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_0_PIN | RELAY_2_PIN | RELAY_3_PIN, 0);
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_1_PIN, 1);
		break;
	case 2:
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_0_PIN | RELAY_1_PIN | RELAY_3_PIN, 0);
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_2_PIN, 1);
		break;
	case 3:
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_0_PIN | RELAY_1_PIN | RELAY_2_PIN, 0);
		HAL_GPIO_WritePin(RELAY_PORT, RELAY_3_PIN, 1);
		break;
	}
}


#define ABS(_X_) (((_X_) > 0) ? (_X_) : -(_X_))

void main( void )
{
	/* STM32F4xx HAL library initialization:
	- Configure the Flash prefetch, instruction and Data caches
	- Configure the Systick to generate an interrupt each 1 msec
	- Set NVIC Group Priority to 4
	- Global MSP (MCU Support Package) initialization
	*/
	HAL_Init();

	/* Configure LED3 and LED4 */
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);

	BSP_LED_Off(LED3);
	BSP_LED_Off(LED4);

	/* Configure the system clock to 180 MHz */
	SystemClock_Config();

	/* Configure USER Button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

	/* Initialize the LCD */
	// LTDC, MSP, ili9341 and SDRAM init inside
	BSP_LCD_Init();

	/* Enable The LCD */
	BSP_LCD_DisplayOn();



	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, framebuffer_back_0);
	BSP_LCD_SetLayerVisible(LCD_BACKGROUND_LAYER, ENABLE);

	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, framebuffer_front_0);
	BSP_LCD_SetLayerVisible(LCD_FOREGROUND_LAYER, ENABLE);

	BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_Clear(LCD_COLOR_BLACK);

	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_Clear(LCD_COLOR_BLACK);



	/* Touch screen initialization */
	Touchscreen_Calibration();

	/* Relay pins */
	{
		GPIO_InitTypeDef  GPIO_InitStruct;

		/* Enable the GPIO_LED Clock */
		RELAY_PORT_CLK_ENABLE();

		/* Configure the GPIO_LED pin */
		GPIO_InitStruct.Pin = RELAY_0_PIN | RELAY_1_PIN | RELAY_2_PIN | RELAY_3_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

		HAL_GPIO_Init(RELAY_PORT, &GPIO_InitStruct);

		HAL_GPIO_WritePin(RELAY_PORT, RELAY_0_PIN | RELAY_1_PIN | RELAY_2_PIN | RELAY_3_PIN, 0);
	}


	int x = 0;
	int y = 0;
	int prev_x = 0;
	int prev_y = 0;
	TS_StateTypeDef  TS_State;

	BSP_LCD_SetFont(&Font24);

	BSP_LCD_SetLayerVisible(LCD_BACKGROUND_LAYER, DISABLE);
	BSP_LCD_SetLayerVisible(LCD_FOREGROUND_LAYER, ENABLE);

	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

	int prev_menu_selected = -1;
	int menu_selected = 0;

	while(1) {

		BSP_TS_GetState(&TS_State);

		/* Read the coordinate */
		x = Calibration_GetX(TS_State.X);
		y = Calibration_GetX(TS_State.Y);

		if (ABS(prev_x - x) > 20 || ABS(prev_y - y) > 20) {

			prev_x = x;
			prev_y = y;


			if (x < 120) {
				if (y < 80) {
					menu_selected = 0;
				} else if (y < 160) {
					menu_selected = 2;
				} else if (y < 240) {

				} else {

				}
			} else {
				if (y < 80) {
					menu_selected = 1;
				} else if (y < 160) {
					menu_selected = 3;
				} else if (y < 240) {

				} else {

				}
			}




			if (prev_menu_selected != menu_selected) {

				prev_menu_selected = menu_selected;

//				BSP_LCD_Clear(LCD_COLOR_BLACK);
				BSP_LCD_DrawBitmap(0, 0, menu_0_bmp);

				char buffer[128] = "X: ";
				i2str(&buffer[3], x, 0, 0);
				BSP_LCD_DisplayStringAt(10, 280, (uint8_t*)buffer, LEFT_MODE);

				memcpy(buffer, "Y: ", 4);
				i2str(&buffer[3], y, 0, 0);
				BSP_LCD_DisplayStringAt(10, 300, (uint8_t*)buffer, LEFT_MODE);


				PushButton_Action(menu_selected);


				BSP_LCD_DrawBitmap_Ex((menu_selected % 2) * 120, (menu_selected / 2) * 80, selected_120x80_bmp);
			}

		}


	}

}





