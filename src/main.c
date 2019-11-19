/*
 * This file is part of MCUSim, an XSPICE library with microcontrollers.
 *
 * Copyright (C) 2017-2019 MCUSim Developers, see AUTHORS.txt for contributors.
 *
 * MCUSim is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MCUSim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#define F_CPU			12000000UL	/* CPU 12 MHz */

/*
 * Entry point of the Xling firmware.
 *
 * The microcontroller will be configured. Tasks and FreeRTOS scheduler will be
 * initialized and started here.
 */

#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "FreeRTOS.h"
#include "task.h"

#include "mcusim/drivers/avr-gcc/avr/display/sh1106/sh1106.h"
#include "mcusim/drivers/avr-gcc/avr/display/sh1106/sh1106_graphics.h"

#include "xling/graphics/xling.h"
#include "xling/graphics/luci.h"
#include "xling/graphics/luci_walking_01.h"
#include "xling/graphics/luci_walking_02.h"
#include "xling/graphics/luci_walking_03.h"
#include "xling/graphics/luci_walking_04.h"

#define SET_BIT(byte, bit)	((byte) |= (1U << (bit)))
#define CLEAR_BIT(byte, bit)	((byte) &= (uint8_t) ~(1U << (bit)))
#define IS_SET(byte, bit)	(((byte) & (1U << (bit))) >> (bit))

/* Buttons pins */
#define BTN1			PD3
#define BTN2			PD2
#define BTN3			PB4

/* OLED pins */
#define OLED_DC			PC5
#define OLED_RST		PC6
#define OLED_CS			PC7

/* OLED SPI pins */
#define OLED_MOSI		PB5
#define OLED_MISO		PB6
#define OLED_SCK		PB7

/* Battery charger pins */
#define BAT_STAT		PA0
#define BAT_LVL			PA3

/* Number of times to re-draw the display */
#define FRAME_ITER		20

static MSIM_SH1106DriverConf drv_conf = {
	.port_spi = &PORTB,
	.ddr_spi = &DDRB,
	.mosi = OLED_MOSI,
	.miso = OLED_MISO,
	.sck = OLED_SCK,
};
static MSIM_SH1106DisplayConf conf = {
	.rst_port = &PORTC,
	.rst_ddr = &DDRC,
	.cs_port = &PORTC,
	.cs_ddr = &DDRC,
	.dc_port = &PORTC,
	.dc_ddr = &DDRC,
	.rst = OLED_RST,
	.cs = OLED_CS,
	.dc = OLED_DC,
};
static volatile uint32_t ms = 0;
static volatile uint16_t bat_lvl = 0;
static volatile uint8_t bat_stat = 2;

/* Tasks for FreeRTOS kernel. */
static void	display_task(void *arg);
//static void	battery_monitor_task(void *arg);

/* Local function declarations. */
static void	init_timer2(void);
static void	init_adc(void);

/*
 * Entry point.
 */
int
main(void)
{
	/* Configure PORTC pins as output. */
	DDRC = 0xFF;
	/* Configure PORTA pins as input. */
	DDRA = 0x00;
	PORTA = 0x00;

	/* Keep CS high (display isn't selected). */
	SET_BIT(PORTC, OLED_CS);

	/* Power On the display. */
	CLEAR_BIT(PORTC, OLED_RST);
	_delay_ms(1);
	SET_BIT(PORTC, OLED_RST);
	_delay_ms(10);

	/* Start the driver for SH1106-based displays. */
	MSIM_SH1106__drvStart(&drv_conf);

	/* Setup Timer/Counter2 to count milliseconds. */
	init_timer2();

	/* Setup ADC to measure battery voltage. */
	init_adc();

	/* Create the tasks. */
	xTaskCreate(display_task, "display", 1024, NULL, 1, NULL);

	/* Start the FreeRTOS scheduler. */
	vTaskStartScheduler();

	for (;;);

	/* Stop the display driver. */
	MSIM_SH1106__drvStop();

	return 0;
}

/*
 * A display task.
 */
static void
display_task(void *arg __attribute((unused)))
{
	const uint8_t *ptr = NULL;
	MSIM_SH1106 *display = NULL;
	uint32_t delay_ms = 1;
	uint8_t frame_id = 1;
	uint8_t col = 0;
	char textbuf[32];

	/* Initialize an OLED display... */
	display = MSIM_SH1106_Init(&conf);

	/* ...and set it up */
	MSIM_SH1106_DisplayOff(display);
	MSIM_SH1106_SetContrast(display, 0x75);
	MSIM_SH1106_DisplayNormal(display);
	MSIM_SH1106_SetScanDirection(display, 0);
	MSIM_SH1106_DisplayOn(display);
	MSIM_SH1106_bufSend(display);

	/* Clean the display */
	for (uint32_t i = 0; i < 8; i++) {
		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, (uint8_t) i);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);

		MSIM_SH1106_bufClear(display);
		for (uint32_t j = 0; j < 128; j++) {
			MSIM_SH1106_bufAppend(display, 0);
		}
		MSIM_SH1106_bufSend(display);
	}

	while (1) {
		/* Remember a moment in time */
		delay_ms = ms;

		/* Re-draw images several times */
		for (uint32_t j = 0; j < FRAME_ITER; j++) {
			/* Clean the display */
			for (uint32_t i = 0; i < 8; i++) {
				MSIM_SH1106_bufClear(display);
				MSIM_SH1106_SetPage(display, (uint8_t) i);
				MSIM_SH1106_SetColumn(display, col);
				MSIM_SH1106_bufSend(display);

				MSIM_SH1106_bufClear(display);
				for (uint32_t k = 0; k < 5; k++) {
					MSIM_SH1106_bufAppend(display, 0);
				}
				MSIM_SH1106_bufSend(display);
			}

			/* Update sprite position */
			col = (uint8_t)((col >= 130) ? 5 : (col + 5));

			/* Choose an animation frame */
			switch (frame_id) {
			case 1:
				ptr = luci_walking_01;
				frame_id++;
				break;
			case 2:
				ptr = luci_walking_02;
				frame_id++;
				break;
			case 3:
				ptr = luci_walking_03;
				frame_id++;
				break;
			case 4:
				ptr = luci_walking_04;
				frame_id++;
				break;
			default:
				ptr = luci_walking_01;
				frame_id = 2;
				break;
			}

			for (uint32_t i = 0; i < 8; i++) {
				MSIM_SH1106_bufClear(display);
				MSIM_SH1106_SetPage(display, (uint8_t) i);
				MSIM_SH1106_SetColumn(display, col);
				MSIM_SH1106_bufSend(display);

				MSIM_SH1106_bufClear(display);
				MSIM_SH1106_bufAppendLast_PF(
				        display, &ptr[i * 59], 59);
				MSIM_SH1106_bufSend(display);
			}

			_delay_ms(65);
		}

		/* Calculate a delay */
		delay_ms = ms - delay_ms;

		snprintf(&textbuf[0], sizeof textbuf, "%d, %d",
		         bat_lvl, bat_stat);

		/* Draw battery level */
		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, 5);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);
		MSIM_SH1106_Print(display, &textbuf[0]);

		snprintf(&textbuf[0], sizeof textbuf, "%lu.%lu FPS",
		         1000 / (delay_ms/FRAME_ITER),
		         1000 % (delay_ms/FRAME_ITER));

		/* Draw FPS */
		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, 6);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);

		MSIM_SH1106_Print(display, &textbuf[0]);

		snprintf(&textbuf[0], sizeof textbuf, "%lu ms, %d",
		         delay_ms, FRAME_ITER);

		/* Draw delay and # of iterations */
		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, 7);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);

		MSIM_SH1106_Print(display, &textbuf[0]);

		ms = 0;

		/* Let the framebuffer hang for some time */
		_delay_ms(3000);
	}

	return;
}

static void
init_timer2(void)
{
	/* CTC mode, WGM22:0 = 2 */
	CLEAR_BIT(TCCR2B, WGM22);
	SET_BIT(TCCR2A, WGM21);
	CLEAR_BIT(TCCR2A, WGM20);

	/* OCR2A is 187 to get a 1ms delay (500 Hz) */
	TCNT2 = 0;
	OCR2A = 187;
	/* Enable output-compare interrupt */
	SET_BIT(TIMSK2, OCIE2A);

	/* Start timer, prescaler to 64: CS22:0 = 3 */
	SET_BIT(TCCR2B, CS22);
	CLEAR_BIT(TCCR2B, CS21);
	CLEAR_BIT(TCCR2B, CS20);
}

static void
init_adc(void)
{
	/* Select Vref = 1.1 V */
	SET_BIT(ADMUX, REFS1);
	CLEAR_BIT(ADMUX, REFS0);

	/*
	 * ADC clock frequency should be between 50 kHz and 200 kHz to achieve
	 * maximum resolution. Let's set an ADC clock prescaler to 64 at 12 MHz
	 * MCU clock frequency:
	 *
	 *     12,000,000 / 64 = 187.5 kHz
	 */
	SET_BIT(ADCSRA, ADPS2);
	SET_BIT(ADCSRA, ADPS1);
	CLEAR_BIT(ADCSRA, ADPS0);

	/* Select ADC3 as a single ended input */
	CLEAR_BIT(ADMUX, MUX4);
	CLEAR_BIT(ADMUX, MUX3);
	CLEAR_BIT(ADMUX, MUX2);
	SET_BIT(ADMUX, MUX1);
	SET_BIT(ADMUX, MUX0);

	/* Enable ADC, its interrupt, auto trigger (Free Running Mode) */
	SET_BIT(ADCSRA, ADIE);
	SET_BIT(ADCSRA, ADEN);
	SET_BIT(ADCSRA, ADATE);

	/* Run the first conversion */
	SET_BIT(ADCSRA, ADSC);
}

ISR(TIMER2_COMPA_vect)
{
	/* Update milliseconds counter */
	ms = (ms == UINT32_MAX) ? 0 : (ms + 1);
}

ISR(ADC_vect)
{
	/* Sample the battery voltage level */
	bat_lvl = (uint16_t)(((ADCH << 8) & 0x0300) | (ADCL & 0x00FF));

	/* Sample the battery status pin */
	bat_stat = PINA & 1U;

	/* Clear ADC result registers */
	ADCH = 0;
	ADCL = 0;
}
