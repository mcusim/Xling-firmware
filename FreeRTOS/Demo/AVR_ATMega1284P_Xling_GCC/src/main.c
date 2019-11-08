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
 *
 *
 * Entry point of the Xling firmware.
 */
#define F_CPU			12000000UL	/* CPU 12 MHz */

#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "mcusim/avr-gcc/avr/drivers/display/sh1106/sh1106.h"
#include "mcusim/avr-gcc/avr/drivers/display/sh1106/sh1106_graphics.h"
#include "xling/graphics/xling.h"
#include "xling/graphics/luci.h"

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

static MSIM_SH1106 *display;
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

static void	timer2_init(void);

int main(void)
{
	const uint8_t *ptr = oled_luci;
	uint32_t delay_ms = 1;
	char textbuf[32];

	/* Configure pins as input/output ones */
	DDRC = 0xFF;

	/* Keep CS high (display isn't selected) */
	SET_BIT(PORTC, OLED_CS);

	/* Power On the display */
	CLEAR_BIT(PORTC, OLED_RST);
	_delay_ms(1);
	SET_BIT(PORTC, OLED_RST);
	_delay_ms(10);

	/* Start the driver for SH1106-based displays */
	MSIM_SH1106__drvStart(&drv_conf);

	/* Setup Timer/Counter2 to count milliseconds */
	timer2_init();

	/* Enable interrupts globally */
	sei();

	/* Initialize an OLED... */
	display = MSIM_SH1106_Init(&conf);

	/* ...and set it up */
	MSIM_SH1106_DisplayOff(display);
	MSIM_SH1106_SetContrast(display, 0x75);
	MSIM_SH1106_DisplayNormal(display);
	MSIM_SH1106_SetScanDirection(display, 0);
	MSIM_SH1106_DisplayOn(display);
	MSIM_SH1106_bufSend(display);

	while (1) {
		/* Remember a moment in time */
		delay_ms = ms;

		/* Re-draw images several times */
		for (uint32_t j = 0; j < FRAME_ITER; j++) {
			for (uint32_t i = 0; i < 1024 / 128; i++) {
				MSIM_SH1106_bufClear(display);
				MSIM_SH1106_SetPage(display, (uint8_t) i);
				MSIM_SH1106_SetColumn(display, 2);
				MSIM_SH1106_bufSend(display);

				MSIM_SH1106_bufClear(display);
				MSIM_SH1106_bufAppendLast_PF(
				        display, &ptr[i * 128], 128);
				MSIM_SH1106_bufSend(display);
			}
		}

		/* Calculate a delay */
		delay_ms = ms - delay_ms;

		snprintf(&textbuf[0], sizeof textbuf, "%lu.%lu FPS",
		         1000 / (delay_ms/FRAME_ITER),
		         1000 % (delay_ms/FRAME_ITER));

		/* Draw FPS */
		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, 0);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);

		MSIM_SH1106_Print(display, &textbuf[0]);

		snprintf(&textbuf[0], sizeof textbuf, "(%lu ms, %d)",
		         delay_ms, FRAME_ITER);

		/* Draw delay and # of iterations */
		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, 1);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);

		MSIM_SH1106_Print(display, &textbuf[0]);

		ms = 0;

		/* Let the framebuffer hang for some time */
		_delay_ms(1000);
	}

	/* Stop the display driver */
	MSIM_SH1106__drvStop();

	return 0;
}

static void timer2_init(void)
{
	/* CTC mode, WGM22:0 = 2 */
	TCCR2A |= (1<<WGM21);
	TCCR2A &= (uint8_t)(~(1<<WGM20));
	TCCR2B &= (uint8_t)(~(1<<WGM22));

	/* OCR2A is 187 to get a 1ms delay (500 Hz) */
	TCNT2 = 0;
	OCR2A = 187;
	TIMSK2 |= (1<<OCIE2A);	/* Enable output-compare interrupt */

	/* Start timer, prescaler to 64: CS22:0 = 3 */
	TCCR2B |= (1<<CS22);
	SET_BIT(TCCR2B, CS22);
	CLEAR_BIT(TCCR2B, CS21);
	CLEAR_BIT(TCCR2B, CS20);
}

ISR(TIMER2_COMPA_vect)
{
	ms = (ms == UINT32_MAX) ? 0 : (ms + 1);
}
