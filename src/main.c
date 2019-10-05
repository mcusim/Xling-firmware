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
#define F_TWI			600000L		/* TWI 600 kHz */

#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "mcusim/avr-gcc/avr/drivers/display/sh1106.h"
#include "mcusim/avr-gcc/avr/drivers/display/sh1106_graphics.h"
#include "xling/graphics/xling.h"
#include "xling/graphics/luci.h"

#define SET_BIT(byte, bit)	((byte)|=(1U<<(bit)))
#define CLEAR_BIT(byte, bit)	((byte)&=(uint8_t)~(1U<<(bit)))
#define IS_SET(byte, bit)	(((byte)&(1U<<(bit)))>>(bit))

#define BTN1			PD0
#define BTN2			PD1
#define BTN3			PD2
#define OLED_RES		PD3
#define OLED_PWR		PD4

#define FRAME_ITER		10

/* SH1106-based display configuration for TWI interface. */
static struct MSIM_SH1106DriverConf drv_conf = {
	.cpu_f = F_CPU,
	.twi_f = F_TWI,
};
static struct MSIM_SH1106DisplayConf conf = {
	.twi_addr = 0x3D,
};

/* SH1106-based display configuration for TWI (bit-bang) interface. */
//static struct MSIM_SH1106DisplayConf conf = {
//	.twi_port = &PORTC,
//	.twi_ddr = &DDRC,
//	.sda = PC4,
//	.scl = PC5,
//	.twi_addr = 0x3D,
//};

static volatile uint32_t ms = 0;
static struct MSIM_SH1106 *display;

static void	timer2_init(void);

int main(void)
{
	char textbuf[32];
//	uint8_t img = 1;
	uint32_t delay_ms = 1;
	const uint8_t *ptr = oled_luci;

	/* Configure pins as input/output ones. */
	DDRD = 0x00;				/* all to input */
	SET_BIT(DDRD, OLED_RES);		/* output */
	SET_BIT(DDRD, OLED_PWR);		/* output */
	DDRC = 0x00;				/* all to input */
	SET_BIT(DDRC, PC1);			/* output */

	/* Set initial values. */
	SET_BIT(PORTD, OLED_RES);
	CLEAR_BIT(PORTD, OLED_PWR);
	_delay_ms(1);

	/* We've to switch OLED on. */
	CLEAR_BIT(PORTD, OLED_RES);
	_delay_ms(1);
	SET_BIT(PORTD, OLED_RES);
	_delay_ms(1);
	SET_BIT(PORTD, OLED_PWR);
	_delay_ms(100);

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
	MSIM_SH1106_SetContrast(display, 0x57);
	MSIM_SH1106_DisplayNormal(display);
	MSIM_SH1106_DisplayOn(display);
	MSIM_SH1106_Send(display);

	while (1) {
		/* Remember a moment in time */
		delay_ms = ms;

		/* Re-draw images several times */
		for (uint32_t j = 0; j < FRAME_ITER; j++) {
			for (uint32_t i = 0; i < 1024 / 128; i++) {
				MSIM_SH1106_Clean(display);
				MSIM_SH1106_SetPage(display, (uint8_t) i);
				MSIM_SH1106_SetColumn(display, 2);

				MSIM_SH1106_Write(display, CB_LASTRAM);
				MSIM_SH1106_WriteData_PF(
				        display, &ptr[i * 128], 128);

				MSIM_SH1106_Send(display);
			}
		}

		/* Calculate a delay */
		delay_ms = ms - delay_ms;

		/* Draw FPS */
		snprintf((char *)&textbuf[0], sizeof textbuf, "%lu.%lu FPS",
		         1000 / (delay_ms/FRAME_ITER),
		         1000 % (delay_ms/FRAME_ITER));
		MSIM_SH1106_Clean(display);
		MSIM_SH1106_SetPage(display, 0);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_Send(display);
		MSIM_SH1106_Print(display, &textbuf[0]);

		/* Draw delay (in ms) and # of iterations */
		snprintf((char *)&textbuf[0], sizeof textbuf, "(%lu ms, %d)",
		         delay_ms, FRAME_ITER);
		MSIM_SH1106_Clean(display);
		MSIM_SH1106_SetPage(display, 1);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_Send(display);
		MSIM_SH1106_Print(display, &textbuf[0]);

		ms = 0;

		/* Let the framebuffer hang for some time */
		_delay_ms(3000);
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
	//PORTC ^= (1<<PC1);
}
