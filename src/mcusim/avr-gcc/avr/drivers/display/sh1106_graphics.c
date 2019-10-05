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
 * Graphics for SH1106-based displays.
 */
#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "mcusim/avr-gcc/avr/drivers/display/sh1106_graphics.h"

#define PM PROGMEM

/* Symbols 6x8 pixels */
static const uint8_t PM zero[] =	{ 0x00, 0x3C, 0x42, 0x42, 0x3C, 0x00 };
static const uint8_t PM one[] =		{ 0x00, 0x00, 0x04, 0x7E, 0x00, 0x00 };
static const uint8_t PM two[] =		{ 0x00, 0x44, 0x62, 0x52, 0x4C, 0x00 };
static const uint8_t PM three[] =	{ 0x00, 0x24, 0x42, 0x4A, 0x34, 0x00 };
static const uint8_t PM four[] =	{ 0x00, 0x18, 0x14, 0x7E, 0x10, 0x00 };
static const uint8_t PM five[] =	{ 0x00, 0x2E, 0x4A, 0x4A, 0x32, 0x00 };
static const uint8_t PM six[] =		{ 0x00, 0x3C, 0x4A, 0x4A, 0x30, 0x00 };
static const uint8_t PM seven[] =	{ 0x00, 0x02, 0x62, 0x1A, 0x06, 0x00 };
static const uint8_t PM eight[] =	{ 0x00, 0x34, 0x4A, 0x4A, 0x34, 0x00 };
static const uint8_t PM nine[] =	{ 0x00, 0x0C, 0x52, 0x52, 0x3C, 0x00 };
static const uint8_t PM dot[] =		{ 0x00, 0x00, 0x40, 0x00, 0x00, 0x00 };

static const uint8_t PM open_brace[] =	{ 0x00, 0x00, 0x3c, 0x42, 0x00, 0x00 };
static const uint8_t PM close_brace[] = { 0x00, 0x00, 0x42, 0x3c, 0x00, 0x00 };
static const uint8_t PM F[] =		{ 0x00, 0x7e, 0x0a, 0x02, 0x00, 0x00 };
static const uint8_t PM P[] =		{ 0x00, 0x7e, 0x0a, 0x0e, 0x00, 0x00 };
static const uint8_t PM S[] =		{ 0x00, 0x24, 0x4a, 0x52, 0x24, 0x00 };
static const uint8_t PM m[] =		{ 0x00, 0x78, 0x08, 0x70, 0x08, 0x70 };
static const uint8_t PM s[] =		{ 0x00, 0x48, 0x54, 0x24, 0x00, 0x00 };
static const uint8_t PM space[] =	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t PM comma[] =	{ 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00 };

static void	write_mem(struct MSIM_SH1106 *, const uint8_t *, size_t);

/* Prints text at the current line of the display. */
int
MSIM_SH1106_Print(struct MSIM_SH1106 *dev, const char *text)
{
	const size_t len = strlen(text);

	MSIM_SH1106_Clean(dev);
	MSIM_SH1106_Write(dev, CB_LASTRAM);

	for (uint32_t i = 0; i < len; i++) {
		switch (text[i]) {
		case '0':
			write_mem(dev, zero, sizeof zero);
			break;
		case '1':
			write_mem(dev, one, sizeof one);
			break;
		case '2':
			write_mem(dev, two, sizeof two);
			break;
		case '3':
			write_mem(dev, three, sizeof three);
			break;
		case '4':
			write_mem(dev, four, sizeof four);
			break;
		case '5':
			write_mem(dev, five, sizeof five);
			break;
		case '6':
			write_mem(dev, six, sizeof six);
			break;
		case '7':
			write_mem(dev, seven, sizeof seven);
			break;
		case '8':
			write_mem(dev, eight, sizeof eight);
			break;
		case '9':
			write_mem(dev, nine, sizeof nine);
			break;
		case '.':
			write_mem(dev, dot, sizeof dot);
			break;
		case '(':
			write_mem(dev, open_brace, sizeof open_brace);
			break;
		case ')':
			write_mem(dev, close_brace, sizeof close_brace);
			break;
		case 'F':
			write_mem(dev, F, sizeof F);
			break;
		case 'P':
			write_mem(dev, P, sizeof P);
			break;
		case 'S':
			write_mem(dev, S, sizeof S);
			break;
		case 'm':
			write_mem(dev, m, sizeof m);
			break;
		case 's':
			write_mem(dev, s, sizeof s);
			break;
		case ' ':
			write_mem(dev, space, sizeof space);
			break;
		case ',':
			write_mem(dev, comma, sizeof comma);
			break;
		default:
			break;
		}
	}
	MSIM_SH1106_Send(dev);

	return 0;
}

static void write_mem(struct MSIM_SH1106 *dev, const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		MSIM_SH1106_Write(dev, pgm_read_byte(&data[i]));
	}
}
