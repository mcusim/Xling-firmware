/*-
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This file is part of a firmware for Xling, a tamagotchi-like toy.
 *
 * Copyright (c) 2019 Dmitry Salychev
 *
 * Xling firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Xling firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef XG_LUCI_WALKING02_H_
#define XG_LUCI_WALKING02_H_ 1

/*
 * The second frame of the walking animation of Luci, 59x64 px.
 */

#include <stdint.h>
#include <avr/pgmspace.h>

static const uint8_t PROGMEM luci_walking_02[] = {
    0x00,
    0x00,
    0x00,
    0x02,
    0x06,
    0x0e,
    0x0e,
    0x1e,
    0x3e,
    0x7e,
    0xde,
    0x1a,
    0x3a,
    0xf2,
    0xe0,
    0xc0,
    0x80,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x20,
    0xe0,
    0xc0,
    0x80,
    0x00,
    0x84,
    0xdc,
    0xf8,
    0xf0,
    0xe0,
    0x60,
    0x20,
    0x30,
    0x30,
    0xb0,
    0xf0,
    0xf0,
    0xf8,
    0xf8,
    0xf8,
    0xf8,
    0xf8,
    0xf8,
    0xf8,
    0xf0,
    0xf0,
    0xe0,
    0xc0,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x03,
    0x0f,
    0x3f,
    0xfc,
    0xe0,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x7f,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xf1,
    0xe0,
    0xe0,
    0xe0,
    0xe0,
    0xe0,
    0xe1,
    0xf3,
    0xf3,
    0xf9,
    0xff,
    0xff,
    0xff,
    0xff,
    0xbf,
    0xbf,
    0x9f,
    0x8f,
    0x07,
    0x03,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xff,
    0xff,
    0xf8,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x03,
    0x07,
    0x0f,
    0x0f,
    0x1f,
    0x7f,
    0xff,
    0xff,
    0xff,
    0xff,
    0x8f,
    0x0f,
    0x0f,
    0x1f,
    0x3f,
    0x07,
    0x0f,
    0x1f,
    0x03,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xff,
    0xff,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xf0,
    0xe0,
    0xe0,
    0xf0,
    0x38,
    0x38,
    0x1c,
    0x1c,
    0x0c,
    0x0e,
    0x0e,
    0x0e,
    0x0e,
    0x7e,
    0x3e,
    0x7e,
    0x3c,
    0x70,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x3f,
    0xfe,
    0xc0,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0xc0,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xf8,
    0xf0,
    0x70,
    0x38,
    0x38,
    0x1c,
    0x1c,
    0x0e,
    0x06,
    0x07,
    0x3f,
    0x1f,
    0x3f,
    0x1e,
    0x38,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x07,
    0x1c,
    0x38,
    0x70,
    0x60,
    0xe0,
    0xc0,
    0xc0,
    0xc0,
    0xc0,
    0xf0,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0xff,
    0x7f,
    0x1f,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x80,
    0x81,
    0xc3,
    0xc3,
    0xff,
    0xff,
    0x7f,
    0x7f,
    0x31,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x03,
    0x07,
    0xff,
    0xff,
    0xff,
    0xf7,
    0xe0,
    0xe0,
    0xe0,
    0xe0,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
};

#endif /* XG_LUCI_WALKING02_H_ */
