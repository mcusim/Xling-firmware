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
 * Low-level driver for SH1106-based displays.
 */
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "mcusim/avr-gcc/avr/drivers/display/sh1106/sh1106.h"

/*
 * Driver should be aware of the display connected via one of the interfaces
 * mentioned below.
 */
#if !defined(configMSIM_DRV_DISPLAY_SH1106_TWIBB) && \
    !defined(configMSIM_DRV_DISPLAY_SH1106_TWI) && \
    !defined(configMSIM_DRV_DISPLAY_SH1106_SPI4)
#error "Please, let the driver know how the SH1106-based display is connected."
#endif
