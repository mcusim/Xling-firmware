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
 *
 * There are several options available to configure the driver:
 *
 * - configMSIM_DRV_DISPLAY_SH1106_DNUM
 *
 *     Maximum number of devices supported by the driver.
 *
 *     Memory for all of the display control blocks is statically allocated
 *     inside the driver, so select a minumum required number of the devices
 *     to support to minimize a RAM footprint.
 *
 * - configMSIM_DRV_DISPLAY_SH1106_BUFSZ
 *
 *     Defines size of the display buffer (in bytes) which is used to
 *     accumulate display commands and framebuffer data.
 *
 * - configMSIM_DRV_DISPLAY_SH1106_TWIBB
 *
 *     If defined, it tells the driver that a display is connected
 *     via a software implemented TWI.
 *
 * - configMSIM_DRV_DISPLAY_SH1106_TWI
 *
 *     If defined, it tells the driver that a display is connected
 *     via a hardware implemented TWI.
 *
 * - configMSIM_DRV_DISPLAY_SH1106_SPI3
 *
 *     If defined, it tells the driver that a display is connected
 *     via a 3-wire SPI.
 *
 * - configMSIM_DRV_DISPLAY_SH1106_SPI4
 *
 *     If defined, it tells the driver that a display is connected
 *     via a 4-wire SPI.
 */
#ifndef MSIM_DRV_DISPLAY_SH1106_H_
#define MSIM_DRV_DISPLAY_SH1106_H_ 1

#include <stdint.h>
#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Return codes for the driver functions */
#define MSIM_SH1106_RC_OK			0
#define MSIM_SH1106_RC_NULLPTR			75
#define MSIM_SH1106_RC_DEVICENOTREADY		76
#define MSIM_SH1106_RC_DRIVERNOTREADY		77

/* The driver init/configuration structure */
struct MSIM_SH1106DriverConf;

/* A display init/configuration structure */
struct MSIM_SH1106DisplayConf;

/* An opaque display control block (DCB) */
struct MSIM_SH1106;

/*
 * TWI control byte.
 *
 * It is a first byte within the command word transmitted to the display over
 * two-wire interface, second one is a data byte.
 *
 *     Co DC 0 0 0 0 0 0, where
 *
 *     Co - the continuation bit,
 *     DC - the display command (or graphics RAM) operation bit, i.e.
 *
 *     Co = 0 - the last control byte, only data bytes to follow,
 *     Co = 1 - the next two bytes are a data byte and another control byte,
 *     DC = 0 - the data byte is for command operation,
 *     DC = 1 - the data byte is for RAM operation.
 */
enum MSIM_SH1106_TWICB {
	CB_LASTCMD = 0x00,	/* 0 0 x x  x x x x */
	CB_LASTRAM = 0x40,	/* 0 1 x x  x x x x */
	CB_CMD = 0x80,		/* 1 0 x x  x x x x */
	CB_RAM = 0xC0,		/* 1 1 x x  x x x x */
};

/*
 * -----------------------------------------------------------------------------
 * An SH1106-based display can be connected by a software implemented
 * (bit banging) two-wire interface (I2C).
 * -----------------------------------------------------------------------------
 */
#if defined(configMSIM_DRV_DISPLAY_SH1106_TWIBB)

struct MSIM_SH1106DisplayConf {
	volatile uint8_t *twi_port;	/* I/O port (PORTC, PORTD, etc.) */
	volatile uint8_t *twi_ddr;	/* DDR of the I/O port (DDRC, etc.) */
	uint8_t sda;			/* SDA pin number (PC0, PC1, etc.) */
	uint8_t scl;			/* SCL pin number (PC2, PC3, etc.) */
	uint8_t twi_addr;		/* TWI address of the display */
};

/*
 * Special utility functions
 *
 * Note: they aren't used in case of software implemented TWI.
 */
#define MSIM_SH1106__drvStart()
#define MSIM_SH1106__drvStop()

/* Display utility functions */
struct MSIM_SH1106 *	MSIM_SH1106_Init(struct MSIM_SH1106DisplayConf *);
void			MSIM_SH1106_Free(struct MSIM_SH1106 *);

/*
 * -----------------------------------------------------------------------------
 * An SH1106-based display can be connected by a hardware implemented two-wire
 * interface (I2C).
 * -----------------------------------------------------------------------------
 */
#elif defined(configMSIM_DRV_DISPLAY_SH1106_TWI)

struct MSIM_SH1106DriverConf {
	uint32_t cpu_f;			/* CPU frequency, in Hz */
	uint32_t twi_f;			/* TWI (SCL) frequency, in Hz */
};

struct MSIM_SH1106DisplayConf {
	uint8_t twi_addr;		/* TWI address of the display */
};

/* Special utility functions */
int	MSIM_SH1106__drvStart(struct MSIM_SH1106DriverConf *);
int	MSIM_SH1106__drvStop(void);

/* Display utility functions */
struct MSIM_SH1106 *	MSIM_SH1106_Init(struct MSIM_SH1106DisplayConf *);
void			MSIM_SH1106_Free(struct MSIM_SH1106 *);

/*
 * -----------------------------------------------------------------------------
 * An SH1106-based display can be connected by a 3-wire SPI.
 * -----------------------------------------------------------------------------
 */
#elif defined(configMSIM_DRV_DISPLAY_SH1106_SPI3)

/*
 * -----------------------------------------------------------------------------
 * An SH1106-based display can be connected by a 4-wire SPI.
 * -----------------------------------------------------------------------------
 */
#elif defined(configMSIM_DRV_DISPLAY_SH1106_SPI4)

#else
#error "Please, let the driver know how the SH1106-based display is connected."
#endif

/* Buffer utility functions */
int	MSIM_SH1106_Clean(struct MSIM_SH1106 *);
int	MSIM_SH1106_Send(struct MSIM_SH1106 *);
int	MSIM_SH1106_Write(struct MSIM_SH1106 *, const uint8_t);
int	MSIM_SH1106_WriteData(struct MSIM_SH1106 *, const uint8_t *, size_t);
int	MSIM_SH1106_WriteData_PF(struct MSIM_SH1106 *, uint_farptr_t, size_t);

/* Display commands */
int	MSIM_SH1106_SetPage(struct MSIM_SH1106 *, uint8_t page);
int	MSIM_SH1106_SetColumn(struct MSIM_SH1106 *, uint8_t col);
int	MSIM_SH1106_DisplayOn(struct MSIM_SH1106 *);
int	MSIM_SH1106_DisplayOff(struct MSIM_SH1106 *);
int	MSIM_SH1106_SetContrast(struct MSIM_SH1106 *, uint8_t val);
int	MSIM_SH1106_DisplayNormal(struct MSIM_SH1106 *);
int	MSIM_SH1106_DisplayInvert(struct MSIM_SH1106 *);
int	MSIM_SH1106_SetStartLine(struct MSIM_SH1106 *, uint8_t line);
int	MSIM_SH1106_SetScanDirection(struct MSIM_SH1106 *, uint8_t reverse);

#ifdef __cplusplus
}
#endif

#endif /* MSIM_DRV_DISPLAY_SH1106_H_ */
