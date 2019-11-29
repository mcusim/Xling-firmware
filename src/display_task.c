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
#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/*
 * A display task.
 *
 * It is supposed to be the only task which updates the display.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "mcusim/drivers/avr-gcc/avr/display/sh1106/sh1106.h"
#include "mcusim/drivers/avr-gcc/avr/display/sh1106/sh1106_graphics.h"

#include "xling/tasks.h"
#include "xling/graphics/xling.h"
#include "xling/graphics/luci.h"
#include "xling/graphics/luci_walking_01.h"
#include "xling/graphics/luci_walking_02.h"
#include "xling/graphics/luci_walking_03.h"
#include "xling/graphics/luci_walking_04.h"

/* Local macros. */
#define SET_BIT(byte, bit)	((byte) |= (1U << (bit)))
#define CLEAR_BIT(byte, bit)	((byte) &= (uint8_t) ~(1U << (bit)))

/* OLED pins */
#define OLED_DC			PC5
#define OLED_RST		PC6
#define OLED_CS			PC7

/* OLED SPI pins */
#define OLED_MOSI		PB5
#define OLED_MISO		PB6
#define OLED_SCK		PB7
/* END Local macros. */

/* Local variables. */
static const MSIM_SH1106DrvConf_t _driver_conf = {
	.port_spi = &PORTB,
	.ddr_spi = &DDRB,
	.mosi = OLED_MOSI,
	.miso = OLED_MISO,
	.sck = OLED_SCK,
};
static const MSIM_SH1106Conf_t _display_conf = {
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
static volatile TaskHandle_t _display_task;

/* Local functions declarations. */
static void init_timer3(void);

void
XG_DisplayTask(void *arg)
{
	const XG_TaskArgs_t * const args = (XG_TaskArgs_t *) arg;
	const uint8_t *ptr;
	MSIM_SH1106_t * const display = MSIM_SH1106_Init(&_display_conf);
	uint8_t frame_id = 1, change_frame = 1, col = 0;
	uint16_t bat_lvl = 0, bat_stat = 0;
	char textbuf[32];
	TickType_t delay;
	BaseType_t status;
	XG_Msg_t msg;

	taskENTER_CRITICAL();
	{
		/* Keep CS high. Display isn't selected by default. */
		SET_BIT(PORTC, OLED_CS);

		/* Power On the display. */
		CLEAR_BIT(PORTC, OLED_RST);
		_delay_ms(1);
		SET_BIT(PORTC, OLED_RST);
		_delay_ms(10);

		/* Start the driver for SH1106-based displays. */
		MSIM_SH1106__drvStart(&_driver_conf);

		/* Setup Timer 3 to resume the display task. */
		init_timer3();
	}
	taskEXIT_CRITICAL();

	/* Obtain a handle of the display task. */
	_display_task = xTaskGetHandle("display");

	/* Setup an OLED display. */
	MSIM_SH1106_DisplayOff(display);
	MSIM_SH1106_SetContrast(display, 0x75);
	MSIM_SH1106_DisplayNormal(display);
	MSIM_SH1106_SetScanDirection(display, 0);
	MSIM_SH1106_DisplayOn(display);
	MSIM_SH1106_bufSend(display);

	/* Clear the display. */
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

	/* Task loop */
	while (1) {
		/* Remember a moment in time. */
		delay = xTaskGetTickCount();

		/* Attempt to receive a message from the queue. */
		status = xQueueReceive(args->display_q, &msg, 0);

		if (status == pdPASS) {
			/*
			 * Data have successfully been received from the queue.
			 */
			switch (msg.type) {
			case XG_MSG_BATLVL:
				bat_lvl = msg.value;
				break;
			case XG_MSG_BATSTATPIN:
				bat_stat = msg.value;
				break;
			default:
				/* Ignore other messages silently. */
				break;
			}
		}

		/*
		 * Draw the next animation frame.
		 */

		/* Choose an animation frame. */
		if (change_frame == 1) {
			/* Clean a part of the display. */
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

			/* Update a sprite position. */
			col = (uint8_t)((col >= 130) ? 5 : (col + 5));

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

			/* Draw the frame. */
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
			change_frame = 0;
		} else {
			change_frame = 1;
		}

		/*
		 * Calculate a delay to receive a message from the queue and
		 * draw an animation frame.
		 */
		delay = xTaskGetTickCount() - delay;

		/* Draw the delay. */
		snprintf(&textbuf[0], sizeof textbuf, "%u ms", delay);

		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, change_frame);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);
		MSIM_SH1106_Print(display, &textbuf[0]);

		/* Draw the battery level. */
		snprintf(&textbuf[0], sizeof textbuf, "%d, %d",
		         bat_lvl, bat_stat);

		MSIM_SH1106_bufClear(display);
		MSIM_SH1106_SetPage(display, 2);
		MSIM_SH1106_SetColumn(display, 2);
		MSIM_SH1106_bufSend(display);
		MSIM_SH1106_Print(display, &textbuf[0]);

		/* Let the task to suspend itself until the next wake. */
		vTaskSuspend(NULL);
	}

	/*
	 * The task must be deleted before reaching the end of its
	 * implementating function.
	 *
	 * NOTE: Shouldn't reach this point!
	 */
	vTaskDelete(NULL);
}

static void
init_timer3(void)
{
	/*
	 * Configure Timer 3 to generate a 24 Hz frequency at the Output Compare
	 * Channel A. It can be achieved by dividing the MCU clock frequency
	 * by 8 to get 1.5 MHz, and by 62,500 to get the desired one:
	 *
	 *     12,000,000 / 8 = 1,500,000 MHz
	 *     1,500,000 / 62,500 = 24 Hz
	 */

	/* CTC mode, WDM33:0 = 4 */
	CLEAR_BIT(TCCR3B, WGM33);
	SET_BIT(TCCR3B, WGM32);
	CLEAR_BIT(TCCR3A, WGM31);
	CLEAR_BIT(TCCR3A, WGM30);

	/* Divide the timer frequency by 62,500 to get 24 Hz. */
	TCNT3 = 0x00;
	OCR3A = 62499;

	/* Enable Output-Compare interrupt (Channel A). */
	SET_BIT(TIMSK3, OCIE3A);

	/* Start timer, prescaler to 8: CS32:0 = 2. */
	CLEAR_BIT(TCCR3B, CS32);
	SET_BIT(TCCR3B, CS31);
	CLEAR_BIT(TCCR3B, CS30);
}

/*
 * A Timer 3 Compare-Match (Channel A) ISR which is used to resume the display
 * updating task.
 */
ISR(TIMER3_COMPA_vect)
{
	/* Resume the suspended task. */
	const BaseType_t yield = xTaskResumeFromISR(_display_task);

	if (yield == pdTRUE) {
		/*
		 * A context switch should now be performed so the ISR returns
		 * directly to the resumed task. This is because the resumed
		 * task had a priority that was equal to or higher than the task
		 * that is currently in the Running state.
		 */
		portYIELD_FROM_ISR();
	}
}
