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
#define F_CPU			12000000UL	/* CPU 12 MHz */

#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/*
 * Entry point of the Xling firmware.
 *
 * An initial configuration of the microcontroller will be performed. Tasks and
 * FreeRTOS scheduler will be created and started here.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "mcusim/drivers/avr-gcc/avr/display/sh1106/sh1106.h"
#include "mcusim/drivers/avr-gcc/avr/display/sh1106/sh1106_graphics.h"

#include "xling/tasks.h"

/* Local macros. */
#define SET_BIT(byte, bit)	((byte) |= (1U << (bit)))
#define CLEAR_BIT(byte, bit)	((byte) &= (uint8_t) ~(1U << (bit)))
#define IS_SET(byte, bit)	(((byte) & (1U << (bit))) >> (bit))

/* Buttons pins */
#define BTN1			PD3
#define BTN2			PD2
#define BTN3			PB4

/* Entry point. */
int
main(void)
{
	XG_TaskArgs_t display_args;
	XG_TaskArgs_t batmon_args;
	QueueHandle_t display_q;
	BaseType_t status;

	/* Configure PORTC pins as output. */
	DDRC = 0xFF;
	/* Configure PORTA pins as input. */
	DDRA = 0x00;
	PORTA = 0x00;

	/* Prepare a display task queue. */
	display_q = xQueueCreate(3, sizeof(XG_Msg_t));

	/* Prepare task arguments. */
	display_args.display_q = display_q;
	batmon_args.display_q = display_q;

	do {
		if (display_q == NULL) {
			/* The display queue couldn't be created. */
			break;
		}

		/* Create the display task. */
		status = xTaskCreate(XG_DisplayTask, "display", 1024,
		                     &display_args, 3, NULL);
		if (status != pdPASS) {
			/* Display task couldn't be created. */
			break;
		}

		/* Create the battery monitor task. */
		status = xTaskCreate(XG_BatteryMonitorTask, "batmon", 1024,
		                     &batmon_args, 1, NULL);
		if (status != pdPASS) {
			/* Battery monitor task couldn't be created. */
			break;
		}

		/* Start the FreeRTOS scheduler. */
		vTaskStartScheduler();

		/* Stop the display driver. */
		MSIM_SH1106__drvStop();
	} while (0);

	/*
	 * If all is well then main() will never reach here as the scheduler
	 * will now be running the tasks. If main() does reach here then it is
	 * likely that there was insufficient heap memory available for the
	 * display queue or tasks to be created.
	 */
	while(1);

	return 0;
}

/*
 * Disable interrupts and sit still in case of a stask overflow.
 *
 * NOTE: This hook will be optimized out by the compiler unless you have
 * a "configCHECK_FOR_STACK_OVERFLOW" option enabled in the FreeRTOSConfig.h.
 */
void
vApplicationStackOverflowHook(TaskHandle_t *pxTask, signed char *pcTaskName)
{
	taskDISABLE_INTERRUPTS();
	while(1);
}
