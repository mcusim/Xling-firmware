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

/* A task initialization function (helper data type only). */
typedef int (*task_initfunc_t)(XG_TaskArgs_t *, UBaseType_t, TaskHandle_t *);

/* A task initialization block (helper data type only). */
typedef struct task_initblk_t {
	task_initfunc_t initf;
	XG_TaskArgs_t *arg;
	UBaseType_t priority;
} task_initblk_t;

/* Local macros. */
#define SET_BIT(byte, bit)	((byte) |= (1U << (bit)))
#define CLEAR_BIT(byte, bit)	((byte) &= (uint8_t) ~(1U << (bit)))
#define IS_SET(byte, bit)	(((byte) & (1U << (bit))) >> (bit))

/* Buttons pins */
#define BTN1			PD3
#define BTN2			PD2
#define BTN3			PB4

/* Local variables. */
static XG_TaskArgs_t _display_args;
static XG_TaskArgs_t _batmon_args;
static XG_TaskArgs_t _slpmod_args;

static const task_initblk_t _init_blocks[] = {
	{ .initf=XG_InitDisplayTask, .arg=&_display_args, .priority=3 },
	{ .initf=XG_InitBatteryMonitorTask, .arg=&_batmon_args, .priority=1 },
	{ .initf=XG_InitSleepModeTask, .arg=&_slpmod_args, .priority=1 },
};
const size_t _tib_num = sizeof(_init_blocks) / sizeof(_init_blocks[0]);
/* END Local variables. */

/* Entry point. */
int
main(void)
{
	const task_initblk_t *tib;
	int rc = 0;

	/* Configure PORTC pins as output. */
	DDRC = 0xFF;
	/* Configure PORTA pins as input. */
	DDRA = 0x00;
	PORTA = 0x00;

	/* Initialize tasks arguments. */
	_display_args.display_q = xQueueCreate(3, sizeof(XG_Msg_t)),
	_display_args.batmon_q = xQueueCreate(3, sizeof(XG_Msg_t)),
	_display_args.slpmod_q = xQueueCreate(3, sizeof(XG_Msg_t)),
	_batmon_args = _display_args;
	_slpmod_args = _display_args;

	/* Create and initialize Xling tasks. */
	for (size_t i = 0; i < _tib_num; i++) {
		tib = &_init_blocks[i];

		/* Initialize a task. */
		rc = tib->initf(tib->arg, tib->priority, NULL);

		if (rc != 0) {
			/* Task couldn't be created/initialized successfully. */
			break;
		}
	}

	/* Start the FreeRTOS scheduler. */
	if (rc == 0) {
		vTaskStartScheduler();
	}

	/* Stop the display driver. */
	MSIM_SH1106__drvStop();

	/*
	 * If all is well then main() will never reach here as the scheduler
	 * will now be running the tasks. If main() does reach here then it is
	 * likely that there was insufficient heap memory available for the
	 * display queue or tasks to be created.
	 */
	while (1);

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
