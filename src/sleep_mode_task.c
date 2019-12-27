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
#include <avr/io.h>

/*
 * This task is supposed to monitor the device activity during a selected
 * period of time and suspend all of the tasks if the device hasn't been used.
 */

/* FreeRTOS headers. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Xling headers. */
#include "xling/tasks.h"

/* Local variables. */
static volatile TaskHandle_t _task_handle;

/* Local functions. */
static void sleepmod_task(void *) __attribute__((noreturn));

int
XG_InitSleepModeTask(XG_TaskArgs_t *arg, UBaseType_t priority)
{
	BaseType_t stat;
	int rc = 0;

	/* Create the sleep mode task. */
	stat = xTaskCreate(sleepmod_task, "Sleep Mode Task",
	                   configMINIMAL_STACK_SIZE, &display_args, priority,
	                   &_task_handle);

	/* Sleep mode task couldn't be created. */
	if (stat != pdPASS) {
		rc = 1;
	}

	return rc;
}

static void
sleepmod_task(void *arg)
{
	/* Task loop */
	while (1) {
		/* Be silent for some time. */
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	/*
	 * The task must be deleted before reaching the end of its
	 * implementating function.
	 *
	 * NOTE: This point shouldn't be reached.
	 */
	vTaskDelete(NULL);
}
