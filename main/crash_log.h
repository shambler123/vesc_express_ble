/*
	Copyright 2026 Markus Stadtmann

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Keeps the essentials of the last panic in RTC memory, since the ESP
 * console is disabled in this firmware and the panic output is lost.
 * Wraps esp_panic_handler through the linker (-Wl,--wrap=esp_panic_handler).
 */

#ifndef MAIN_CRASH_LOG_H_
#define MAIN_CRASH_LOG_H_

#include <stdint.h>
#include <stdbool.h>

#define CRASH_LOG_STR_LEN	40

typedef struct {
	bool valid;			// A crash record exists from the previous run
	int reset_reason;	// esp_reset_reason() of this boot
	uint32_t pc;		// mepc
	uint32_t ra;		// return address
	uint32_t sp;
	uint32_t mcause;
	uint32_t mtval;
	char reason[CRASH_LOG_STR_LEN];
	char description[CRASH_LOG_STR_LEN];
	char task[CRASH_LOG_STR_LEN];
} crash_log_t;

// Call once early at boot, takes the snapshot and clears the RTC record
void crash_log_init(void);
const crash_log_t *crash_log_get(void);

#endif /* MAIN_CRASH_LOG_H_ */
