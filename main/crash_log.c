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

#include "crash_log.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_system.h"
#include "esp_private/panic_internal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_IDF_TARGET_ARCH_RISCV
#include "riscv/rvruntime-frames.h"
#endif

#define CRASH_MAGIC		0xC7A5410C

typedef struct {
	uint32_t magic;
	uint32_t pc;
	uint32_t ra;
	uint32_t sp;
	uint32_t mcause;
	uint32_t mtval;
	char reason[CRASH_LOG_STR_LEN];
	char description[CRASH_LOG_STR_LEN];
	char task[CRASH_LOG_STR_LEN];
	char details[CRASH_LOG_DETAILS_LEN];
	uint32_t uptime_ms;
} crash_rtc_t;

static RTC_NOINIT_ATTR crash_rtc_t m_rtc;
static crash_log_t m_log;

static void copy_str(char *dst, const char *src) {
	if (src) {
		strncpy(dst, src, CRASH_LOG_STR_LEN - 1);
		dst[CRASH_LOG_STR_LEN - 1] = '\0';
	} else {
		dst[0] = '\0';
	}
}

void __real_esp_panic_handler(panic_info_t *info);

// Set by esp_system_abort() before the panic handler runs (esp_system/panic.c)
extern bool g_panic_abort;
extern char *g_panic_abort_details;

// Runs in panic context: no allocation, no locks, keep it short.
void __wrap_esp_panic_handler(panic_info_t *info) {
	m_rtc.magic = CRASH_MAGIC;
	m_rtc.pc = (uint32_t)info->addr;
	m_rtc.ra = 0;
	m_rtc.sp = 0;
	m_rtc.mcause = 0;
	m_rtc.mtval = 0;
#if CONFIG_IDF_TARGET_ARCH_RISCV
	if (info->frame) {
		const RvExcFrame *f = (const RvExcFrame *)info->frame;
		m_rtc.pc = f->mepc;
		m_rtc.ra = f->ra;
		m_rtc.sp = f->sp;
		m_rtc.mcause = f->mcause;
		m_rtc.mtval = f->mtval;
	}
#endif
	copy_str(m_rtc.reason, info->reason);
	copy_str(m_rtc.description, info->description);

	TaskHandle_t t = xTaskGetCurrentTaskHandleForCore(info->core);
	copy_str(m_rtc.task, t ? pcTaskGetName(t) : NULL);

	// abort() and failed asserts pass their message here
	m_rtc.details[0] = '\0';
	if (g_panic_abort && g_panic_abort_details) {
		strncpy(m_rtc.details, g_panic_abort_details, CRASH_LOG_DETAILS_LEN - 1);
		m_rtc.details[CRASH_LOG_DETAILS_LEN - 1] = '\0';
	}
	m_rtc.uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

	__real_esp_panic_handler(info);
}

void crash_log_init(void) {
	memset(&m_log, 0, sizeof(m_log));
	m_log.reset_reason = (int)esp_reset_reason();

	if (m_rtc.magic == CRASH_MAGIC) {
		m_log.valid = true;
		m_log.pc = m_rtc.pc;
		m_log.ra = m_rtc.ra;
		m_log.sp = m_rtc.sp;
		m_log.mcause = m_rtc.mcause;
		m_log.mtval = m_rtc.mtval;
		memcpy(m_log.reason, m_rtc.reason, CRASH_LOG_STR_LEN);
		memcpy(m_log.description, m_rtc.description, CRASH_LOG_STR_LEN);
		memcpy(m_log.task, m_rtc.task, CRASH_LOG_STR_LEN);
		memcpy(m_log.details, m_rtc.details, CRASH_LOG_DETAILS_LEN);
		m_log.details[CRASH_LOG_DETAILS_LEN - 1] = '\0';
		m_log.uptime_ms = m_rtc.uptime_ms;
		m_log.reason[CRASH_LOG_STR_LEN - 1] = '\0';
		m_log.description[CRASH_LOG_STR_LEN - 1] = '\0';
		m_log.task[CRASH_LOG_STR_LEN - 1] = '\0';
	}

	memset(&m_rtc, 0, sizeof(m_rtc));
}

const crash_log_t *crash_log_get(void) {
	return &m_log;
}
