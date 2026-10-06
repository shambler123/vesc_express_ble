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

#include "bms_ble.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "bms.h"
#include "comm_ble.h"
#include "commands.h"
#include "utils.h"
#include "lispif.h"
#include "lispbm.h"
#include "lbm_vesc_utils.h"

#if CONFIG_BT_BLUEDROID_ENABLED

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"

// Settings
#define BMS_GATTC_APP_ID		1
#define RX_BUF_SIZE				320
#define CMD_TIMEOUT_MS			1500
#define POLL_INTERVAL_MS		1000
#define TARGET_SCAN_MS			10000	// Scan this long for the target before trying a direct connect
#define CONNECT_TIMEOUT_MS		20000	// Bluedroid connect timeout is CONFIG_BT_BLE_ESTAB_LINK_CONN_TOUT (5 s) + discovery
#define RECONNECT_DELAY_MS		3000	// First retry, doubles on every failure up to RECONNECT_DELAY_MAX_MS
#define RECONNECT_DELAY_MAX_MS	60000
#define RECONNECT_DELAY_PHONE_MS 10000	// Minimum retry delay while VESC Tool is connected
#define MAX_POLL_FAILS			5
#define TASK_PERIOD_MS			50

// Scan policy. User scans (device list) listen 40 % of the time when nothing
// is connected and 10 % when the phone or the BMS is attached. The internal
// scan used to find the BMS for a (re)connect is passive and only listens
// 5 % of the time. Bluedroid also uses the last scan parameters while it
// initiates a connection, so this keeps the radio free for WiFi and the
// VESC Tool link.
#define SCAN_INTERVAL_FREE		0x50	// 50 ms
#define SCAN_WINDOW_FREE		0x20	// 20 ms (40 %)
#define SCAN_INTERVAL_BUSY		0xA0	// 100 ms
#define SCAN_WINDOW_BUSY		0x10	// 10 ms (10 %)
#define SCAN_INTERVAL_LOW		0xA0	// 100 ms
#define SCAN_WINDOW_LOW			0x14	// 20 ms (20 %), passive, used while looking for the BMS
#define SCAN_INTERVAL_INIT		0x30	// 30 ms
#define SCAN_WINDOW_INIT		0x30	// 30 ms (100 %), only while the controller initiates the
										// connection, bounded by CONFIG_BT_BLE_ESTAB_LINK_CONN_TOUT

// Connection parameters for the BMS link. Discovery runs at 100 to 150 ms,
// afterwards the link is slowed down to 200 to 300 ms since the BMS is only
// polled once per second. This leaves most of the air-time to the phone.
#define BMS_CONN_INT_DISC_MIN	80		// 100 ms
#define BMS_CONN_INT_DISC_MAX	120		// 150 ms
#define BMS_CONN_INT_MIN		160		// 200 ms
#define BMS_CONN_INT_MAX		240		// 300 ms
#define BMS_CONN_TIMEOUT		600		// 6 s

#define UUID_JBD_SVC			0xFF00
#define UUID_JBD_RX				0xFF01
#define UUID_JBD_TX				0xFF02
#define UUID_DALY_SVC			0xFFF0
#define UUID_DALY_RX			0xFFF1
#define UUID_DALY_TX			0xFFF2
#define UUID_LIPOWER_SVC		0xFFE0
#define UUID_LIPOWER_RXTX		0xFFE1
#define UUID_LIPOWER_ADV		0xAF30

// Nordic UART service as used by the LiTech BMS (128 bit, little endian order)
static const uint8_t UUID_NUS_SVC[16] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
static const uint8_t UUID_NUS_TX[16]  = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E}; // we write here
static const uint8_t UUID_NUS_RX[16]  = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E}; // notifications

#define LITECH_SLAVE_ID			0x01
#define LITECH_LIVE_ADDR		0xD000
#define LITECH_LIVE_COUNT		0x3B
#define LITECH_CELL_UNUSED		0xEE49

#define DALY_VARIANT_D2			1
#define DALY_VARIANT_X81		2
#define DALY_VARIANT_A5			3

#define DBG_LINES				6
#define DBG_LINE_LEN			96
#define DBG(fmt, ...) do { if (m_debug) { dbg_push(fmt, ##__VA_ARGS__); } } while (0)

// Progress marker that survives a reset (RTC memory), used to find the
// stage a crash happened in without a serial console.
#define STAGE_MAGIC				0xB35B1E00
static RTC_NOINIT_ATTR uint32_t m_stage_rtc;
static uint32_t m_last_stage = 0;
static int m_reset_reason = 0;

#define STAGE(n) do { m_stage_rtc = STAGE_MAGIC | (n); } while (0)

// Private variables
static esp_gatt_if_t m_gattc_if = ESP_GATT_IF_NONE;
static SemaphoreHandle_t m_mutex = NULL;
static SemaphoreHandle_t m_rx_sem = NULL;
static TaskHandle_t m_task = NULL;
static volatile bool m_debug = false;
static volatile bool m_update_vesc = true;
static volatile bool m_send_can = true;

// Debug lines are queued here and printed by the supervisor task, since
// commands_printf_lisp needs more stack than the Bluetooth task offers.
static char m_dbg_lines[DBG_LINES][DBG_LINE_LEN];
static volatile int m_dbg_head = 0;
static volatile int m_dbg_tail = 0;

static volatile bms_ble_state_t m_state = BMS_BLE_STATE_DISABLED;

static bool m_target_set = false;
static uint8_t m_target_addr[6];
static uint8_t m_target_addr_type = BLE_ADDR_TYPE_PUBLIC;
static volatile bool m_target_addr_known = false;	// Address type confirmed by a scan or a connection
static bms_ble_type_t m_target_type = BMS_BLE_TYPE_AUTO;
static volatile bool m_auto_reconnect = false;
static uint32_t m_last_attempt = 0;
static uint32_t m_reconnect_delay = RECONNECT_DELAY_MS;
static uint32_t m_connect_start = 0;
static volatile bool m_open_requested = false;
static volatile bool m_target_seen = false;

static struct {
	volatile bool open;
	volatile bool subscribed;
	uint16_t conn_id;
	esp_bd_addr_t bda;
	bms_ble_type_t type;
	uint16_t svc_start;
	uint16_t svc_end;
	uint16_t rx_handle;
	uint16_t tx_handle;
	bool tx_write_nr;
} m_conn;

static volatile bool m_scanning = false;
static volatile bool m_scan_user = false;
static volatile bool m_scan_params_pending = false;
static volatile bool m_open_after_params = false;
static volatile bool m_open_after_scan_stop = false;
static uint32_t m_scan_duration_s = 0;
static bms_ble_scan_entry_t m_scan[BMS_BLE_SCAN_MAX];
static int m_scan_num = 0;

static uint8_t m_rx_buf[RX_BUF_SIZE];
static volatile int m_rx_len = 0;
static volatile bool m_rx_valid = false;
static volatile uint8_t m_rx_expect = 0;		// Expected command / head byte
static volatile uint8_t m_rx_expect2 = 0;		// Protocol specific second byte

// Staging area for multi-frame replies (Daly legacy protocol), filled from
// the BT callback and consumed by the poll task after the reply is complete.
static float m_stage_cells[BMS_BLE_MAX_CELLS];
static float m_stage_temps[BMS_BLE_MAX_TEMPS];
static int m_stage_frames = 0;

static bms_ble_data_t m_data;
static bms_ble_data_t m_work;

// Private functions
static void bms_ble_task(void *arg);
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void update_vesc_bms(void);

static uint32_t now_ms(void) {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void dbg_push(const char *fmt, ...) {
	int next = (m_dbg_head + 1) % DBG_LINES;
	if (next == m_dbg_tail) {
		return; // Full, drop
	}
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(m_dbg_lines[m_dbg_head], DBG_LINE_LEN, fmt, ap);
	va_end(ap);
	m_dbg_head = next;
}

static void dbg_flush(void) {
	while (m_dbg_tail != m_dbg_head) {
		commands_printf_lisp("BMS-BLE: %s", m_dbg_lines[m_dbg_tail]);
		m_dbg_tail = (m_dbg_tail + 1) % DBG_LINES;
	}
}

static uint32_t age_ms(uint32_t t) {
	return now_ms() - t;
}

static uint16_t be16(const uint8_t *p) {
	return ((uint16_t)p[0] << 8) | p[1];
}

static uint16_t crc_modbus(const uint8_t *data, int len) {
	uint16_t crc = 0xFFFF;
	for (int i = 0; i < len; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
		}
	}
	return crc;
}

static const char *type_name(bms_ble_type_t t) {
	switch (t) {
	case BMS_BLE_TYPE_AUTO: return "auto";
	case BMS_BLE_TYPE_JBD: return "jbd";
	case BMS_BLE_TYPE_DALY: return "daly";
	case BMS_BLE_TYPE_LIPOWER: return "lipower";
	case BMS_BLE_TYPE_LITECH: return "litech";
	default: return "unknown";
	}
}

static bool lock(void) {
	return m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

static void unlock(void) {
	xSemaphoreGive(m_mutex);
}

static void reset_conn(void) {
	memset(&m_conn, 0, sizeof(m_conn));
	m_conn.conn_id = 0xFFFF;
	m_conn.type = BMS_BLE_TYPE_UNKNOWN;
}

static bool any_link_busy(void) {
	return comm_ble_is_connected() || m_conn.open;
}

// ---------------------------------------------------------------------------
// Scanning
// ---------------------------------------------------------------------------

typedef enum {
	SCAN_MODE_USER = 0,		// device list, active
	SCAN_MODE_FIND,			// looking for the configured BMS, passive, low duty
	SCAN_MODE_INIT,			// connection initiation, full duty for a few seconds
} scan_mode_t;

static void scan_apply_params(scan_mode_t mode) {
	bool busy = any_link_busy();
	uint16_t interval, window;
	switch (mode) {
	case SCAN_MODE_INIT: interval = SCAN_INTERVAL_INIT; window = SCAN_WINDOW_INIT; break;
	case SCAN_MODE_FIND: interval = SCAN_INTERVAL_LOW; window = SCAN_WINDOW_LOW; break;
	default:
		interval = busy ? SCAN_INTERVAL_BUSY : SCAN_INTERVAL_FREE;
		window = busy ? SCAN_WINDOW_BUSY : SCAN_WINDOW_FREE;
		break;
	}
	esp_ble_scan_params_t p = {
		.scan_type = mode == SCAN_MODE_USER ? BLE_SCAN_TYPE_ACTIVE : BLE_SCAN_TYPE_PASSIVE,
		.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval = interval,
		.scan_window = window,
		.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
	};
	esp_ble_gap_set_scan_params(&p);
}

static bool scan_start_internal(uint32_t seconds, bool user) {
	if (m_gattc_if == ESP_GATT_IF_NONE) {
		return false;
	}

	if (user) {
		m_scan_user = true;
	}

	if (m_scanning || m_scan_params_pending) {
		return true;
	}

	m_scan_duration_s = seconds;
	m_scan_params_pending = true;
	scan_apply_params(user ? SCAN_MODE_USER : SCAN_MODE_FIND);
	return true;
}

static void schedule_retry(void) {
	m_last_attempt = now_ms();
	m_reconnect_delay *= 2;
	if (m_reconnect_delay > RECONNECT_DELAY_MAX_MS) {
		m_reconnect_delay = RECONNECT_DELAY_MAX_MS;
	}
}

static uint32_t retry_delay(void) {
	uint32_t d = m_reconnect_delay;
	if (comm_ble_is_connected() && d < RECONNECT_DELAY_PHONE_MS) {
		d = RECONNECT_DELAY_PHONE_MS;
	}
	return d;
}

static bms_ble_type_t type_from_adv(uint8_t *adv, uint16_t len, const uint8_t *name, uint8_t name_len) {
	if (name && name_len >= 7 && memcmp(name, "BT-BMS-", 7) == 0) {
		return BMS_BLE_TYPE_LITECH;
	}
	if (name && name_len >= 3 && memcmp(name, "DL-", 3) == 0) {
		return BMS_BLE_TYPE_DALY;
	}
	if (name && name_len >= 4 && memcmp(name, "JBD-", 4) == 0) {
		return BMS_BLE_TYPE_JBD;
	}

	// Daly dongles often advertise no service UUID, only a manufacturer id
	{
		uint8_t l = 0;
		uint8_t *m = esp_ble_resolve_adv_data_by_type(adv, len, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &l);
		if (m && l >= 2) {
			uint16_t id = m[0] | (m[1] << 8);
			if (id == 0x102 || id == 0x104 || id == 0x302 || id == 0x303 || id == 0x402) {
				return BMS_BLE_TYPE_DALY;
			}
		}
	}

	const uint8_t types[] = {ESP_BLE_AD_TYPE_16SRV_CMPL, ESP_BLE_AD_TYPE_16SRV_PART};
	for (int t = 0; t < 2; t++) {
		uint8_t l = 0;
		uint8_t *p = esp_ble_resolve_adv_data_by_type(adv, len, types[t], &l);
		if (!p) {
			continue;
		}
		for (int i = 0; i + 1 < l; i += 2) {
			uint16_t uuid = p[i] | (p[i + 1] << 8);
			switch (uuid) {
			case UUID_JBD_SVC: return BMS_BLE_TYPE_JBD;
			case UUID_DALY_SVC: return BMS_BLE_TYPE_DALY;
			case UUID_LIPOWER_SVC:
			case UUID_LIPOWER_ADV: return BMS_BLE_TYPE_LIPOWER;
			default: break;
			}
		}
	}
	return BMS_BLE_TYPE_UNKNOWN;
}

static void scan_result(struct ble_scan_result_evt_param *r) {
	uint16_t adv_len = r->adv_data_len + r->scan_rsp_len;

	uint8_t name_len = 0;
	uint8_t *name = esp_ble_resolve_adv_data_by_type(r->ble_adv, adv_len, ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
	if (!name) {
		name = esp_ble_resolve_adv_data_by_type(r->ble_adv, adv_len, ESP_BLE_AD_TYPE_NAME_SHORT, &name_len);
	}

	bms_ble_type_t type = type_from_adv(r->ble_adv, adv_len, name, name_len);

	if (m_target_set && memcmp(r->bda, m_target_addr, 6) == 0) {
		m_target_addr_type = r->ble_addr_type;
		m_target_addr_known = true;
	}

	// Connect flow: looking for the configured target
	if (m_state == BMS_BLE_STATE_CONNECTING && !m_target_seen && !m_open_requested &&
			memcmp(r->bda, m_target_addr, 6) == 0) {
		m_target_seen = true;
		STAGE(3);
		DBG("Target found (rssi %d, addr type %d)", r->rssi, r->ble_addr_type);
		m_open_after_scan_stop = true;
		esp_ble_gap_stop_scanning();
	}

	if (!m_scan_user) {
		return;
	}

	// Only list devices that look like a BMS, or that have a name
	if (type == BMS_BLE_TYPE_UNKNOWN && !name) {
		return;
	}

	if (!lock()) {
		return;
	}

	int idx = -1;
	for (int i = 0; i < m_scan_num; i++) {
		if (memcmp(m_scan[i].addr, r->bda, 6) == 0) {
			idx = i;
			break;
		}
	}

	if (idx < 0) {
		if (m_scan_num < BMS_BLE_SCAN_MAX) {
			idx = m_scan_num++;
			memset(&m_scan[idx], 0, sizeof(m_scan[idx]));
			memcpy(m_scan[idx].addr, r->bda, 6);
		} else {
			// Replace the oldest unknown entry
			uint32_t oldest = 0xFFFFFFFF;
			for (int i = 0; i < m_scan_num; i++) {
				if (m_scan[i].type == BMS_BLE_TYPE_UNKNOWN && m_scan[i].last_seen < oldest) {
					oldest = m_scan[i].last_seen;
					idx = i;
				}
			}
			if (idx < 0 || type == BMS_BLE_TYPE_UNKNOWN) {
				unlock();
				return;
			}
			memset(&m_scan[idx], 0, sizeof(m_scan[idx]));
			memcpy(m_scan[idx].addr, r->bda, 6);
		}
	}

	m_scan[idx].addr_type = r->ble_addr_type;
	m_scan[idx].rssi = r->rssi;
	m_scan[idx].last_seen = now_ms();
	if (type != BMS_BLE_TYPE_UNKNOWN) {
		m_scan[idx].type = type;
	}
	if (name && name_len > 0 && m_scan[idx].name[0] == '\0') {
		int l = name_len < (BMS_BLE_NAME_LEN - 1) ? name_len : (BMS_BLE_NAME_LEN - 1);
		memcpy(m_scan[idx].name, name, l);
		m_scan[idx].name[l] = '\0';
	}

	unlock();
}

static void do_open(void) {
	if (m_gattc_if == ESP_GATT_IF_NONE || m_open_requested || !m_target_set || !m_auto_reconnect) {
		return;
	}
	m_open_requested = true;
	STAGE(4);
	DBG("Opening %02X:%02X:%02X:%02X:%02X:%02X type %s", m_target_addr[0], m_target_addr[1],
			m_target_addr[2], m_target_addr[3], m_target_addr[4], m_target_addr[5], type_name(m_target_type));
	esp_ble_gap_set_prefer_conn_params(m_target_addr, BMS_CONN_INT_DISC_MIN, BMS_CONN_INT_DISC_MAX, 0, BMS_CONN_TIMEOUT);
	esp_err_t r = esp_ble_gattc_open(m_gattc_if, m_target_addr, m_target_addr_type, true);
	if (r != ESP_OK) {
		DBG("gattc_open failed: %d", r);
		m_open_requested = false;
		m_state = BMS_BLE_STATE_IDLE;
		schedule_retry();
	}
}

// The controller uses the scan parameters while it initiates the connection,
// so switch to the full duty window right before the open. The attempt is
// bounded by CONFIG_BT_BLE_ESTAB_LINK_CONN_TOUT and the retry backoff keeps
// the average radio load low.
static void request_open(void) {
	if (m_scanning) {
		m_open_after_scan_stop = true;
		esp_ble_gap_stop_scanning();
	} else {
		m_open_after_params = true;
		scan_apply_params(SCAN_MODE_INIT);
	}
}

void bms_ble_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
	switch (event) {
	case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
		if (m_scan_params_pending) {
			m_scan_params_pending = false;
			if (esp_ble_gap_start_scanning(m_scan_duration_s) != ESP_OK) {
				m_scan_user = false;
			}
		}
		if (m_open_after_params) {
			m_open_after_params = false;
			do_open();
		}
		break;

	case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
		m_scanning = param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS;
		if (!m_scanning) {
			DBG("Scan start failed: %d", param->scan_start_cmpl.status);
			m_scan_user = false;
		}
		break;

	case ESP_GAP_BLE_SCAN_RESULT_EVT:
		if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
			scan_result(&param->scan_rst);
		} else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
			m_scanning = false;
			m_scan_user = false;
			if (m_open_after_scan_stop) {
				m_open_after_scan_stop = false;
				m_open_after_params = true;
				scan_apply_params(SCAN_MODE_INIT);
			}
		}
		break;

	case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
		m_scanning = false;
		m_scan_user = false;
		if (m_open_after_scan_stop) {
			m_open_after_scan_stop = false;
			m_open_after_params = true;
			scan_apply_params(SCAN_MODE_INIT);
		}
		break;

	default:
		break;
	}
}

// ---------------------------------------------------------------------------
// GATT client
// ---------------------------------------------------------------------------

static void proto_on_rx(const uint8_t *data, uint16_t len);

static void discover_chars(void) {
	esp_bt_uuid_t rx = {.len = ESP_UUID_LEN_16};
	esp_bt_uuid_t tx = {.len = ESP_UUID_LEN_16};
	bool same = false;
	switch (m_conn.type) {
	case BMS_BLE_TYPE_JBD: rx.uuid.uuid16 = UUID_JBD_RX; tx.uuid.uuid16 = UUID_JBD_TX; break;
	case BMS_BLE_TYPE_DALY: rx.uuid.uuid16 = UUID_DALY_RX; tx.uuid.uuid16 = UUID_DALY_TX; break;
	case BMS_BLE_TYPE_LIPOWER: rx.uuid.uuid16 = UUID_LIPOWER_RXTX; tx.uuid.uuid16 = UUID_LIPOWER_RXTX; same = true; break;
	case BMS_BLE_TYPE_LITECH:
		rx.len = ESP_UUID_LEN_128;
		tx.len = ESP_UUID_LEN_128;
		memcpy(rx.uuid.uuid128, UUID_NUS_RX, 16);
		memcpy(tx.uuid.uuid128, UUID_NUS_TX, 16);
		break;
	default: break;
	}

	esp_gattc_char_elem_t elem;
	uint16_t count = 1;
	if (esp_ble_gattc_get_char_by_uuid(m_gattc_if, m_conn.conn_id, m_conn.svc_start,
			m_conn.svc_end, rx, &elem, &count) == ESP_GATT_OK && count > 0) {
		m_conn.rx_handle = elem.char_handle;
		if (same) {
			m_conn.tx_handle = elem.char_handle;
			m_conn.tx_write_nr = (elem.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0;
		}
	}

	if (!same) {
		count = 1;
		if (esp_ble_gattc_get_char_by_uuid(m_gattc_if, m_conn.conn_id, m_conn.svc_start,
				m_conn.svc_end, tx, &elem, &count) == ESP_GATT_OK && count > 0) {
			m_conn.tx_handle = elem.char_handle;
			m_conn.tx_write_nr = (elem.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0;
		}
	}

	DBG("Chars rx=0x%04X tx=0x%04X write_nr=%d", m_conn.rx_handle, m_conn.tx_handle, m_conn.tx_write_nr);

	if (m_conn.rx_handle == 0 || m_conn.tx_handle == 0) {
		DBG("Characteristics not found");
		esp_ble_gattc_close(m_gattc_if, m_conn.conn_id);
		return;
	}

	esp_ble_gattc_register_for_notify(m_gattc_if, m_conn.bda, m_conn.rx_handle);
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
	switch (event) {
	case ESP_GATTC_REG_EVT:
		if (param->reg.app_id == BMS_GATTC_APP_ID && param->reg.status == ESP_GATT_OK) {
			m_gattc_if = gattc_if;
			m_state = BMS_BLE_STATE_IDLE;
		}
		break;

	case ESP_GATTC_OPEN_EVT:
		m_open_requested = false;
		if (param->open.status == ESP_GATT_OK) {
			reset_conn();
			m_conn.open = true;
			m_conn.conn_id = param->open.conn_id;
			memcpy(m_conn.bda, param->open.remote_bda, 6);
			if (!m_target_set || !m_auto_reconnect) {
				DBG("Open ok but target was forgotten, closing");
				esp_ble_gattc_close(gattc_if, m_conn.conn_id);
				break;
			}
			DBG("Open ok, conn_id %d", m_conn.conn_id);
			STAGE(5);
			m_target_addr_known = true;
			// One GATT procedure at a time: the service search starts in
			// ESP_GATTC_CFG_MTU_EVT, like the esp-idf gatt_client example.
			if (esp_ble_gattc_send_mtu_req(gattc_if, m_conn.conn_id) != ESP_OK) {
				esp_ble_gattc_search_service(gattc_if, m_conn.conn_id, NULL);
			}
		} else {
			DBG("Open failed: %d", param->open.status);
			m_state = BMS_BLE_STATE_IDLE;
			schedule_retry();
		}
		break;

	case ESP_GATTC_CFG_MTU_EVT:
		if (m_conn.open && param->cfg_mtu.conn_id == m_conn.conn_id) {
			DBG("MTU %d", param->cfg_mtu.mtu);
			STAGE(6);
			esp_ble_gattc_search_service(gattc_if, m_conn.conn_id, NULL);
		}
		break;

	case ESP_GATTC_SEARCH_RES_EVT:
		if (param->search_res.conn_id != m_conn.conn_id) {
			break;
		}
		{
			bms_ble_type_t t = BMS_BLE_TYPE_UNKNOWN;
			uint16_t uuid = 0;
			if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
				uuid = param->search_res.srvc_id.uuid.uuid.uuid16;
				if (uuid == UUID_JBD_SVC) {
					t = BMS_BLE_TYPE_JBD;
				} else if (uuid == UUID_DALY_SVC) {
					t = BMS_BLE_TYPE_DALY;
				} else if (uuid == UUID_LIPOWER_SVC) {
					t = BMS_BLE_TYPE_LIPOWER;
				}
			} else if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 &&
					memcmp(param->search_res.srvc_id.uuid.uuid.uuid128, UUID_NUS_SVC, 16) == 0) {
				t = BMS_BLE_TYPE_LITECH;
				uuid = 0x6E40;
			}

			if (t != BMS_BLE_TYPE_UNKNOWN && (m_target_type == BMS_BLE_TYPE_AUTO || m_target_type == t) &&
					m_conn.type == BMS_BLE_TYPE_UNKNOWN) {
				m_conn.type = t;
				m_conn.svc_start = param->search_res.start_handle;
				m_conn.svc_end = param->search_res.end_handle;
				DBG("Service 0x%04X -> %s", uuid, type_name(t));
			}
		}
		break;

	case ESP_GATTC_SEARCH_CMPL_EVT:
		if (param->search_cmpl.conn_id != m_conn.conn_id) {
			break;
		}
		STAGE(7);
		if (m_conn.type == BMS_BLE_TYPE_UNKNOWN) {
			DBG("No supported BMS service found");
			esp_ble_gattc_close(gattc_if, m_conn.conn_id);
		} else {
			discover_chars();
			STAGE(8);
		}
		break;

	case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
		STAGE(9);
		if (param->reg_for_notify.status != ESP_GATT_OK) {
			DBG("Notify reg failed: %d", param->reg_for_notify.status);
			esp_ble_gattc_close(gattc_if, m_conn.conn_id);
			break;
		}

		esp_gattc_descr_elem_t descr;
		uint16_t count = 1;
		esp_bt_uuid_t u = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG};
		if (esp_ble_gattc_get_descr_by_char_handle(gattc_if, m_conn.conn_id, m_conn.rx_handle,
				u, &descr, &count) == ESP_GATT_OK && count > 0) {
			uint8_t v[2] = {0x01, 0x00};
			esp_ble_gattc_write_char_descr(gattc_if, m_conn.conn_id, descr.handle, 2, v,
					ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
		} else {
			DBG("No CCCD, assuming notifications are on");
			m_conn.subscribed = true;
		}
	} break;

	case ESP_GATTC_WRITE_DESCR_EVT:
		if (param->write.conn_id != m_conn.conn_id) {
			break;
		}
		if (param->write.status == ESP_GATT_OK) {
			DBG("Subscribed");
			STAGE(10);
			m_conn.subscribed = true;

			esp_ble_conn_update_params_t cp = {
				.min_int = BMS_CONN_INT_MIN,
				.max_int = BMS_CONN_INT_MAX,
				.latency = 0,
				.timeout = BMS_CONN_TIMEOUT,
			};
			memcpy(cp.bda, m_conn.bda, 6);
			esp_ble_gap_update_conn_params(&cp);
		} else {
			DBG("CCCD write failed: %d", param->write.status);
			esp_ble_gattc_close(gattc_if, m_conn.conn_id);
		}
		break;

	case ESP_GATTC_NOTIFY_EVT:
		if (m_conn.open && param->notify.conn_id == m_conn.conn_id &&
				param->notify.handle == m_conn.rx_handle) {
			proto_on_rx(param->notify.value, param->notify.value_len);
		}
		break;

	case ESP_GATTC_DISCONNECT_EVT:
		if (m_conn.open && param->disconnect.conn_id == m_conn.conn_id) {
			DBG("Disconnected, reason 0x%X", param->disconnect.reason);
			reset_conn();
			schedule_retry();
			if (m_state == BMS_BLE_STATE_CONNECTED || m_state == BMS_BLE_STATE_CONNECTING) {
				m_state = BMS_BLE_STATE_IDLE;
			}
			if (lock()) {
				m_data.valid = false;
				unlock();
			}
		}
		break;

	case ESP_GATTC_CLOSE_EVT:
		if (m_conn.open && param->close.conn_id == m_conn.conn_id) {
			reset_conn();
			schedule_retry();
			if (m_state == BMS_BLE_STATE_CONNECTED || m_state == BMS_BLE_STATE_CONNECTING) {
				m_state = BMS_BLE_STATE_IDLE;
			}
		}
		break;

	default:
		break;
	}
}

// ---------------------------------------------------------------------------
// Request / response helpers (poll task context)
// ---------------------------------------------------------------------------

static bool send_cmd(const uint8_t *cmd, int len, uint8_t expect, uint8_t expect2) {
	if (!m_conn.open || !m_conn.subscribed || m_conn.tx_handle == 0) {
		return false;
	}

	m_rx_len = 0;
	m_rx_valid = false;
	m_rx_expect = expect;
	m_rx_expect2 = expect2;
	m_stage_frames = 0;
	xSemaphoreTake(m_rx_sem, 0);

	STAGE(11);
	esp_err_t r = esp_ble_gattc_write_char(m_gattc_if, m_conn.conn_id, m_conn.tx_handle, len,
			(uint8_t *)cmd, m_conn.tx_write_nr ? ESP_GATT_WRITE_TYPE_NO_RSP : ESP_GATT_WRITE_TYPE_RSP,
			ESP_GATT_AUTH_REQ_NONE);

	if (r != ESP_OK) {
		DBG("write failed: %d", r);
		return false;
	}

	return true;
}

static bool wait_rx(int timeout_ms) {
	if (xSemaphoreTake(m_rx_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
		return false;
	}
	return m_rx_valid;
}

static bool transact(const uint8_t *cmd, int len, uint8_t expect, uint8_t expect2) {
	if (!send_cmd(cmd, len, expect, expect2)) {
		return false;
	}
	bool ok = wait_rx(CMD_TIMEOUT_MS);
	if (!ok) {
		m_work.err_count++;
	}
	return ok;
}

static void rx_reset(void) {
	m_rx_len = 0;
}

static void rx_done(void) {
	STAGE(12);
	m_rx_valid = true;
	xSemaphoreGive(m_rx_sem);
}

// ---------------------------------------------------------------------------
// JBD protocol
// ---------------------------------------------------------------------------

static const uint8_t JBD_CMD_BASIC[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
static const uint8_t JBD_CMD_CELLS[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

static void jbd_on_rx(void) {
	if (m_rx_buf[0] != 0xDD) {
		rx_reset();
		return;
	}
	if (m_rx_len < 7) {
		return;
	}
	int total = m_rx_buf[3] + 7;
	if (total > RX_BUF_SIZE) {
		rx_reset();
		return;
	}
	if (m_rx_len < total) {
		return;
	}
	if (m_rx_buf[total - 1] != 0x77) {
		rx_reset();
		return;
	}

	uint16_t sum = 0;
	for (int i = 2; i < total - 3; i++) {
		sum += m_rx_buf[i];
	}
	uint16_t crc = (uint16_t)(0x10000 - sum);
	if (crc != be16(&m_rx_buf[total - 3])) {
		m_work.err_count++;
		rx_reset();
		return;
	}

	if (m_rx_buf[1] != m_rx_expect || m_rx_buf[2] != 0x00) {
		rx_reset();
		return;
	}

	m_rx_len = total;
	rx_done();
}

static bool jbd_poll(void) {
	if (!transact(JBD_CMD_BASIC, sizeof(JBD_CMD_BASIC), 0x03, 0)) {
		return false;
	}

	const uint8_t *b = m_rx_buf;
	if (m_rx_len < 27) {
		return false;
	}

	m_work.voltage = be16(&b[4]) / 100.0f;
	m_work.current = (int16_t)be16(&b[6]) / 100.0f;
	m_work.ah_remain = be16(&b[8]) / 100.0f;
	m_work.ah_nominal = be16(&b[10]) / 100.0f;
	m_work.cycles = be16(&b[12]);
	m_work.balance_bits = ((uint32_t)be16(&b[18]) << 16) | be16(&b[16]);
	m_work.problem = be16(&b[20]);
	m_work.soc = b[23] / 100.0f;
	m_work.chg_fet = (b[24] & 0x01) != 0;
	m_work.dis_fet = (b[24] & 0x02) != 0;
	m_work.cell_count = b[25] > BMS_BLE_MAX_CELLS ? BMS_BLE_MAX_CELLS : b[25];
	m_work.temp_count = b[26] > BMS_BLE_MAX_TEMPS ? BMS_BLE_MAX_TEMPS : b[26];
	for (int i = 0; i < m_work.temp_count && (27 + i * 2 + 1) < m_rx_len; i++) {
		m_work.temps[i] = ((int)be16(&b[27 + i * 2]) - 2731) / 10.0f;
	}
	m_work.temp_mos_valid = false;

	if (!transact(JBD_CMD_CELLS, sizeof(JBD_CMD_CELLS), 0x04, 0)) {
		return false;
	}

	int nc = m_rx_buf[3] / 2;
	if (nc > BMS_BLE_MAX_CELLS) {
		nc = BMS_BLE_MAX_CELLS;
	}
	if (m_work.cell_count == 0 || m_work.cell_count > nc) {
		m_work.cell_count = nc;
	}
	for (int i = 0; i < nc; i++) {
		m_work.cells[i] = be16(&m_rx_buf[4 + i * 2]) / 1000.0f;
	}

	return true;
}

// ---------------------------------------------------------------------------
// Daly protocol
// ---------------------------------------------------------------------------

static int modbus_cmd(uint8_t *out, uint8_t dev, uint16_t addr, uint16_t count) {
	out[0] = dev;
	out[1] = 0x03;
	out[2] = addr >> 8;
	out[3] = addr & 0xFF;
	out[4] = count >> 8;
	out[5] = count & 0xFF;
	uint16_t crc = crc_modbus(out, 6);
	out[6] = crc & 0xFF;
	out[7] = crc >> 8;
	return 8;
}

// Shared by Daly (0xD2 / 0x81) and LiPower: head, 0x03, len, data..., crc16 (LE)
static void modbus_on_rx(void) {
	if (m_rx_buf[0] != m_rx_expect || (m_rx_len >= 2 && m_rx_buf[1] != 0x03)) {
		rx_reset();
		return;
	}
	if (m_rx_len < 3) {
		return;
	}
	int total = m_rx_buf[2] + 5;
	if (total > RX_BUF_SIZE) {
		rx_reset();
		return;
	}
	if (m_rx_len < total) {
		return;
	}
	uint16_t crc = crc_modbus(m_rx_buf, total - 2);
	uint16_t got = m_rx_buf[total - 2] | (m_rx_buf[total - 1] << 8);
	if (crc != got) {
		m_work.err_count++;
		rx_reset();
		return;
	}
	m_rx_len = total;
	rx_done();
}

// Legacy Daly frames: A5 01 cmd 08 d[8] cs, several frames may arrive in one
// notification and multi-frame replies (0x95, 0x96) span notifications.
static void daly_a5_on_rx(void) {
	while (m_rx_len >= 13) {
		if (m_rx_buf[0] != 0xA5) {
			// Resync
			memmove(m_rx_buf, m_rx_buf + 1, m_rx_len - 1);
			m_rx_len--;
			continue;
		}

		uint8_t cs = 0;
		for (int i = 0; i < 12; i++) {
			cs += m_rx_buf[i];
		}

		bool complete = false;
		if (cs == m_rx_buf[12] && m_rx_buf[2] == m_rx_expect) {
			const uint8_t *d = &m_rx_buf[4];
			switch (m_rx_expect) {
			case 0x95: {
				int frame = d[0];
				if (frame >= 1 && frame <= (BMS_BLE_MAX_CELLS / 3) + 1) {
					for (int i = 0; i < 3; i++) {
						int c = (frame - 1) * 3 + i;
						if (c < BMS_BLE_MAX_CELLS) {
							m_stage_cells[c] = be16(&d[1 + i * 2]) / 1000.0f;
						}
					}
					if (frame > m_stage_frames) {
						m_stage_frames = frame;
					}
					complete = m_stage_frames * 3 >= m_rx_expect2;
				}
			} break;

			case 0x96: {
				int frame = d[0];
				if (frame >= 1 && frame <= 2) {
					for (int i = 0; i < 7; i++) {
						int t = (frame - 1) * 7 + i;
						if (t < BMS_BLE_MAX_TEMPS) {
							m_stage_temps[t] = (float)d[1 + i] - 40.0f;
						}
					}
					if (frame > m_stage_frames) {
						m_stage_frames = frame;
					}
					complete = m_stage_frames * 7 >= m_rx_expect2;
				}
			} break;

			default:
				complete = true;
				break;
			}
		} else if (cs != m_rx_buf[12]) {
			m_work.err_count++;
		}

		if (complete) {
			// Keep this frame at the start of the buffer for the parser
			m_rx_len = 13;
			rx_done();
			return;
		}

		memmove(m_rx_buf, m_rx_buf + 13, m_rx_len - 13);
		m_rx_len -= 13;
	}
}

static bool daly_a5_transact(uint8_t cmd, uint8_t expect2) {
	uint8_t f[13] = {0xA5, 0x40, cmd, 0x08, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t cs = 0;
	for (int i = 0; i < 12; i++) {
		cs += f[i];
	}
	f[12] = cs;
	return transact(f, 13, cmd, expect2);
}

static bool daly_poll_d2(void) {
	uint8_t cmd[8];
	modbus_cmd(cmd, 0xD2, 0x0000, 62);
	if (!transact(cmd, 8, 0xD2, 0)) {
		return false;
	}
	if (m_rx_buf[2] != 124) {
		return false;
	}

	const uint8_t *d = &m_rx_buf[3];
	m_work.voltage = be16(&d[80]) / 10.0f;
	m_work.current = ((int)be16(&d[82]) - 30000) / 10.0f;
	m_work.soc = be16(&d[84]) / 1000.0f;
	m_work.ah_remain = be16(&d[96]) / 10.0f;
	int nc = be16(&d[98]);
	int nt = be16(&d[100]);
	m_work.cell_count = nc > 32 ? 32 : nc;
	m_work.temp_count = nt > BMS_BLE_MAX_TEMPS ? BMS_BLE_MAX_TEMPS : nt;
	m_work.cycles = be16(&d[102]);
	m_work.balance_bits = be16(&d[104]);
	m_work.chg_fet = be16(&d[106]) != 0;
	m_work.dis_fet = be16(&d[108]) != 0;
	m_work.problem = 0;
	for (int i = 0; i < 8; i++) {
		m_work.problem = (m_work.problem << 8) | d[116 + i];
	}
	for (int i = 0; i < m_work.cell_count; i++) {
		m_work.cells[i] = be16(&d[i * 2]) / 1000.0f;
	}
	for (int i = 0; i < m_work.temp_count; i++) {
		m_work.temps[i] = (float)be16(&d[64 + i * 2]) - 40.0f;
	}

	// MOS temperature, optional
	modbus_cmd(cmd, 0xD2, 0x003E, 9);
	if (transact(cmd, 8, 0xD2, 0) && m_rx_buf[2] >= 10) {
		uint16_t raw = be16(&m_rx_buf[11]);
		if (raw != 0 && raw != 0xFFFF) {
			m_work.temp_mos = (float)raw - 40.0f;
			m_work.temp_mos_valid = true;
		} else {
			m_work.temp_mos_valid = false;
		}
	} else {
		m_work.temp_mos_valid = false;
		m_work.err_count--; // Not an error, many units do not support it
	}

	return true;
}

static bool daly_poll_x81(void) {
	uint8_t cmd[8];
	modbus_cmd(cmd, 0x81, 0x0000, 64);
	if (!transact(cmd, 8, 0x51, 0)) {
		return false;
	}
	if (m_rx_buf[2] != 128) {
		return false;
	}

	const uint8_t *d = &m_rx_buf[3];
	m_work.voltage = be16(&d[112]) / 10.0f;
	m_work.current = ((int)be16(&d[114]) - 30000) / 10.0f;
	m_work.soc = be16(&d[116]) / 1000.0f;
	int nc = be16(&d[120]);
	int nt = be16(&d[122]);
	m_work.cell_count = nc > BMS_BLE_MAX_CELLS ? BMS_BLE_MAX_CELLS : nc;
	m_work.temp_count = nt > BMS_BLE_MAX_TEMPS ? BMS_BLE_MAX_TEMPS : nt;
	for (int i = 0; i < m_work.cell_count; i++) {
		m_work.cells[i] = be16(&d[i * 2]) / 1000.0f;
	}
	for (int i = 0; i < m_work.temp_count; i++) {
		m_work.temps[i] = (float)be16(&d[96 + i * 2]) - 40.0f;
	}
	m_work.temp_mos_valid = false;

	modbus_cmd(cmd, 0x81, 0x0041, 62);
	if (!transact(cmd, 8, 0x51, 0)) {
		return false;
	}
	if (m_rx_buf[2] != 124) {
		return false;
	}

	d = &m_rx_buf[3];
	m_work.ah_remain = be16(&d[20]) / 10.0f;
	m_work.cycles = be16(&d[22]);
	m_work.balance_bits = be16(&d[24]);
	m_work.chg_fet = be16(&d[34]) != 0;
	m_work.dis_fet = be16(&d[36]) != 0;
	m_work.problem = ((uint32_t)be16(&d[88]) << 16) | be16(&d[90]);

	return true;
}

static bool daly_poll_a5(void) {
	// 0x94: cell count, temp count, cycles
	if (!daly_a5_transact(0x94, 0)) {
		return false;
	}
	const uint8_t *d = &m_rx_buf[4];
	int nc = d[0];
	int nt = d[1];
	m_work.cell_count = nc > BMS_BLE_MAX_CELLS ? BMS_BLE_MAX_CELLS : nc;
	m_work.temp_count = nt > BMS_BLE_MAX_TEMPS ? BMS_BLE_MAX_TEMPS : nt;
	m_work.cycles = be16(&d[5]);

	// 0x90: voltage, current, soc
	if (!daly_a5_transact(0x90, 0)) {
		return false;
	}
	d = &m_rx_buf[4];
	m_work.voltage = be16(&d[0]) / 10.0f;
	m_work.current = ((int)be16(&d[4]) - 30000) / 10.0f;
	m_work.soc = be16(&d[6]) / 1000.0f;

	// 0x93: mosfet status, remaining capacity
	if (!daly_a5_transact(0x93, 0)) {
		return false;
	}
	d = &m_rx_buf[4];
	m_work.chg_fet = d[1] != 0;
	m_work.dis_fet = d[2] != 0;
	m_work.ah_remain = (float)(((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) | ((uint32_t)d[6] << 8) | d[7]) / 1000.0f;

	// 0x95: cell voltages, 3 per frame
	if (m_work.cell_count > 0) {
		if (!daly_a5_transact(0x95, m_work.cell_count)) {
			return false;
		}
		memcpy(m_work.cells, m_stage_cells, sizeof(float) * m_work.cell_count);
	}

	// 0x96: temperatures, 7 per frame
	if (m_work.temp_count > 0) {
		if (!daly_a5_transact(0x96, m_work.temp_count)) {
			return false;
		}
		memcpy(m_work.temps, m_stage_temps, sizeof(float) * m_work.temp_count);
	}

	// 0x98: failure status, optional
	if (daly_a5_transact(0x98, 0)) {
		d = &m_rx_buf[4];
		m_work.problem = 0;
		for (int i = 0; i < 7; i++) {
			m_work.problem = (m_work.problem << 8) | d[i];
		}
	} else {
		m_work.err_count--;
	}

	m_work.balance_bits = 0;
	m_work.temp_mos_valid = false;
	return true;
}

static bool daly_poll(void) {
	if (m_work.proto_variant == 0) {
		if (daly_poll_d2()) {
			m_work.proto_variant = DALY_VARIANT_D2;
		} else if (daly_poll_x81()) {
			m_work.proto_variant = DALY_VARIANT_X81;
		} else if (daly_poll_a5()) {
			m_work.proto_variant = DALY_VARIANT_A5;
		} else {
			return false;
		}
		DBG("Daly protocol variant %d", m_work.proto_variant);
		return true;
	}

	switch (m_work.proto_variant) {
	case DALY_VARIANT_D2: return daly_poll_d2();
	case DALY_VARIANT_X81: return daly_poll_x81();
	case DALY_VARIANT_A5: return daly_poll_a5();
	default: return false;
	}
}

// ---------------------------------------------------------------------------
// LiPower protocol
// ---------------------------------------------------------------------------

static const uint8_t LIPOWER_IDS[] = {0x22, 0x0B, 0x08, 0x38};

static bool lipower_poll_id(uint8_t id) {
	uint8_t cmd[8];
	modbus_cmd(cmd, id, 0x0400, 8);
	if (!transact(cmd, 8, id, 0)) {
		return false;
	}
	if (m_rx_buf[2] < 14) {
		return false;
	}

	const uint8_t *f = m_rx_buf;
	m_work.ah_remain = be16(&f[3]);
	m_work.soc = be16(&f[5]) / 100.0f;
	m_work.runtime_s = (uint32_t)be16(&f[7]) * 3600 + (uint32_t)be16(&f[9]) * 60;
	float cur = be16(&f[13]) / 100.0f;
	m_work.current = f[12] ? -cur : cur;
	m_work.voltage = be16(&f[15]) / 10.0f;
	m_work.cell_count = 0;
	m_work.temp_count = 0;
	m_work.temp_mos_valid = false;
	m_work.chg_fet = true;
	m_work.dis_fet = true;
	return true;
}

static bool lipower_poll(void) {
	if (m_work.proto_variant == 0) {
		for (int i = 0; i < (int)sizeof(LIPOWER_IDS); i++) {
			if (lipower_poll_id(LIPOWER_IDS[i])) {
				m_work.proto_variant = i + 1;
				DBG("LiPower frame head 0x%02X", LIPOWER_IDS[i]);
				return true;
			}
		}
		return false;
	}
	return lipower_poll_id(LIPOWER_IDS[m_work.proto_variant - 1]);
}

// ---------------------------------------------------------------------------
// LiTech protocol (Modbus RTU over Nordic UART, register map reverse
// engineered by shambler: live data block 0xD000..0xD03A)
// ---------------------------------------------------------------------------

static bool litech_poll(void) {
	uint8_t cmd[8];
	modbus_cmd(cmd, LITECH_SLAVE_ID, LITECH_LIVE_ADDR, LITECH_LIVE_COUNT);
	if (!transact(cmd, 8, LITECH_SLAVE_ID, 0)) {
		return false;
	}
	if (m_rx_buf[2] != LITECH_LIVE_COUNT * 2) {
		return false;
	}

	const uint8_t *d = &m_rx_buf[3];
#define LREG(a) be16(&d[((a) - LITECH_LIVE_ADDR) * 2])

	int nc = 0;
	for (int i = 0; i < 32; i++) {
		uint16_t v = be16(&d[i * 2]);
		if (v == LITECH_CELL_UNUSED || v == 0) {
			continue;
		}
		if (nc < BMS_BLE_MAX_CELLS) {
			m_work.cells[nc++] = v / 1000.0f;
		}
	}
	m_work.cell_count = nc;

	m_work.voltage = LREG(0xD025) / 100.0f;
	// TODO: not verified against a load, 0xD032 is the best candidate (0.01 A, signed)
	m_work.current = (int16_t)LREG(0xD032) / 100.0f;
	m_work.temp_count = 4;
	for (int i = 0; i < 4; i++) {
		m_work.temps[i] = LREG(0xD026 + i) / 10.0f - 40.0f;
	}
	m_work.temp_mos_valid = false;
	m_work.soc = LREG(0xD034) / 100.0f;
	m_work.soh = LREG(0xD035) / 100.0f;
	m_work.ah_remain = LREG(0xD036) / 10.0f;
	m_work.ah_nominal = LREG(0xD038) / 10.0f;
	m_work.cycles = LREG(0xD03A);
	m_work.chg_fet = true;
	m_work.dis_fet = true;
	m_work.balance_bits = 0;
	m_work.problem = 0;
#undef LREG
	return true;
}

// ---------------------------------------------------------------------------
// Protocol dispatch
// ---------------------------------------------------------------------------

static void proto_on_rx(const uint8_t *data, uint16_t len) {
	if (m_rx_valid) {
		return; // Previous reply not consumed yet
	}

	if (m_rx_len + len > RX_BUF_SIZE) {
		m_rx_len = 0;
	}
	memcpy(&m_rx_buf[m_rx_len], data, len);
	m_rx_len += len;

	switch (m_conn.type) {
	case BMS_BLE_TYPE_JBD:
		jbd_on_rx();
		break;
	case BMS_BLE_TYPE_DALY:
		if (m_rx_expect == 0xD2 || m_rx_expect == 0x51) {
			modbus_on_rx();
		} else {
			daly_a5_on_rx();
		}
		break;
	case BMS_BLE_TYPE_LIPOWER:
	case BMS_BLE_TYPE_LITECH:
		modbus_on_rx();
		break;
	default:
		m_rx_len = 0;
		break;
	}
}

static bool proto_poll(void) {
	bool ok = false;
	switch (m_conn.type) {
	case BMS_BLE_TYPE_JBD: ok = jbd_poll(); break;
	case BMS_BLE_TYPE_DALY: ok = daly_poll(); break;
	case BMS_BLE_TYPE_LIPOWER: ok = lipower_poll(); break;
	case BMS_BLE_TYPE_LITECH: ok = litech_poll(); break;
	default: break;
	}

	if (!ok) {
		return false;
	}

	m_work.type = m_conn.type;
	m_work.valid = true;
	m_work.update_time = xTaskGetTickCount();
	m_work.msg_count++;

	m_work.cell_min = 10.0f;
	m_work.cell_max = 0.0f;
	for (int i = 0; i < m_work.cell_count; i++) {
		if (m_work.cells[i] > 0.1f && m_work.cells[i] < m_work.cell_min) {
			m_work.cell_min = m_work.cells[i];
		}
		if (m_work.cells[i] > m_work.cell_max) {
			m_work.cell_max = m_work.cells[i];
		}
	}
	if (m_work.cell_count == 0) {
		m_work.cell_min = 0.0f;
	}

	if (lock()) {
		m_data = m_work;
		unlock();
	}

	STAGE(13);
	if (m_update_vesc) {
		update_vesc_bms();
		STAGE(14);
		if (m_send_can) {
			// Forward to the VESC on the CAN bus, like the OW BMS bridge does
			bms_send_status_can();
		}
	}
	STAGE(15);

	return true;
}

static void update_vesc_bms(void) {
	volatile bms_values *v = bms_get_values();
	const bms_ble_data_t *d = &m_work;

	v->v_tot = d->voltage;
	v->i_in = d->current;
	v->i_in_ic = d->current;
	v->ah_cnt = d->ah_remain;
	v->wh_cnt = d->ah_remain * d->voltage;
	v->cell_num = d->cell_count;
	for (int i = 0; i < d->cell_count && i < BMS_MAX_CELLS; i++) {
		v->v_cell[i] = d->cells[i];
		v->bal_state[i] = (d->balance_bits >> i) & 1;
	}
	v->v_cell_min = d->cell_min;
	v->v_cell_max = d->cell_max;
	v->temp_adc_num = d->temp_count;
	float t_max = -300.0f;
	for (int i = 0; i < d->temp_count && i < BMS_MAX_TEMPS; i++) {
		v->temps_adc[i] = d->temps[i];
		if (d->temps[i] > t_max) {
			t_max = d->temps[i];
		}
	}
	v->temp_max_cell = d->temp_count > 0 ? t_max : 0.0f;
	if (d->temp_mos_valid) {
		v->temp_ic = d->temp_mos;
	}
	v->soc = d->soc;
	v->soh = d->soh;
	v->is_charging = d->current > 0.05f;
	v->is_balancing = d->balance_bits != 0;
	v->is_charge_allowed = d->chg_fet;
	snprintf((char *)v->status, BMS_STATUS_LEN, "%s BLE", type_name(d->type));
	v->update_time = xTaskGetTickCount();
}

// ---------------------------------------------------------------------------
// Supervisor task
// ---------------------------------------------------------------------------

static void begin_connect(void) {
	m_connect_start = now_ms();
	m_target_seen = false;
	m_open_after_scan_stop = false;
	m_state = BMS_BLE_STATE_CONNECTING;
	STAGE(1);
	memset(&m_work, 0, sizeof(m_work));
	m_work.soh = 1.0f;
	if (m_target_addr_known) {
		m_target_seen = true;
		DBG("Connecting directly (retry delay %lu ms)", (unsigned long)m_reconnect_delay);
		request_open();
	} else {
		DBG("Connecting, scanning for target");
		scan_start_internal(TARGET_SCAN_MS / 1000 + 1, false);
	}
}

static void bms_ble_task(void *arg) {
	(void)arg;
	uint32_t last_poll = 0;
	int poll_fails = 0;

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
		dbg_flush();

		switch (m_state) {
		case BMS_BLE_STATE_DISABLED:
			break;

		case BMS_BLE_STATE_IDLE:
			if (m_conn.open && (!m_target_set || !m_auto_reconnect)) {
				// Link survived a disconnect/forget request, drop it
				esp_ble_gattc_close(m_gattc_if, m_conn.conn_id);
			} else if (m_target_set && m_auto_reconnect && !m_conn.open && !m_open_requested &&
					age_ms(m_last_attempt) > retry_delay()) {
				begin_connect();
			}
			break;

		case BMS_BLE_STATE_CONNECTING:
			if (m_conn.open && m_conn.subscribed) {
				DBG("Connected (%s)", type_name(m_conn.type));
				m_state = BMS_BLE_STATE_CONNECTED;
				poll_fails = 0;
				last_poll = 0;
				m_work.proto_variant = 0;
				m_work.msg_count = m_data.msg_count;
				m_work.err_count = m_data.err_count;
				vTaskDelay(pdMS_TO_TICKS(300));
			} else if (!m_target_seen && !m_open_requested && !m_conn.open &&
					age_ms(m_connect_start) > TARGET_SCAN_MS) {
				// Not seen while scanning, try a direct connection
				m_target_seen = true;
				DBG("Target not seen, trying direct connect");
				request_open();
			} else if (age_ms(m_connect_start) > CONNECT_TIMEOUT_MS) {
				DBG("Connect timeout");
				if (m_conn.open) {
					esp_ble_gattc_close(m_gattc_if, m_conn.conn_id);
				}
				m_open_requested = false;
				schedule_retry();
				m_state = BMS_BLE_STATE_IDLE;
			}
			break;

		case BMS_BLE_STATE_CONNECTED:
			if (!m_conn.open) {
				m_state = BMS_BLE_STATE_IDLE;
				break;
			}

			if (!m_auto_reconnect) {
				esp_ble_gattc_close(m_gattc_if, m_conn.conn_id);
				m_state = BMS_BLE_STATE_IDLE;
				break;
			}

			if (age_ms(last_poll) >= POLL_INTERVAL_MS) {
				last_poll = now_ms();
				if (proto_poll()) {
					poll_fails = 0;
					m_reconnect_delay = RECONNECT_DELAY_MS;
				} else {
					poll_fails++;
					DBG("Poll failed (%d)", poll_fails);
					if (poll_fails >= MAX_POLL_FAILS) {
						DBG("Too many failures, reconnecting");
						esp_ble_gattc_close(m_gattc_if, m_conn.conn_id);
						schedule_retry();
						m_state = BMS_BLE_STATE_IDLE;
					}
				}
			}
			break;
		}
	}
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void bms_ble_init(void) {
	if (m_task != NULL) {
		return;
	}

	m_reset_reason = (int)esp_reset_reason();
	if ((m_stage_rtc & 0xFFFFFF00) == STAGE_MAGIC) {
		m_last_stage = m_stage_rtc & 0xFF;
	}
	m_stage_rtc = STAGE_MAGIC;

	m_mutex = xSemaphoreCreateMutex();
	m_rx_sem = xSemaphoreCreateBinary();
	reset_conn();
	memset(&m_data, 0, sizeof(m_data));

	esp_ble_gattc_register_callback(gattc_event_handler);
	esp_ble_gattc_app_register(BMS_GATTC_APP_ID);

	xTaskCreatePinnedToCore(bms_ble_task, "bms_ble", 6144, NULL, 6, &m_task, tskNO_AFFINITY);
}

bool bms_ble_available(void) {
	return m_gattc_if != ESP_GATT_IF_NONE;
}

bool bms_ble_scan_start(float seconds) {
	if (seconds < 1.0f) {
		seconds = 1.0f;
	}
	if (seconds > 60.0f) {
		seconds = 60.0f;
	}

	if (lock()) {
		m_scan_num = 0;
		unlock();
	}

	return scan_start_internal((uint32_t)seconds, true);
}

void bms_ble_scan_stop(void) {
	if (m_scanning) {
		esp_ble_gap_stop_scanning();
	}
	m_scan_user = false;
}

bool bms_ble_scan_active(void) {
	return (m_scanning || m_scan_params_pending) && m_scan_user;
}

int bms_ble_scan_results(bms_ble_scan_entry_t *out, int max) {
	int n = 0;
	if (lock()) {
		n = m_scan_num < max ? m_scan_num : max;
		memcpy(out, m_scan, sizeof(bms_ble_scan_entry_t) * n);
		unlock();
	}
	return n;
}

bool bms_ble_connect(const uint8_t addr[6], bms_ble_type_t type) {
	if (m_gattc_if == ESP_GATT_IF_NONE) {
		return false;
	}

	bool same = m_target_set && memcmp(addr, m_target_addr, 6) == 0 && type == m_target_type;

	memcpy(m_target_addr, addr, 6);
	if (!same) {
		m_target_addr_type = BLE_ADDR_TYPE_PUBLIC;
		m_target_addr_known = false;
	}
	m_target_type = type;
	m_target_set = true;
	m_auto_reconnect = true;
	m_reconnect_delay = RECONNECT_DELAY_MS;

	if (!same && m_conn.open) {
		esp_ble_gattc_close(m_gattc_if, m_conn.conn_id);
	}

	m_last_attempt = now_ms() - RECONNECT_DELAY_MS;
	return true;
}

void bms_ble_disconnect(void) {
	m_auto_reconnect = false;
	m_target_set = false;
	m_target_addr_known = false;
	m_open_after_scan_stop = false;
	m_open_after_params = false;
	if (m_conn.open) {
		esp_ble_gattc_close(m_gattc_if, m_conn.conn_id);
	}
	if (m_scanning && !m_scan_user) {
		esp_ble_gap_stop_scanning();
	}
	if (m_state == BMS_BLE_STATE_CONNECTING) {
		m_state = BMS_BLE_STATE_IDLE;
	}
	if (lock()) {
		m_data.valid = false;
		unlock();
	}
}

bool bms_ble_is_connected(void) {
	return m_state == BMS_BLE_STATE_CONNECTED && m_conn.open;
}

bms_ble_state_t bms_ble_state(void) {
	return m_state;
}

bool bms_ble_target(uint8_t addr[6], bms_ble_type_t *type) {
	if (!m_target_set) {
		return false;
	}
	memcpy(addr, m_target_addr, 6);
	*type = m_target_type;
	return true;
}

const bms_ble_data_t *bms_ble_get_data(void) {
	return &m_data;
}

void bms_ble_set_update_vesc(bool enabled) {
	m_update_vesc = enabled;
}

void bms_ble_set_debug(bool enabled) {
	m_debug = enabled;
}

void bms_ble_set_send_can(bool enabled) {
	m_send_can = enabled;
}

// ---------------------------------------------------------------------------
// LispBM extensions
// ---------------------------------------------------------------------------

static lbm_uint sym_auto, sym_jbd, sym_daly, sym_lipower, sym_litech, sym_unknown;
static lbm_uint sym_disabled, sym_idle, sym_connecting, sym_connected;

typedef struct {
	const char *name;
	lbm_uint sym;
} key_t_;

static key_t_ m_keys[] = {
	{"voltage", 0}, {"current", 0}, {"soc", 0}, {"ah-remain", 0}, {"ah-nominal", 0},
	{"cycles", 0}, {"cell-count", 0}, {"temp-count", 0}, {"cell-min", 0}, {"cell-max", 0},
	{"temp-mos", 0}, {"chg-fet", 0}, {"dis-fet", 0}, {"balance", 0}, {"problem", 0},
	{"msg-count", 0}, {"err-count", 0}, {"age", 0}, {"runtime", 0}, {"variant", 0},
	{"soh", 0},
};

static lbm_value make_str(const char *s) {
	lbm_value res;
	size_t len = strlen(s);
	if (lbm_create_array(&res, len + 1)) {
		lbm_array_header_t *arr = (lbm_array_header_t *)lbm_car(res);
		memcpy(arr->data, s, len);
		((char *)arr->data)[len] = '\0';
		return res;
	}
	return ENC_SYM_MERROR;
}

static lbm_value type_sym(bms_ble_type_t t) {
	switch (t) {
	case BMS_BLE_TYPE_AUTO: return lbm_enc_sym(sym_auto);
	case BMS_BLE_TYPE_JBD: return lbm_enc_sym(sym_jbd);
	case BMS_BLE_TYPE_DALY: return lbm_enc_sym(sym_daly);
	case BMS_BLE_TYPE_LIPOWER: return lbm_enc_sym(sym_lipower);
	case BMS_BLE_TYPE_LITECH: return lbm_enc_sym(sym_litech);
	default: return lbm_enc_sym(sym_unknown);
	}
}

static bool sym_to_type(lbm_value v, bms_ble_type_t *t) {
	if (!lbm_is_symbol(v)) {
		return false;
	}
	lbm_uint s = lbm_dec_sym(v);
	if (s == sym_auto) *t = BMS_BLE_TYPE_AUTO;
	else if (s == sym_jbd) *t = BMS_BLE_TYPE_JBD;
	else if (s == sym_daly) *t = BMS_BLE_TYPE_DALY;
	else if (s == sym_lipower) *t = BMS_BLE_TYPE_LIPOWER;
	else if (s == sym_litech) *t = BMS_BLE_TYPE_LITECH;
	else return false;
	return true;
}

static bool parse_mac(const char *s, uint8_t addr[6]) {
	unsigned int t[6];
	if (!s || sscanf(s, "%x:%x:%x:%x:%x:%x", &t[0], &t[1], &t[2], &t[3], &t[4], &t[5]) != 6) {
		return false;
	}
	for (int i = 0; i < 6; i++) {
		if (t[i] > 255) {
			return false;
		}
		addr[i] = (uint8_t)t[i];
	}
	return true;
}

static void mac_to_str(const uint8_t addr[6], char *out) {
	sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

// (bms-ble-scan [seconds]) -> bool
static lbm_value ext_scan(lbm_value *args, lbm_uint argn) {
	float seconds = 5.0f;
	if (argn >= 1) {
		if (!lbm_is_number(args[0])) {
			return ENC_SYM_TERROR;
		}
		seconds = lbm_dec_as_float(args[0]);
	}
	if (!bms_ble_available()) {
		lbm_set_error_reason("BMS BLE not available");
		return ENC_SYM_EERROR;
	}
	return lbm_enc_bool(bms_ble_scan_start(seconds));
}

static lbm_value ext_scan_stop(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	bms_ble_scan_stop();
	return ENC_SYM_TRUE;
}

static lbm_value ext_scanning(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_bool(bms_ble_scan_active());
}

// (bms-ble-scan-results) -> ((name mac rssi type) ...)
static lbm_value ext_scan_results(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	static bms_ble_scan_entry_t entries[BMS_BLE_SCAN_MAX]; // lisp extensions run on one thread
	int n = bms_ble_scan_results(entries, BMS_BLE_SCAN_MAX);

	lbm_value res = ENC_SYM_NIL;
	for (int i = n - 1; i >= 0; i--) {
		char mac[20];
		mac_to_str(entries[i].addr, mac);

		lbm_value name = make_str(entries[i].name);
		lbm_value macv = make_str(mac);
		if (name == ENC_SYM_MERROR || macv == ENC_SYM_MERROR) {
			return ENC_SYM_MERROR;
		}

		lbm_value item = lbm_cons(type_sym(entries[i].type), ENC_SYM_NIL);
		item = lbm_cons(lbm_enc_i(entries[i].rssi), item);
		item = lbm_cons(macv, item);
		item = lbm_cons(name, item);
		res = lbm_cons(item, res);
		if (res == ENC_SYM_MERROR) {
			return ENC_SYM_MERROR;
		}
	}
	return res;
}

// (bms-ble-connect "aa:bb:cc:dd:ee:ff" ['auto|'jbd|'daly|'lipower]) -> bool
static lbm_value ext_connect(lbm_value *args, lbm_uint argn) {
	if (argn < 1 || argn > 2 || !lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	uint8_t addr[6];
	if (!parse_mac(lbm_dec_str(args[0]), addr)) {
		lbm_set_error_reason("Invalid MAC address");
		return ENC_SYM_EERROR;
	}

	bms_ble_type_t type = BMS_BLE_TYPE_AUTO;
	if (argn >= 2 && !sym_to_type(args[1], &type)) {
		lbm_set_error_reason("Invalid BMS type");
		return ENC_SYM_EERROR;
	}

	if (!bms_ble_available()) {
		lbm_set_error_reason("BMS BLE not available");
		return ENC_SYM_EERROR;
	}

	return lbm_enc_bool(bms_ble_connect(addr, type));
}

static lbm_value ext_disconnect(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	bms_ble_disconnect();
	return ENC_SYM_TRUE;
}

static lbm_value ext_connected(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_bool(bms_ble_is_connected());
}

static lbm_value ext_state(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	switch (bms_ble_state()) {
	case BMS_BLE_STATE_IDLE: return lbm_enc_sym(sym_idle);
	case BMS_BLE_STATE_CONNECTING: return lbm_enc_sym(sym_connecting);
	case BMS_BLE_STATE_CONNECTED: return lbm_enc_sym(sym_connected);
	default: return lbm_enc_sym(sym_disabled);
	}
}

static lbm_value ext_type(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	if (bms_ble_is_connected()) {
		return type_sym(m_conn.type);
	}
	return type_sym(m_target_set ? m_target_type : BMS_BLE_TYPE_UNKNOWN);
}

// (bms-ble-target) -> (mac type) or nil
static lbm_value ext_target(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	uint8_t addr[6];
	bms_ble_type_t type;
	if (!bms_ble_target(addr, &type)) {
		return ENC_SYM_NIL;
	}
	char mac[20];
	mac_to_str(addr, mac);
	lbm_value macv = make_str(mac);
	if (macv == ENC_SYM_MERROR) {
		return macv;
	}
	return lbm_cons(macv, lbm_cons(type_sym(type), ENC_SYM_NIL));
}

// (bms-ble-get 'key) -> value
static lbm_value ext_get(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || !lbm_is_symbol(args[0])) {
		return ENC_SYM_TERROR;
	}

	lbm_uint s = lbm_dec_sym(args[0]);
	int key = -1;
	for (int i = 0; i < (int)(sizeof(m_keys) / sizeof(m_keys[0])); i++) {
		if (m_keys[i].sym == s) {
			key = i;
			break;
		}
	}

	const bms_ble_data_t *d = &m_data;
	lbm_value res;
	if (!lock()) {
		return ENC_SYM_EERROR;
	}

	switch (key) {
	case 0: res = lbm_enc_float(d->voltage); break;
	case 1: res = lbm_enc_float(d->current); break;
	case 2: res = lbm_enc_float(d->soc); break;
	case 3: res = lbm_enc_float(d->ah_remain); break;
	case 4: res = lbm_enc_float(d->ah_nominal); break;
	case 5: res = lbm_enc_i(d->cycles); break;
	case 6: res = lbm_enc_i(d->cell_count); break;
	case 7: res = lbm_enc_i(d->temp_count); break;
	case 8: res = lbm_enc_float(d->cell_min); break;
	case 9: res = lbm_enc_float(d->cell_max); break;
	case 10: res = d->temp_mos_valid ? lbm_enc_float(d->temp_mos) : ENC_SYM_NIL; break;
	case 11: res = lbm_enc_bool(d->chg_fet); break;
	case 12: res = lbm_enc_bool(d->dis_fet); break;
	case 13: res = lbm_enc_u32(d->balance_bits); break;
	case 14: res = lbm_enc_u32((uint32_t)d->problem); break;
	case 15: res = lbm_enc_u32(d->msg_count); break;
	case 16: res = lbm_enc_u32(d->err_count); break;
	case 17: res = d->valid ? lbm_enc_float(UTILS_AGE_S(d->update_time)) : lbm_enc_float(-1.0f); break;
	case 18: res = lbm_enc_u32(d->runtime_s); break;
	case 19: res = lbm_enc_i(d->proto_variant); break;
	case 20: res = lbm_enc_float(d->soh); break;
	default:
		unlock();
		lbm_set_error_reason("Unknown key");
		return ENC_SYM_EERROR;
	}

	unlock();
	return res;
}

// (bms-ble-cell i) -> float
static lbm_value ext_cell(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	int i = lbm_dec_as_i32(args[0]);
	if (i < 0 || i >= BMS_BLE_MAX_CELLS) {
		return ENC_SYM_EERROR;
	}
	return lbm_enc_float(m_data.cells[i]);
}

// (bms-ble-temp i) -> float
static lbm_value ext_temp(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	int i = lbm_dec_as_i32(args[0]);
	if (i < 0 || i >= BMS_BLE_MAX_TEMPS) {
		return ENC_SYM_EERROR;
	}
	return lbm_enc_float(m_data.temps[i]);
}

static lbm_value ext_set_update_vesc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	bms_ble_set_update_vesc(lbm_dec_bool(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_set_send_can(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	bms_ble_set_send_can(lbm_dec_bool(args[0]));
	return ENC_SYM_TRUE;
}

// (bms-ble-stats) -> (reset-reason stage-before-reset current-stage free-heap min-free-heap task-stack-free)
static lbm_value ext_stats(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	lbm_value res = ENC_SYM_NIL;
	res = lbm_cons(lbm_enc_i(m_task ? (int)uxTaskGetStackHighWaterMark(m_task) * (int)sizeof(StackType_t) : -1), res);
	res = lbm_cons(lbm_enc_u32(heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT)), res);
	res = lbm_cons(lbm_enc_u32(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)), res);
	res = lbm_cons(lbm_enc_i(m_stage_rtc & 0xFF), res);
	res = lbm_cons(lbm_enc_i(m_last_stage), res);
	res = lbm_cons(lbm_enc_i(m_reset_reason), res);
	return res;
}

static lbm_value ext_debug(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	bms_ble_set_debug(lbm_dec_bool(args[0]));
	return ENC_SYM_TRUE;
}

void bms_ble_load_extensions(void) {
	lbm_add_symbol_const("auto", &sym_auto);
	lbm_add_symbol_const("jbd", &sym_jbd);
	lbm_add_symbol_const("daly", &sym_daly);
	lbm_add_symbol_const("lipower", &sym_lipower);
	lbm_add_symbol_const("litech", &sym_litech);
	lbm_add_symbol_const("unknown", &sym_unknown);
	lbm_add_symbol_const("disabled", &sym_disabled);
	lbm_add_symbol_const("idle", &sym_idle);
	lbm_add_symbol_const("connecting", &sym_connecting);
	lbm_add_symbol_const("connected", &sym_connected);
	for (int i = 0; i < (int)(sizeof(m_keys) / sizeof(m_keys[0])); i++) {
		lbm_add_symbol_const(m_keys[i].name, &m_keys[i].sym);
	}

	lbm_add_extension("bms-ble-scan", ext_scan);
	lbm_add_extension("bms-ble-scan-stop", ext_scan_stop);
	lbm_add_extension("bms-ble-scanning", ext_scanning);
	lbm_add_extension("bms-ble-scan-results", ext_scan_results);
	lbm_add_extension("bms-ble-connect", ext_connect);
	lbm_add_extension("bms-ble-disconnect", ext_disconnect);
	lbm_add_extension("bms-ble-connected", ext_connected);
	lbm_add_extension("bms-ble-state", ext_state);
	lbm_add_extension("bms-ble-type", ext_type);
	lbm_add_extension("bms-ble-target", ext_target);
	lbm_add_extension("bms-ble-get", ext_get);
	lbm_add_extension("bms-ble-cell", ext_cell);
	lbm_add_extension("bms-ble-temp", ext_temp);
	lbm_add_extension("bms-ble-set-update-vesc", ext_set_update_vesc);
	lbm_add_extension("bms-ble-set-send-can", ext_set_send_can);
	lbm_add_extension("bms-ble-debug", ext_debug);
	lbm_add_extension("bms-ble-stats", ext_stats);
}

#else

void bms_ble_init(void) {}
bool bms_ble_available(void) { return false; }
bool bms_ble_scan_start(float seconds) { (void)seconds; return false; }
void bms_ble_scan_stop(void) {}
bool bms_ble_scan_active(void) { return false; }
int bms_ble_scan_results(bms_ble_scan_entry_t *out, int max) { (void)out; (void)max; return 0; }
bool bms_ble_connect(const uint8_t addr[6], bms_ble_type_t type) { (void)addr; (void)type; return false; }
void bms_ble_disconnect(void) {}
bool bms_ble_is_connected(void) { return false; }
bms_ble_state_t bms_ble_state(void) { return BMS_BLE_STATE_DISABLED; }
bool bms_ble_target(uint8_t addr[6], bms_ble_type_t *type) { (void)addr; (void)type; return false; }
static bms_ble_data_t m_data_stub;
const bms_ble_data_t *bms_ble_get_data(void) { return &m_data_stub; }
void bms_ble_set_update_vesc(bool enabled) { (void)enabled; }
void bms_ble_set_debug(bool enabled) { (void)enabled; }
void bms_ble_set_send_can(bool enabled) { (void)enabled; }
void bms_ble_load_extensions(void) {}

#endif
