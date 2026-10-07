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
 * BLE central (GATT client) that connects to an external smart BMS and
 * mirrors its values into the VESC bms_values structure. Runs next to the
 * normal VESC Tool BLE link (GATT server), both links can be open at the same
 * time.
 *
 * Supported BMS types:
 *   - JBD / Jiabaida / Overkill Solar   (service 0xFF00, notify 0xFF01, write 0xFF02)
 *   - Daly smart BMS                     (service 0xFFF0, notify 0xFFF1, write 0xFFF2)
 *     Protocol variants: Modbus 0xD2, Modbus 0x81 (newer K/H series), legacy 0xA5
 *   - LiPower / Ective                   (service 0xFFE0, notify+write 0xFFE1)
 *   - LiTech "BT-BMS-xxxx"               (Nordic UART service, Modbus RTU slave 1, live data at 0xD000)
 *   - JK / Jikong                        (service 0xFFE0, notify+write 0xFFE1, JK02 24S/32S records)
 *   - ANT                                (service 0xFFE0, notify+write 0xFFE1, 0x7EA1 protocol or legacy 0xDBDB)
 *   - Stoked Stock / Indy Speed Control  (service 00002760-08c2-11e1-9073-0e8ac72e1001, write ...e0001,
 *     "SSBMS"                             notify ...e0002, Modbus-like slave 0x16 with CRC-16/XMODEM)
 *
 * 0xFFE0 is shared by LiPower, JK and ANT. With type 'auto the protocols are
 * probed in the order JK, ANT, ANT legacy, LiPower after the connection is up.
 */

#ifndef MAIN_BMS_BLE_H_
#define MAIN_BMS_BLE_H_

#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"

#define BMS_BLE_MAX_CELLS		32
#define BMS_BLE_MAX_TEMPS		8
#define BMS_BLE_SCAN_MAX		16
#define BMS_BLE_NAME_LEN		24

typedef enum {
	BMS_BLE_TYPE_AUTO = 0,
	BMS_BLE_TYPE_JBD,
	BMS_BLE_TYPE_DALY,
	BMS_BLE_TYPE_LIPOWER,
	BMS_BLE_TYPE_LITECH,
	BMS_BLE_TYPE_JK,
	BMS_BLE_TYPE_ANT,
	BMS_BLE_TYPE_SSBMS,
	BMS_BLE_TYPE_UNKNOWN,
} bms_ble_type_t;

typedef enum {
	BMS_BLE_STATE_DISABLED = 0,
	BMS_BLE_STATE_IDLE,
	BMS_BLE_STATE_CONNECTING,
	BMS_BLE_STATE_CONNECTED,
} bms_ble_state_t;

typedef struct {
	uint8_t addr[6];
	uint8_t addr_type;
	int8_t rssi;
	bms_ble_type_t type;
	char name[BMS_BLE_NAME_LEN];
	uint32_t last_seen;
} bms_ble_scan_entry_t;

typedef struct {
	bms_ble_type_t type;
	int proto_variant;
	bool valid;
	uint32_t update_time;
	uint32_t msg_count;
	uint32_t err_count;

	float voltage;
	float current;			// A, positive = charging
	float soc;				// 0.0 - 1.0
	float soh;				// 0.0 - 1.0
	float ah_remain;
	float ah_nominal;
	uint32_t cycles;
	uint32_t runtime_s;

	int cell_count;
	float cells[BMS_BLE_MAX_CELLS];
	float cell_min;
	float cell_max;

	int temp_count;
	float temps[BMS_BLE_MAX_TEMPS];
	float temp_mos;
	bool temp_mos_valid;

	bool chg_fet;
	bool dis_fet;
	uint32_t balance_bits;
	uint64_t problem;
} bms_ble_data_t;

#if CONFIG_BT_BLUEDROID_ENABLED
#include "esp_gap_ble_api.h"
// Must be called from the GAP callback of the BLE server so that scan
// events reach this module (Bluedroid only has one GAP callback).
void bms_ble_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
#endif

// Call after bluedroid has been enabled
void bms_ble_init(void);
bool bms_ble_available(void);

bool bms_ble_scan_start(float seconds);
void bms_ble_scan_stop(void);
bool bms_ble_scan_active(void);
int bms_ble_scan_results(bms_ble_scan_entry_t *out, int max);

bool bms_ble_connect(const uint8_t addr[6], bms_ble_type_t type);
void bms_ble_disconnect(void);
bool bms_ble_is_connected(void);
bms_ble_state_t bms_ble_state(void);
bool bms_ble_target(uint8_t addr[6], bms_ble_type_t *type);

const bms_ble_data_t *bms_ble_get_data(void);
void bms_ble_set_update_vesc(bool enabled);
void bms_ble_set_debug(bool enabled);
void bms_ble_set_send_can(bool enabled);

void bms_ble_load_extensions(void);

#endif /* MAIN_BMS_BLE_H_ */
