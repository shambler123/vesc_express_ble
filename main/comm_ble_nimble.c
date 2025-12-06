/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se
	NimBLE port for dual-role BLE support

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

#include "comm_ble.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "packet.h"
#include "commands.h"
#include "conf_general.h"
#include "main.h"

#define TAG "COMM_BLE"
#define GATTS_CHAR_VAL_LEN_MAX 512
#define DEFAULT_BLE_MTU 20

// Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DC9E6E (reversed for NimBLE)
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x6e, 0x9e, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

// RX Characteristic UUID: 6E400002-... (write)
static const ble_uuid128_t gatt_svr_chr_rx_uuid =
    BLE_UUID128_INIT(0x6e, 0x9e, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

// TX Characteristic UUID: 6E400003-... (notify)
static const ble_uuid128_t gatt_svr_chr_tx_uuid =
    BLE_UUID128_INIT(0x6e, 0x9e, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static bool is_connected = false;
static uint16_t ble_current_mtu = DEFAULT_BLE_MTU;
static uint16_t conn_handle = 0;
static uint16_t tx_attr_handle = 0;
static PACKET_STATE_t *packet_state;

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // RX Characteristic (write from client)
                .uuid = &gatt_svr_chr_rx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // TX Characteristic (notify to client)
                .uuid = &gatt_svr_chr_tx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .val_handle = &tx_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, // No more characteristics
            },
        },
    },
    {
        0, // No more services
    },
};

static int gatt_svr_chr_access(uint16_t conn_h, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    
    // RX characteristic - receive data from client
    if (ble_uuid_cmp(uuid, &gatt_svr_chr_rx_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            // Process incoming data
            for (int i = 0; i < OS_MBUF_PKTLEN(ctxt->om); i++) {
                uint8_t byte;
                os_mbuf_copydata(ctxt->om, i, 1, &byte);
                packet_process_byte(byte, packet_state);
            }
            return 0;
        }
    }
    
    // TX characteristic - just return empty for reads
    if (ble_uuid_cmp(uuid, &gatt_svr_chr_tx_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            return 0;
        }
    }
    
    return BLE_ATT_ERR_UNLIKELY;
}

static void ble_advertise(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Connection %s; status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
                is_connected = true;
                ble_current_mtu = DEFAULT_BLE_MTU;
                LED_BLUE_ON();
                
                // Request higher MTU
                ble_att_set_preferred_mtu(512);
                ble_gattc_exchange_mtu(conn_handle, NULL, NULL);
            } else {
                ble_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
            is_connected = false;
            conn_handle = 0;
            LED_BLUE_OFF();
            ble_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertise complete");
            ble_advertise();
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update: %d", event->mtu.value);
            ble_current_mtu = event->mtu.value - 3; // ATT header
            if (ble_current_mtu > GATTS_CHAR_VAL_LEN_MAX) {
                ble_current_mtu = GATTS_CHAR_VAL_LEN_MAX;
            }
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe: handle=%d, cur_notify=%d",
                     event->subscribe.attr_handle,
                     event->subscribe.cur_notify);
            break;

        default:
            break;
    }
    return 0;
}

static void ble_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)backup.config.ble_name;
    fields.name_len = strlen((char *)backup.config.ble_name);
    fields.name_is_complete = 1;
    fields.uuids128 = (ble_uuid128_t[]) { gatt_svr_svc_uuid };
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 0x20;
    adv_params.itvl_max = 0x40;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising: rc=%d", rc);
    }
}

static void ble_on_sync(void) {
    int rc;

    rc = ble_hs_id_infer_auto(0, &ble_svc_gap_device_addr);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type: rc=%d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr_val, NULL);
    ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);

    ble_advertise();
}

static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "BLE reset; reason=%d", reason);
}

static void nimble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void process_packet(unsigned char *data, unsigned int len) {
    commands_process_packet(data, len, comm_ble_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
    if (!is_connected || tx_attr_handle == 0) {
        return;
    }

    uint16_t bytes_sent = 0;
    while (bytes_sent < len) {
        uint16_t chunk = (len - bytes_sent > ble_current_mtu) ? 
                          ble_current_mtu : (len - bytes_sent);
        
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buffer + bytes_sent, chunk);
        if (om) {
            int rc = ble_gatts_notify_custom(conn_handle, tx_attr_handle, om);
            if (rc != 0) {
                ESP_LOGE(TAG, "Notify failed: rc=%d", rc);
                break;
            }
        }
        bytes_sent += chunk;
    }
}

void comm_ble_init(void) {
    int rc;

    packet_state = calloc(1, sizeof(PACKET_STATE_t));
    packet_init(send_packet_raw, process_packet, packet_state);

    // Initialize NVS (required by NimBLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize NimBLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NimBLE: %d", ret);
        return;
    }

    // Configure NimBLE host
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Set device name
    rc = ble_svc_gap_device_name_set((char *)backup.config.ble_name);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting device name: rc=%d", rc);
    }

    // Initialize GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error counting GATT services: rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error adding GATT services: rc=%d", rc);
        return;
    }

    // Set preferred MTU
    ble_att_set_preferred_mtu(512);

    // Start NimBLE host task
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "BLE initialized with NimBLE");
}

bool comm_ble_is_connected(void) {
    return is_connected;
}

int comm_ble_mtu_now(void) {
    return ble_current_mtu;
}

void comm_ble_send_packet(unsigned char *data, unsigned int len) {
    packet_send_packet(data, len, packet_state);
}
