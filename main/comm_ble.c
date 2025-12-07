/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se
	NimBLE port for dual-role BLE support - v4

	This file is part of the VESC firmware.
	
	Try using extended advertising or different adv params
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

// Service UUID (same as original Bluedroid)
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

// RX Characteristic UUID (write from VESC Tool)
static const ble_uuid128_t gatt_svr_chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

// TX Characteristic UUID (notify to VESC Tool)
static const ble_uuid128_t gatt_svr_chr_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static bool is_connected = false;
static uint16_t ble_current_mtu = DEFAULT_BLE_MTU;
static uint16_t conn_handle = 0;
static uint16_t tx_attr_handle = 0;
static uint8_t own_addr_type;
static PACKET_STATE_t *packet_state;
static bool ble_initialized = false;
static volatile bool want_advertising = true;
static bool adv_configured = false;

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_advertise(void);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_rx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &gatt_svr_chr_tx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .val_handle = &tx_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gatt_svr_chr_access(uint16_t conn_h, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    
    if (ble_uuid_cmp(uuid, &gatt_svr_chr_rx_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t *data = malloc(om_len);
            if (data) {
                os_mbuf_copydata(ctxt->om, 0, om_len, data);
                for (uint16_t i = 0; i < om_len; i++) {
                    packet_process_byte(data[i], packet_state);
                }
                free(data);
            }
            return 0;
        }
    }
    
    if (ble_uuid_cmp(uuid, &gatt_svr_chr_tx_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            return 0;
        }
    }
    
    return BLE_ATT_ERR_UNLIKELY;
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Connection %s; status=%d handle=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status, event->connect.conn_handle);
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
                is_connected = true;
                ble_current_mtu = DEFAULT_BLE_MTU;
                LED_BLUE_ON();
                ble_att_set_preferred_mtu(512);
                ble_gattc_exchange_mtu(conn_handle, NULL, NULL);
            } else {
                ble_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected; reason=%d handle=%d", 
                     event->disconnect.reason,
                     event->disconnect.conn.conn_handle);
            is_connected = false;
            conn_handle = 0;
            LED_BLUE_OFF();
            ble_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertise complete");
            if (!is_connected) {
                ble_advertise();
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update: %d", event->mtu.value);
            ble_current_mtu = event->mtu.value - 3;
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

static void configure_adv_data(void) {
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)backup.config.ble_name;
    fields.name_len = strlen((char *)backup.config.ble_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        commands_printf_lisp("COMM: adv_set_fields err=%d", rc);
        return;
    }

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.uuids128 = (ble_uuid128_t[]) { gatt_svr_svc_uuid };
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        commands_printf_lisp("COMM: adv_rsp_set_fields err=%d", rc);
    }
    
    adv_configured = true;
}

static void ble_advertise(void) {
    struct ble_gap_adv_params adv_params;
    int rc;

    commands_printf_lisp("COMM: ble_advertise()");

    if (ble_gap_adv_active()) {
        commands_printf_lisp("COMM: Already advertising");
        return;
    }
    
    // Configure adv data only once
    if (!adv_configured) {
        configure_adv_data();
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 0x20;  // 20ms
    adv_params.itvl_max = 0x40;  // 40ms
    adv_params.channel_map = 0;  // Use all channels
    adv_params.filter_policy = BLE_HCI_ADV_FILT_NONE;
    adv_params.high_duty_cycle = 0;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        commands_printf_lisp("COMM: adv_start err=%d", rc);
    } else {
        commands_printf_lisp("COMM: Advertising started OK");
    }
}

static void ble_on_sync(void) {
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type: rc=%d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);

    ble_advertise();
    ble_initialized = true;
}

static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "BLE reset; reason=%d", reason);
    adv_configured = false;  // Need to reconfigure after reset
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

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NimBLE: %d", ret);
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = ble_svc_gap_device_name_set((char *)backup.config.ble_name);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting device name: rc=%d", rc);
    }

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

    ble_att_set_preferred_mtu(512);
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

bool comm_ble_is_initialized(void) {
    return ble_initialized;
}

// Public function to restart advertising
void comm_ble_restart_advertising(void) {
    want_advertising = true;
    commands_printf_lisp("COMM: restart_adv init=%d adv=%d srvr_conn=%d", 
                         ble_initialized, ble_gap_adv_active(), is_connected);
    if (ble_initialized && !ble_gap_adv_active()) {
        ble_advertise();
    }
}

void comm_ble_check_advertising(void) {
    if (want_advertising && ble_initialized && !ble_gap_adv_active() && !is_connected) {
        ble_advertise();
    }
}