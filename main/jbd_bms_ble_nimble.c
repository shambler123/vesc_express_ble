/*
 * jbd_bms_ble.c - NimBLE version
 * 
 * JBD BMS BLE Client for VESC Express using NimBLE stack
 * This version works alongside the NimBLE GATTS server
 */

#include "jbd_bms_ble.h"
#include "bms.h"
#include "lispif.h"
#include "lispbm.h"
#include "commands.h"

#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define TAG "JBD_BMS"
#define VESC_PRINT(fmt, ...) commands_printf_lisp(fmt, ##__VA_ARGS__)

#define CONN_TIMEOUT_MS     15000
#define RX_BUFFER_SIZE      256

// JBD BMS Service and Characteristics (16-bit UUIDs)
static const ble_uuid16_t jbd_svc_uuid = BLE_UUID16_INIT(0xFF00);
static const ble_uuid16_t jbd_rx_uuid = BLE_UUID16_INIT(0xFF01);  // Notify
static const ble_uuid16_t jbd_tx_uuid = BLE_UUID16_INIT(0xFF02);  // Write

static jbd_bms_data_t m_bms_data;
static SemaphoreHandle_t m_data_mutex = NULL;

static struct {
    bool connecting;
    bool connected;
    bool service_found;
    bool subscribed;
    uint16_t conn_handle;
    uint16_t svc_start_handle;
    uint16_t svc_end_handle;
    uint16_t rx_handle;  // For notifications (read from BMS)
    uint16_t tx_handle;  // For writes (send to BMS)
    uint16_t cccd_handle;
    ble_addr_t target_addr;
} m_conn;

static struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    int idx;
    bool receiving;
    uint8_t expected_reg;
    int expected_len;
} m_rx;

static const uint8_t CMD_BASIC[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
static const uint8_t CMD_CELLS[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

// Forward declarations
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static int chr_disced_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_chr *chr, void *arg);
static int svc_disced_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_svc *svc, void *arg);

static bool parse_mac(const char *s, ble_addr_t *addr) {
    if (!s || strlen(s) < 17) return false;
    unsigned int t[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &t[0],&t[1],&t[2],&t[3],&t[4],&t[5]) != 6) return false;
    // NimBLE uses reversed byte order for address
    for (int i = 0; i < 6; i++) {
        addr->val[5-i] = (uint8_t)t[i];
    }
    return true;
}

static void parse_basic(void) {
    if (m_rx.idx < 30) return;
    uint8_t *b = m_rx.buffer;
    if (!m_data_mutex || xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    
    m_bms_data.voltage = ((b[4] << 8) | b[5]) / 100.0f;
    m_bms_data.current = (int16_t)((b[6] << 8) | b[7]) / 100.0f;
    m_bms_data.remain_capacity = ((b[8] << 8) | b[9]) / 100.0f;
    m_bms_data.nominal_capacity = ((b[10] << 8) | b[11]) / 100.0f;
    m_bms_data.cycles = (b[12] << 8) | b[13];
    m_bms_data.soc = b[23];
    m_bms_data.charge_fet = (b[24] & 0x02) ? 1 : 0;
    m_bms_data.discharge_fet = (b[24] & 0x01) ? 1 : 0;
    m_bms_data.cell_count = b[25];
    m_bms_data.temp_count = b[26];
    if (m_bms_data.temp_count > JBD_MAX_TEMPS) m_bms_data.temp_count = JBD_MAX_TEMPS;
    
    for (int i = 0; i < m_bms_data.temp_count && (27 + i*2 + 1) < m_rx.idx; i++) {
        uint16_t tr = (b[27 + i*2] << 8) | b[27 + i*2 + 1];
        m_bms_data.temps[i] = (tr - 2731) / 10.0f;
    }
    
    m_bms_data.connected = true;
    m_bms_data.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    m_bms_data.msg_count++;
    xSemaphoreGive(m_data_mutex);
    
    VESC_PRINT("JBD: V=%.2f I=%.2f SOC=%d%%", m_bms_data.voltage, m_bms_data.current, m_bms_data.soc);
}

static void parse_cells(void) {
    if (m_rx.idx < 7) return;
    uint8_t *b = m_rx.buffer;
    uint8_t nc = b[3] / 2;
    if (nc > JBD_MAX_CELLS) nc = JBD_MAX_CELLS;
    if (!m_data_mutex || xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    
    m_bms_data.cell_min = 5.0f;
    m_bms_data.cell_max = 0.0f;
    for (int i = 0; i < nc; i++) {
        int idx = 4 + i * 2;
        if (idx + 1 >= m_rx.idx) break;
        float v = ((b[idx] << 8) | b[idx + 1]) / 1000.0f;
        m_bms_data.cell_voltages[i] = v;
        if (v < m_bms_data.cell_min && v > 0.5f) m_bms_data.cell_min = v;
        if (v > m_bms_data.cell_max) m_bms_data.cell_max = v;
    }
    m_bms_data.msg_count++;
    xSemaphoreGive(m_data_mutex);
}

static void handle_notify(const uint8_t *data, size_t len) {
    VESC_PRINT("JBD: RX %d bytes", len);
    
    if (len >= 4 && data[0] == 0xDD) {
        m_rx.receiving = true;
        m_rx.idx = 0;
        m_rx.expected_reg = data[1];
        m_rx.expected_len = data[3] + 7;
    }
    if (m_rx.receiving) {
        for (size_t i = 0; i < len && m_rx.idx < RX_BUFFER_SIZE; i++) {
            m_rx.buffer[m_rx.idx++] = data[i];
        }
        if (data[len - 1] == 0x77 && m_rx.idx >= m_rx.expected_len) {
            if (m_rx.expected_reg == 0x03) parse_basic();
            else if (m_rx.expected_reg == 0x04) parse_cells();
            m_rx.receiving = false;
            m_rx.idx = 0;
        }
    }
}

static int subscribe_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg) {
    if (error->status == 0) {
        VESC_PRINT("JBD: Subscribed to notifications");
        m_conn.subscribed = true;
        m_conn.connected = true;
        m_conn.connecting = false;
        if (m_data_mutex && xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            m_bms_data.connected = true;
            xSemaphoreGive(m_data_mutex);
        }
        VESC_PRINT("JBD: *** CONNECTED ***");
    } else {
        VESC_PRINT("JBD: Subscribe failed: %d", error->status);
    }
    return 0;
}

static int dsc_disced_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    if (error->status == 0 && dsc != NULL) {
        if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            VESC_PRINT("JBD: CCCD handle=0x%04x", dsc->handle);
            m_conn.cccd_handle = dsc->handle;
            
            // Subscribe to notifications
            uint8_t value[2] = {0x01, 0x00}; // Enable notifications
            int rc = ble_gattc_write_flat(conn_handle, dsc->handle, value, 2, subscribe_cb, NULL);
            if (rc != 0) {
                VESC_PRINT("JBD: Write CCCD failed: %d", rc);
            }
        }
    } else if (error->status == BLE_HS_EDONE) {
        // Discovery complete
        if (m_conn.cccd_handle == 0) {
            VESC_PRINT("JBD: CCCD not found, trying direct subscribe");
            // Try subscribing anyway
            uint8_t value[2] = {0x01, 0x00};
            ble_gattc_write_flat(conn_handle, m_conn.rx_handle + 1, value, 2, subscribe_cb, NULL);
        }
    }
    return 0;
}

static int chr_disced_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != NULL) {
        uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);
        VESC_PRINT("JBD: CHR uuid=0x%04x handle=0x%04x", uuid16, chr->val_handle);
        
        if (uuid16 == 0xFF01) {
            m_conn.rx_handle = chr->val_handle;
            VESC_PRINT("JBD: RX (notify) handle=0x%04x", chr->val_handle);
        } else if (uuid16 == 0xFF02) {
            m_conn.tx_handle = chr->val_handle;
            VESC_PRINT("JBD: TX (write) handle=0x%04x", chr->val_handle);
        }
    } else if (error->status == BLE_HS_EDONE) {
        VESC_PRINT("JBD: CHR discovery done, RX=0x%04x TX=0x%04x", m_conn.rx_handle, m_conn.tx_handle);
        
        if (m_conn.rx_handle != 0) {
            // Discover descriptors to find CCCD
            int rc = ble_gattc_disc_all_dscs(conn_handle, m_conn.rx_handle,
                                              m_conn.svc_end_handle, dsc_disced_cb, NULL);
            if (rc != 0) {
                VESC_PRINT("JBD: DSC discovery failed: %d", rc);
            }
        } else {
            VESC_PRINT("JBD: RX characteristic not found!");
            m_conn.connecting = false;
        }
    } else {
        VESC_PRINT("JBD: CHR discovery error: %d", error->status);
    }
    return 0;
}

static int svc_disced_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_svc *svc, void *arg) {
    if (error->status == 0 && svc != NULL) {
        uint16_t uuid16 = ble_uuid_u16(&svc->uuid.u);
        VESC_PRINT("JBD: SVC uuid=0x%04x handles=%d-%d", uuid16, svc->start_handle, svc->end_handle);
        
        if (uuid16 == 0xFF00) {
            m_conn.service_found = true;
            m_conn.svc_start_handle = svc->start_handle;
            m_conn.svc_end_handle = svc->end_handle;
        }
    } else if (error->status == BLE_HS_EDONE) {
        VESC_PRINT("JBD: SVC discovery done, found=%d", m_conn.service_found);
        
        if (m_conn.service_found) {
            // Discover characteristics
            int rc = ble_gattc_disc_all_chrs(conn_handle, m_conn.svc_start_handle,
                                              m_conn.svc_end_handle, chr_disced_cb, NULL);
            if (rc != 0) {
                VESC_PRINT("JBD: CHR discovery failed: %d", rc);
                m_conn.connecting = false;
            }
        } else {
            VESC_PRINT("JBD: Service 0xFF00 not found!");
            m_conn.connecting = false;
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
    } else {
        VESC_PRINT("JBD: SVC discovery error: %d", error->status);
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            VESC_PRINT("JBD: GAP connect status=%d", event->connect.status);
            if (event->connect.status == 0) {
                m_conn.conn_handle = event->connect.conn_handle;
                VESC_PRINT("JBD: Connected, handle=%d", m_conn.conn_handle);
                
                // Start service discovery
                int rc = ble_gattc_disc_svc_by_uuid(event->connect.conn_handle,
                                                    &jbd_svc_uuid.u, svc_disced_cb, NULL);
                if (rc != 0) {
                    VESC_PRINT("JBD: SVC discovery start failed: %d", rc);
                    m_conn.connecting = false;
                }
            } else {
                VESC_PRINT("JBD: Connection failed: %d", event->connect.status);
                m_conn.connecting = false;
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            VESC_PRINT("JBD: Disconnected reason=%d", event->disconnect.reason);
            m_conn.connected = false;
            m_conn.connecting = false;
            m_conn.service_found = false;
            m_conn.subscribed = false;
            m_conn.rx_handle = 0;
            m_conn.tx_handle = 0;
            m_conn.cccd_handle = 0;
            if (m_data_mutex && xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                m_bms_data.connected = false;
                xSemaphoreGive(m_data_mutex);
            }
            break;
            
        case BLE_GAP_EVENT_NOTIFY_RX:
            handle_notify(event->notify_rx.om->om_data, event->notify_rx.om->om_len);
            break;
            
        case BLE_GAP_EVENT_MTU:
            VESC_PRINT("JBD: MTU updated to %d", event->mtu.value);
            break;
            
        default:
            VESC_PRINT("JBD: GAP event %d", event->type);
            break;
    }
    return 0;
}

bool jbd_bms_ble_connect(const char *mac) {
    VESC_PRINT("JBD: Connect to '%s'", mac);
    
    if (!m_data_mutex) {
        m_data_mutex = xSemaphoreCreateMutex();
        if (!m_data_mutex) {
            VESC_PRINT("JBD: Mutex create failed");
            return false;
        }
    }
    
    if (m_conn.connected) {
        VESC_PRINT("JBD: Already connected");
        return true;
    }
    if (m_conn.connecting) {
        VESC_PRINT("JBD: Already connecting");
        return false;
    }
    
    memset(&m_conn, 0, sizeof(m_conn));
    memset(&m_rx, 0, sizeof(m_rx));
    
    if (!parse_mac(mac, &m_conn.target_addr)) {
        VESC_PRINT("JBD: Invalid MAC");
        return false;
    }
    
    // Try RANDOM address type first (most JBD BMS use random static addresses)
    m_conn.target_addr.type = BLE_ADDR_RANDOM;
    m_conn.connecting = true;
    
    VESC_PRINT("JBD: Connecting (RANDOM)...");
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &m_conn.target_addr,
                              30000, NULL, gap_event_cb, NULL);
    
    if (rc != 0) {
        VESC_PRINT("JBD: Connect call failed: %d, trying PUBLIC", rc);
        
        // Try PUBLIC address type
        m_conn.target_addr.type = BLE_ADDR_PUBLIC;
        rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &m_conn.target_addr,
                              30000, NULL, gap_event_cb, NULL);
        if (rc != 0) {
            VESC_PRINT("JBD: Connect failed: %d", rc);
            m_conn.connecting = false;
            return false;
        }
    }
    
    // Wait for connection
    uint32_t start = xTaskGetTickCount();
    while (m_conn.connecting && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(CONN_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (m_conn.connected) {
        return true;
    }
    
    if (m_conn.connecting) {
        VESC_PRINT("JBD: Timeout");
        ble_gap_terminate(m_conn.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        m_conn.connecting = false;
    }
    
    return false;
}

void jbd_bms_ble_disconnect(void) {
    if (m_conn.connected || m_conn.connecting) {
        ble_gap_terminate(m_conn.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    m_conn.connected = false;
    m_conn.connecting = false;
}

bool jbd_bms_ble_is_connected(void) {
    return m_conn.connected;
}

static bool send_cmd(const uint8_t *cmd, size_t len) {
    if (!m_conn.connected || m_conn.tx_handle == 0) {
        VESC_PRINT("JBD: Not ready to send");
        return false;
    }
    
    int rc = ble_gattc_write_no_rsp_flat(m_conn.conn_handle, m_conn.tx_handle, cmd, len);
    if (rc != 0) {
        VESC_PRINT("JBD: Write failed: %d", rc);
        return false;
    }
    return true;
}

bool jbd_bms_ble_request_basic(void) { return send_cmd(CMD_BASIC, sizeof(CMD_BASIC)); }
bool jbd_bms_ble_request_cells(void) { return send_cmd(CMD_CELLS, sizeof(CMD_CELLS)); }
const jbd_bms_data_t* jbd_bms_ble_get_data(void) { return &m_bms_data; }

void jbd_bms_ble_update_vesc_bms(void) {
    if (!m_bms_data.connected) return;
    volatile bms_values *bms = bms_get_values();
    if (!m_data_mutex || xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    
    bms->v_tot = m_bms_data.voltage;
    bms->i_in = m_bms_data.current;
    bms->i_in_ic = m_bms_data.current;
    bms->ah_cnt = m_bms_data.remain_capacity;
    bms->wh_cnt = m_bms_data.remain_capacity * m_bms_data.voltage;
    bms->cell_num = m_bms_data.cell_count;
    bms->soc = m_bms_data.soc / 100.0f;
    bms->soh = 1.0f;
    for (int i = 0; i < m_bms_data.cell_count && i < BMS_MAX_CELLS; i++) {
        bms->v_cell[i] = m_bms_data.cell_voltages[i];
    }
    bms->v_cell_min = m_bms_data.cell_min;
    bms->v_cell_max = m_bms_data.cell_max;
    bms->temp_adc_num = m_bms_data.temp_count;
    for (int i = 0; i < m_bms_data.temp_count && i < BMS_MAX_TEMPS; i++) {
        bms->temps_adc[i] = m_bms_data.temps[i];
    }
    bms->update_time = xTaskGetTickCount();
    xSemaphoreGive(m_data_mutex);
}

// LispBM Extensions
static lbm_value ext_connect(lbm_value *args, lbm_uint argn) {
    if (argn != 1 || !lbm_is_array_r(args[0])) return ENC_SYM_EERROR;
    lbm_array_header_t *arr = (lbm_array_header_t *)lbm_car(args[0]);
    return jbd_bms_ble_connect((const char *)arr->data) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_disconnect(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; jbd_bms_ble_disconnect(); return ENC_SYM_TRUE;
}

static lbm_value ext_connected(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return jbd_bms_ble_is_connected() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_req_basic(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return jbd_bms_ble_request_basic() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_req_cells(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return jbd_bms_ble_request_cells() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_voltage(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_float(m_bms_data.voltage);
}

static lbm_value ext_current(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_float(m_bms_data.current);
}

static lbm_value ext_soc(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_i(m_bms_data.soc);
}

static lbm_value ext_cell_count(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_i(m_bms_data.cell_count);
}

static lbm_value ext_cell_v(lbm_value *args, lbm_uint argn) {
    if (argn != 1) return ENC_SYM_EERROR;
    int c = lbm_dec_as_i32(args[0]);
    if (c < 0 || c >= JBD_MAX_CELLS) return lbm_enc_float(0.0f);
    return lbm_enc_float(m_bms_data.cell_voltages[c]);
}

static lbm_value ext_cell_min(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_float(m_bms_data.cell_min);
}

static lbm_value ext_cell_max(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_float(m_bms_data.cell_max);
}

static lbm_value ext_temp_bms(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_float(m_bms_data.temp_count > 0 ? m_bms_data.temps[0] : 0.0f);
}

static lbm_value ext_temp_bat(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn;
    return lbm_enc_float((m_bms_data.temp_count > 1) ? m_bms_data.temps[1] : 
                         (m_bms_data.temp_count > 0) ? m_bms_data.temps[0] : 0.0f);
}

static lbm_value ext_cycles(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_i(m_bms_data.cycles);
}

static lbm_value ext_cap(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_float(m_bms_data.remain_capacity);
}

static lbm_value ext_nom_cap(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; return lbm_enc_float(m_bms_data.nominal_capacity);
}

static lbm_value ext_update_vesc(lbm_value *args, lbm_uint argn) {
    (void)args; (void)argn; jbd_bms_ble_update_vesc_bms(); return ENC_SYM_TRUE;
}

void jbd_bms_ble_load_extensions(void) {
    lbm_add_extension("jbd-bms-connect", ext_connect);
    lbm_add_extension("jbd-bms-disconnect", ext_disconnect);
    lbm_add_extension("jbd-bms-connected", ext_connected);
    lbm_add_extension("jbd-bms-request-basic", ext_req_basic);
    lbm_add_extension("jbd-bms-request-cells", ext_req_cells);
    lbm_add_extension("jbd-bms-get-voltage", ext_voltage);
    lbm_add_extension("jbd-bms-get-current", ext_current);
    lbm_add_extension("jbd-bms-get-soc", ext_soc);
    lbm_add_extension("jbd-bms-get-cell-count", ext_cell_count);
    lbm_add_extension("jbd-bms-get-cell-voltage", ext_cell_v);
    lbm_add_extension("jbd-bms-get-cell-min", ext_cell_min);
    lbm_add_extension("jbd-bms-get-cell-max", ext_cell_max);
    lbm_add_extension("jbd-bms-get-temp-bms", ext_temp_bms);
    lbm_add_extension("jbd-bms-get-temp-bat", ext_temp_bat);
    lbm_add_extension("jbd-bms-get-cycles", ext_cycles);
    lbm_add_extension("jbd-bms-get-capacity", ext_cap);
    lbm_add_extension("jbd-bms-get-nominal-cap", ext_nom_cap);
    lbm_add_extension("jbd-bms-update-vesc-bms", ext_update_vesc);
    
    VESC_PRINT("JBD: Extensions loaded (NimBLE)");
}
