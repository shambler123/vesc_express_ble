/*
 * jbd_bms_ble.c - NimBLE version v9
 * 
 * Try different advertising approach - use random address for advertising
 * while connected as central with public address
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
#include "freertos/timers.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define TAG "JBD_BMS"
#define VESC_PRINT(fmt, ...) commands_printf_lisp(fmt, ##__VA_ARGS__)

#define CONN_TIMEOUT_MS     10000
#define RX_BUFFER_SIZE      256

#define JBD_SVC_UUID        0xFF00
#define JBD_RX_UUID         0xFF01
#define JBD_TX_UUID         0xFF02

static jbd_bms_data_t m_bms_data;
static SemaphoreHandle_t m_data_mutex = NULL;
static TimerHandle_t m_adv_timer = NULL;
static int m_adv_retry_count = 0;

extern void comm_ble_restart_advertising(void);
extern void comm_ble_check_advertising(void);

static struct {
    volatile bool connecting;
    volatile bool connected;
    volatile bool service_found;
    volatile bool subscribed;
    volatile bool connect_done;
    volatile int connect_status;
    uint16_t conn_handle;
    uint16_t svc_start_handle;
    uint16_t svc_end_handle;
    uint16_t rx_handle;
    uint16_t tx_handle;
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

static int gap_event_cb(struct ble_gap_event *event, void *arg);

static bool parse_mac(const char *s, ble_addr_t *addr) {
    if (!s || strlen(s) < 17) return false;
    unsigned int t[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &t[0],&t[1],&t[2],&t[3],&t[4],&t[5]) != 6) return false;
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

// Timer callback for advertising restart
static void adv_timer_cb(TimerHandle_t xTimer) {
    m_adv_retry_count++;
    VESC_PRINT("JBD: Adv timer #%d, adv=%d", m_adv_retry_count, ble_gap_adv_active());
    
    if (ble_gap_adv_active()) {
        VESC_PRINT("JBD: Adv already active!");
        return;
    }
    
    // Try both the restart and the check
    comm_ble_restart_advertising();
    vTaskDelay(pdMS_TO_TICKS(100));
    comm_ble_check_advertising();
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (ble_gap_adv_active()) {
        VESC_PRINT("JBD: Adv started OK!");
    } else if (m_adv_retry_count < 60) {
        // Retry - use longer delays
        int delay = 3000;
        xTimerChangePeriod(m_adv_timer, pdMS_TO_TICKS(delay), 0);
        xTimerStart(m_adv_timer, 0);
    } else {
        VESC_PRINT("JBD: Gave up on advertising after 60 tries");
    }
}

static void schedule_adv_restart(void) {
    m_adv_retry_count = 0;
    if (!m_adv_timer) {
        m_adv_timer = xTimerCreate("adv_timer", pdMS_TO_TICKS(3000), pdFALSE, NULL, adv_timer_cb);
    }
    if (m_adv_timer) {
        VESC_PRINT("JBD: Scheduling adv restart in 3s");
        xTimerStart(m_adv_timer, 0);
    }
}

static int subscribe_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg) {
    if (error->status == 0) {
        VESC_PRINT("JBD: Subscribed! h=%d", conn_handle);
        m_conn.subscribed = true;
        m_conn.connected = true;
        m_conn.connecting = false;
        m_conn.connect_done = true;
        m_conn.connect_status = 0;
        if (m_data_mutex && xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            m_bms_data.connected = true;
            xSemaphoreGive(m_data_mutex);
        }
        VESC_PRINT("JBD: *** BMS CONNECTED (h=%d) ***", m_conn.conn_handle);
        
        // Schedule advertising restart
        schedule_adv_restart();
    } else {
        VESC_PRINT("JBD: Subscribe fail: %d", error->status);
        m_conn.connect_done = true;
        m_conn.connect_status = error->status;
    }
    return 0;
}

static int dsc_disced_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    if (error->status == 0 && dsc != NULL) {
        if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            m_conn.cccd_handle = dsc->handle;
            uint8_t value[2] = {0x01, 0x00};
            ble_gattc_write_flat(conn_handle, dsc->handle, value, 2, subscribe_cb, NULL);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (m_conn.cccd_handle == 0 && m_conn.rx_handle != 0) {
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
        if (uuid16 == JBD_RX_UUID) m_conn.rx_handle = chr->val_handle;
        else if (uuid16 == JBD_TX_UUID) m_conn.tx_handle = chr->val_handle;
    } else if (error->status == BLE_HS_EDONE) {
        VESC_PRINT("JBD: RX=0x%04x TX=0x%04x", m_conn.rx_handle, m_conn.tx_handle);
        if (m_conn.rx_handle != 0) {
            ble_gattc_disc_all_dscs(conn_handle, m_conn.rx_handle,
                                    m_conn.svc_end_handle, dsc_disced_cb, NULL);
        } else {
            m_conn.connect_done = true;
            m_conn.connect_status = -1;
        }
    }
    return 0;
}

static int svc_disced_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_svc *svc, void *arg) {
    if (error->status == 0 && svc != NULL) {
        uint16_t uuid16 = ble_uuid_u16(&svc->uuid.u);
        if (uuid16 == JBD_SVC_UUID) {
            m_conn.service_found = true;
            m_conn.svc_start_handle = svc->start_handle;
            m_conn.svc_end_handle = svc->end_handle;
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (m_conn.service_found) {
            ble_gattc_disc_all_chrs(conn_handle, m_conn.svc_start_handle,
                                    m_conn.svc_end_handle, chr_disced_cb, NULL);
        } else {
            VESC_PRINT("JBD: Service not found");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            m_conn.connect_done = true;
            m_conn.connect_status = -1;
        }
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (!m_conn.connecting) {
                return 0;  // Not our connection
            }
            
            VESC_PRINT("JBD: CONNECT st=%d h=%d", event->connect.status, event->connect.conn_handle);
            if (event->connect.status == 0) {
                m_conn.conn_handle = event->connect.conn_handle;
                VESC_PRINT("JBD: BMS handle=%d", m_conn.conn_handle);
                ble_uuid16_t svc_uuid = BLE_UUID16_INIT(JBD_SVC_UUID);
                ble_gattc_disc_svc_by_uuid(event->connect.conn_handle,
                                           &svc_uuid.u, svc_disced_cb, NULL);
            } else {
                m_conn.connect_done = true;
                m_conn.connect_status = event->connect.status;
                m_conn.connecting = false;
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            if (event->disconnect.conn.conn_handle != m_conn.conn_handle) {
                return 0;  // Not our connection
            }
            VESC_PRINT("JBD: BMS DISCONNECT h=%d", event->disconnect.conn.conn_handle);
            m_conn.connected = false;
            m_conn.connecting = false;
            m_conn.connect_done = true;
            m_conn.conn_handle = 0xFFFF;
            if (m_data_mutex && xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                m_bms_data.connected = false;
                xSemaphoreGive(m_data_mutex);
            }
            break;
            
        case BLE_GAP_EVENT_NOTIFY_RX:
            if (event->notify_rx.conn_handle == m_conn.conn_handle) {
                handle_notify(event->notify_rx.om->om_data, event->notify_rx.om->om_len);
            }
            break;
            
        default:
            break;
    }
    return 0;
}

static void stop_all_gap_activity(void) {
    ble_gap_adv_stop();
    ble_gap_disc_cancel();
    ble_gap_conn_cancel();
    vTaskDelay(pdMS_TO_TICKS(500));
}

static bool do_connect(void) {
    m_conn.target_addr.type = BLE_ADDR_PUBLIC;
    m_conn.connecting = true;
    m_conn.connected = false;
    m_conn.connect_done = false;
    m_conn.connect_status = -1;
    m_conn.service_found = false;
    m_conn.subscribed = false;
    m_conn.rx_handle = 0;
    m_conn.tx_handle = 0;
    m_conn.cccd_handle = 0;
    m_conn.conn_handle = 0xFFFF;
    
    VESC_PRINT("JBD: Connecting...");
    
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &m_conn.target_addr, 10000, NULL, gap_event_cb, NULL);
    
    if (rc != 0) {
        VESC_PRINT("JBD: gap_connect err: %d", rc);
        m_conn.connecting = false;
        return false;
    }
    
    uint32_t start = xTaskGetTickCount();
    while (!m_conn.connect_done && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(CONN_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (m_conn.connected) return true;
    
    if (!m_conn.connect_done) {
        VESC_PRINT("JBD: Timeout, cancelling");
        ble_gap_conn_cancel();
        uint32_t cancel_start = xTaskGetTickCount();
        while (!m_conn.connect_done && (xTaskGetTickCount() - cancel_start) < pdMS_TO_TICKS(2000)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    
    m_conn.connecting = false;
    vTaskDelay(pdMS_TO_TICKS(500));
    return false;
}

bool jbd_bms_ble_connect(const char *mac) {
    VESC_PRINT("JBD: Connect '%s'", mac);
    
    if (!m_data_mutex) m_data_mutex = xSemaphoreCreateMutex();
    
    if (m_conn.connected) {
        VESC_PRINT("JBD: Already connected to BMS");
        return true;
    }
    
    if (!ble_hs_is_enabled()) {
        VESC_PRINT("JBD: BLE host not enabled!");
        return false;
    }
    
    // Check if VESC Tool is connected - if so, skip this attempt
    extern bool comm_ble_is_connected(void);
    if (comm_ble_is_connected()) {
        VESC_PRINT("JBD: VESC Tool connected, skipping BMS connect");
        return false;
    }
    
    memset(&m_conn, 0, sizeof(m_conn));
    m_conn.conn_handle = 0xFFFF;
    memset(&m_rx, 0, sizeof(m_rx));
    
    if (!parse_mac(mac, &m_conn.target_addr)) {
        VESC_PRINT("JBD: Invalid MAC");
        return false;
    }
    
    VESC_PRINT("JBD: Target %02X:%02X:%02X:%02X:%02X:%02X",
               m_conn.target_addr.val[5], m_conn.target_addr.val[4],
               m_conn.target_addr.val[3], m_conn.target_addr.val[2],
               m_conn.target_addr.val[1], m_conn.target_addr.val[0]);
    
    VESC_PRINT("JBD: adv=%d", ble_gap_adv_active());
    
    VESC_PRINT("JBD: Stopping adv...");
    stop_all_gap_activity();
    
    if (do_connect()) {
        VESC_PRINT("JBD: BMS connected!");
        return true;
    }
    
    // Failed - restart advertising immediately
    VESC_PRINT("JBD: Connect failed, restarting adv");
    schedule_adv_restart();
    
    return false;
}

void jbd_bms_ble_disconnect(void) {
    if (m_conn.connected && m_conn.conn_handle != 0xFFFF) {
        ble_gap_terminate(m_conn.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    } else if (m_conn.connecting) {
        ble_gap_conn_cancel();
    }
    m_conn.connected = false;
    m_conn.connecting = false;
}

bool jbd_bms_ble_is_connected(void) { return m_conn.connected; }

static bool send_cmd(const uint8_t *cmd, size_t len) {
    if (!m_conn.connected || m_conn.tx_handle == 0) return false;
    return ble_gattc_write_no_rsp_flat(m_conn.conn_handle, m_conn.tx_handle, cmd, len) == 0;
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
    
    VESC_PRINT("JBD: Loaded (NimBLE v9)");
}