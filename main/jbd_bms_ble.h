/*
 * jbd_bms_ble.h
 *
 * JBD (Jiabaida) BMS BLE Client Driver for VESC Express
 * 
 * This module provides BLE client functionality to connect to and
 * communicate with JBD smart BMS devices over Bluetooth Low Energy.
 *
 * Copyright 2024 - Based on VESC Express codebase
 * Licensed under GPL-3.0
 */

#ifndef JBD_BMS_BLE_H_
#define JBD_BMS_BLE_H_

#include <stdint.h>
#include <stdbool.h>

// JBD BMS BLE Service and Characteristic UUIDs
#define JBD_SERVICE_UUID        "0000ff00-0000-1000-8000-00805f9b34fb"
#define JBD_CHAR_RX_UUID        "0000ff01-0000-1000-8000-00805f9b34fb"  // Notify
#define JBD_CHAR_TX_UUID        "0000ff02-0000-1000-8000-00805f9b34fb"  // Write

// JBD Protocol Commands
#define JBD_CMD_START           0xDD
#define JBD_CMD_END             0x77
#define JBD_CMD_READ            0xA5
#define JBD_CMD_WRITE           0x5A
#define JBD_REG_BASIC           0x03
#define JBD_REG_CELLS           0x04
#define JBD_REG_HARDWARE        0x05

// Maximum supported cells
#define JBD_MAX_CELLS           32
#define JBD_MAX_TEMPS           4

// BMS Data Structure
typedef struct {
    // Connection state
    bool connected;
    uint32_t last_update_ms;
    
    // Basic info (Register 0x03)
    float voltage;              // Pack voltage (V)
    float current;              // Pack current (A), positive = charging
    float remain_capacity;      // Remaining capacity (Ah)
    float nominal_capacity;     // Nominal/design capacity (Ah)
    uint16_t cycles;            // Charge cycle count
    uint8_t soc;                // State of charge (0-100%)
    uint8_t cell_count;         // Number of cells
    uint8_t temp_count;         // Number of temperature sensors
    float temps[JBD_MAX_TEMPS]; // Temperature readings (C)
    uint8_t charge_fet;         // Charge FET status (1=ON)
    uint8_t discharge_fet;      // Discharge FET status (1=ON)
    uint16_t balance_status_lo; // Balance status low word
    uint16_t balance_status_hi; // Balance status high word
    uint16_t protection_status; // Protection status flags
    
    // Cell voltages (Register 0x04)
    float cell_voltages[JBD_MAX_CELLS];
    float cell_min;             // Minimum cell voltage
    float cell_max;             // Maximum cell voltage
    
    // Hardware info (Register 0x05)
    char hw_version[16];
    
    // Statistics
    uint32_t msg_count;
    uint32_t error_count;
} jbd_bms_data_t;

// Function prototypes

// Initialize the JBD BMS BLE client module
void jbd_bms_ble_init(void);

// Connect to a JBD BMS by MAC address string (format: "XX:XX:XX:XX:XX:XX")
bool jbd_bms_ble_connect(const char *mac_addr);

// Disconnect from the BMS
void jbd_bms_ble_disconnect(void);

// Check if connected
bool jbd_bms_ble_is_connected(void);

// Request basic BMS data (async - data available after notification)
bool jbd_bms_ble_request_basic(void);

// Request cell voltage data (async - data available after notification)
bool jbd_bms_ble_request_cells(void);

// Get pointer to current BMS data (read-only)
const jbd_bms_data_t* jbd_bms_ble_get_data(void);

// Copy JBD BMS data to VESC bms_values structure
void jbd_bms_ble_update_vesc_bms(void);

// Load LispBM extensions
void jbd_bms_ble_load_extensions(void);

#endif /* JBD_BMS_BLE_H_ */
