/*
 * custom_ble.h - Stub for NimBLE migration
 * This disables custom BLE functionality when using NimBLE
 */

#ifndef MAIN_BLE_CUSTOM_BLE_H_
#define MAIN_BLE_CUSTOM_BLE_H_

#include <stdbool.h>
#include <stdint.h>

// Stub functions - do nothing with NimBLE
static inline void custom_ble_init(void) {}
static inline void custom_ble_start(void) {}
static inline void custom_ble_stop(void) {}
static inline bool custom_ble_is_enabled(void) { return false; }

#endif /* MAIN_BLE_CUSTOM_BLE_H_ */
