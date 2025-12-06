/*
 * custom_ble.h - Stub for NimBLE migration
 */

#ifndef MAIN_BLE_CUSTOM_BLE_H_
#define MAIN_BLE_CUSTOM_BLE_H_

#include <stdbool.h>
#include <stdint.h>

void custom_ble_init(void);
void custom_ble_start(void);
void custom_ble_stop(void);
bool custom_ble_is_enabled(void);
bool custom_ble_started(void);
void custom_ble_set_name(const char *name);
void custom_ble_advertise(void);
void custom_ble_disconnect(void);

#endif
