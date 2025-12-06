#include "ble/custom_ble.h"

void custom_ble_init(void) {}
void custom_ble_start(void) {}
void custom_ble_stop(void) {}
bool custom_ble_is_enabled(void) { return false; }
bool custom_ble_started(void) { return false; }
void custom_ble_set_name(const char *name) { (void)name; }
void custom_ble_advertise(void) {}
void custom_ble_disconnect(void) {}
