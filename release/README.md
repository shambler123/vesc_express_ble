# Release files

| File | Install |
|------|---------|
| `vesc_express_ble_VESC_Express_T.bin` | VESC Tool → Firmware → Custom File, with the VESC Express selected (built for HW "VESC Express T", ESP32-C3, IDF 5.5.4) |
| `float_accessories_v3.5.30_bms_ble.vescpkg` | VESC Tool → VESC Packages → Load Custom |

After flashing: set BLE mode to Open or Encrypted in the VESC Express settings, install the package, enable BMS in the package, then scan and pick the BMS in the Bluetooth BMS group of the BMS config tab. See `doc/bms_ble.md`.
