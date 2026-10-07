# Release files

| File | Install |
|------|---------|
| `vesc_express_ble_VESC_Express_T.bin` | VESC Tool → Firmware → Custom File, with the VESC Express selected (built for HW "VESC Express T", ESP32-C3, IDF 5.5.4) |
| `float_accessories_v3.5.33_bms_ble.vescpkg` | VESC Tool → VESC Packages → Load Custom |
| `float_accessories_native_4.0.0_blinker_horn_blebms.vescpkg` | Same, but the techfoundrynz native-lib branch (`feat/float-accessories-native`, needs FW 7.00) with blinker, horn, pubmote buttons and Bluetooth BMS added as separate commits: branch `feat/float-accessories-native` in github.com/shambler123/vesc_float_accessories_blinker_horn. Settings are stored as a VESC custom config, so they do not carry over from the 3.5.x package. |

After flashing: set BLE mode to Open or Encrypted in the VESC Express settings, install the package, enable BMS in the package, then scan and pick the BMS in the Bluetooth BMS group of the BMS config tab. See `doc/bms_ble.md`.

## Flashing over USB

Normal updates: connect the Express by USB, VESC Tool → Connection → USB/Serial, then Firmware → Custom File as above. Faster than BLE, settings are kept.

Full flash with esptool (bootloader + partition table + app), e.g. when the device does not boot or after a wrong build:

```
source ~/esp/esp-idf-v5.5.4/export.sh
./flash_usb.sh                 # auto-detects /dev/cu.usbmodem*
```

If the port is not found, hold the BOOT button while plugging in the USB cable to enter the ROM bootloader. `flash_usb.sh` does not erase NVS, so the Express config and the package settings survive. To wipe everything: `python3 -m esptool --chip esp32c3 -p <port> erase_flash` first.
