#!/bin/sh
# Full USB flash of a VESC Express T (ESP32-C3): bootloader, partition table,
# OTA data and the app. Settings in NVS are kept (no erase).
# Usage: ./flash_usb.sh [port]   e.g. ./flash_usb.sh /dev/cu.usbmodem1101
set -e
cd "$(dirname "$0")"
PORT="${1:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)}"
[ -n "$PORT" ] || { echo "no USB port found, plug in the Express"; exit 1; }
if ! command -v esptool.py >/dev/null 2>&1 && ! python3 -c "import esptool" 2>/dev/null; then
	echo "esptool missing: run 'source ~/esp/esp-idf-v5.5.4/export.sh' or 'pip3 install esptool'"; exit 1
fi
python3 -m esptool --chip esp32c3 -p "$PORT" -b 460800 --before default_reset --after hard_reset \
	write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m \
	0x0 bootloader.bin 0x8000 partition-table.bin 0xf000 ota_data_initial.bin 0x20000 vesc_express_ble_VESC_Express_T.bin
