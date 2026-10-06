# External BMS over BLE

VESC Express can connect to a smart BMS over Bluetooth Low Energy and mirror
its values into the VESC BMS data, so VESC Tool shows the pack on its BMS page
and LispBM scripts can read it with `get-bms-val`.

The BMS link is a second, independent BLE connection. The normal VESC Tool BLE
connection keeps working at the same time, both links share the radio.

## Supported BMS

| Type      | Service / characteristics             | Protocol                                                             |
|-----------|----------------------------------------|----------------------------------------------------------------------|
| `jbd`     | 0xFF00, notify 0xFF01, write 0xFF02    | JBD / Jiabaida / Overkill Solar, commands 0x03 and 0x04              |
| `daly`    | 0xFFF0, notify 0xFFF1, write 0xFFF2    | Daly Modbus 0xD2 (most BLE dongles), Modbus 0x81 (K/H series), legacy 0xA5 frames. The variant is detected automatically. |
| `lipower` | 0xFFE0, notify + write 0xFFE1          | LiPower / Ective, Modbus register 0x0400. Only pack values, no cell voltages. |
| `jk`      | 0xFFE0, notify + write 0xFFE1          | JK / Jikong JK02 records (300 bytes). Device info is read first, the layout depends on the firmware version (24S below V11, 32S from V11). Names `JK-*`, manufacturer ids 0x0B65 / 0x4B4A. |
| `ant`     | 0xFFE0, notify + write 0xFFE1          | ANT, protocol `7E A1` (status 0x01) and legacy `DB DB` (140 byte status), auto-detected. Names `ANT*`. |
| `litech`  | Nordic UART 6e400001, notify 6e400003, write 6e400002 | LiTech "BT-BMS-xxxx" (Silicon Labs module), Modbus RTU slave 1, live block 0xD000..0xD03A: 32 cell registers (0xEE49 = unused), pack voltage, 4 temperatures, SOC, SOH, remaining / design capacity, cycles. The current register is not verified yet, 0xD032 is used. |

0xFFE0 is shared by LiPower, JK and ANT. With type `auto` the firmware probes
JK, ANT and LiPower in that order once the connection is up, so a scan result
that only shows `lipower` for an unnamed 0xFFE0 device still connects to the
right protocol.

## How it works

* `bms_ble.c` registers a GATT client next to the GATT server used by VESC
  Tool. It needs `BLE mode` set to *Open*, *Encrypted* or *Scripting* in the
  VESC Express settings (in scripting mode the client becomes available after
  `ble-start`).
* `(bms-ble-connect mac type)` stores a target. A supervisor task scans for the
  address, connects, discovers the service, subscribes to notifications and
  polls the BMS once per second. On any disconnect it reconnects after 3 s.
* Scanning follows the policy that jeremym8884 used on the ESP32 display:
  60 % listening time (30 ms window / 50 ms interval) when nothing is connected,
  10 % (10 ms / 100 ms) as soon as the phone or the BMS is attached. The
  automatic scan stops once the BMS is connected, the two links then simply
  take turns. The BMS link asks for a 30 to 60 ms connection interval to leave
  most of the air-time to VESC Tool.
* Values are written into `bms_values` after every successful poll
  (`v_tot`, `i_in`, `ah_cnt`, `wh_cnt`, cells, temperatures, `soc`, FET state).
  This can be disabled with `(bms-ble-set-update-vesc nil)`.

## LispBM extensions

```
(bms-ble-scan [seconds])        ; start a scan, returns immediately
(bms-ble-scanning)              ; t while the scan runs
(bms-ble-scan-stop)
(bms-ble-scan-results)          ; ((name mac rssi type) ...), type is 'jbd 'daly 'lipower 'litech 'jk 'ant or 'unknown
                                ; detected from the advertised service UUID, the Daly manufacturer id or the
                                ; name prefix (JBD-, DL-, BT-BMS-)
(bms-ble-connect mac [type])    ; mac as "A5:C2:37:17:C7:1A", type 'auto (default) 'jbd 'daly 'lipower 'litech 'jk 'ant
(bms-ble-disconnect)            ; disconnect and stop reconnecting
(bms-ble-connected)
(bms-ble-state)                 ; 'disabled 'idle 'connecting 'connected
(bms-ble-type)
(bms-ble-target)                ; (mac type) or nil
(bms-ble-get key)               ; see below
(bms-ble-cell i)                ; cell voltage
(bms-ble-temp i)                ; temperature sensor
(bms-ble-set-update-vesc bool)  ; mirror into the VESC BMS values (default t)
(bms-ble-set-send-can bool)     ; forward the values to the VESC over CAN after each poll (default t)
(bms-ble-debug bool)            ; print connection progress to the REPL
```

Keys for `bms-ble-get`: `voltage`, `current` (positive = charging), `soc`
(0..1), `ah-remain`, `ah-nominal`, `cycles`, `cell-count`, `temp-count`,
`soh`, `cell-min`, `cell-max`, `temp-mos` (nil when the BMS does not report it),
`chg-fet`, `dis-fet`, `balance`, `problem`, `msg-count`, `err-count`, `age`
(seconds since the last update, -1 when nothing was received), `runtime`,
`variant` (detected protocol variant).

`bms_ble_example.lisp` in the repository root shows a complete script that
scans, saves the chosen BMS in the EEPROM emulation and reconnects on boot.

## Build configuration

The `sdkconfig.defaults.*` files enable the GATT client, BLE scanning and raise
`CONFIG_BT_CTRL_BLE_MAX_ACT` to 4 (advertising + scan + two connections). When
you build with an old `sdkconfig`, run `idf.py fullclean` first.
