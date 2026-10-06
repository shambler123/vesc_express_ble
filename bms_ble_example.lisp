; External BMS over BLE (JBD, Daly, LiPower, LiTech)
;
; The firmware connects to the BMS in the background and keeps the
; connection alive. The VESC Tool BLE link stays usable at the same time.
; All values are mirrored into the VESC BMS structure, so VESC Tool shows
; them on the BMS page and (get-bms-val 'bms-v-tot) etc. work as usual.
;
; Extensions:
;   (bms-ble-scan [seconds])     start a scan (non blocking)
;   (bms-ble-scanning)           t while the scan runs
;   (bms-ble-scan-results)       ((name mac rssi type) ...)
;   (bms-ble-connect mac [type]) type: 'auto 'jbd 'daly 'lipower 'litech, default 'auto
;   (bms-ble-disconnect)
;   (bms-ble-connected)
;   (bms-ble-state)              'disabled 'idle 'connecting 'connected
;   (bms-ble-type)               detected / configured BMS type
;   (bms-ble-target)             (mac type) or nil
;   (bms-ble-get key)            'voltage 'current 'soc 'ah-remain 'ah-nominal 'cycles
;                                'cell-count 'temp-count 'cell-min 'cell-max 'temp-mos
;                                'chg-fet 'dis-fet 'balance 'problem 'msg-count 'err-count
;                                'age 'runtime 'variant
;   (bms-ble-cell i) (bms-ble-temp i)
;   (bms-ble-set-update-vesc bool)  mirror into VESC BMS values (default t)
;   (bms-ble-debug bool)            print connection debug output

; The selected BMS is stored in the EEPROM emulation:
;   addr 0: upper 3 MAC bytes, addr 1: lower 3 MAC bytes, addr 2: type (0 auto 1 jbd 2 daly 3 lipower 4 litech)
(def eeprom-mac-hi 0)
(def eeprom-mac-lo 1)
(def eeprom-type 2)

(defun type-to-int (ty)
    (cond ((eq ty 'jbd) 1) ((eq ty 'daly) 2) ((eq ty 'lipower) 3) ((eq ty 'litech) 4) (t 0)))
(defun int-to-type (i)
    (cond ((= i 1) 'jbd) ((= i 2) 'daly) ((= i 3) 'lipower) ((= i 4) 'litech) (t 'auto)))

; "A5:C2:37:17:C7:1A" -> (0xA5C237 0x17C71A)
(defun mac-to-ints (mac) {
        (var b (map (fn (x) (str-to-i x 16)) (str-split mac ":")))
        (list
            (bits-enc-int (bits-enc-int (ix b 2) 8 (ix b 1) 8) 16 (ix b 0) 8)
            (bits-enc-int (bits-enc-int (ix b 5) 8 (ix b 4) 8) 16 (ix b 3) 8)
        )
})

(defun ints-to-mac (hi lo)
    (str-merge
        (str-from-n (bits-dec-int hi 16 8) "%02X") ":"
        (str-from-n (bits-dec-int hi 8 8) "%02X") ":"
        (str-from-n (bits-dec-int hi 0 8) "%02X") ":"
        (str-from-n (bits-dec-int lo 16 8) "%02X") ":"
        (str-from-n (bits-dec-int lo 8 8) "%02X") ":"
        (str-from-n (bits-dec-int lo 0 8) "%02X")))

; Save and use a BMS, e.g. (bms-save "A5:C2:37:17:C7:1A" 'jbd)
(defun bms-save (mac ty) {
        (var ints (mac-to-ints mac))
        (eeprom-store-i eeprom-mac-hi (ix ints 0))
        (eeprom-store-i eeprom-mac-lo (ix ints 1))
        (eeprom-store-i eeprom-type (type-to-int ty))
        (bms-ble-connect mac ty)
})

(defun bms-forget () {
        (eeprom-erase eeprom-mac-hi)
        (eeprom-erase eeprom-mac-lo)
        (eeprom-erase eeprom-type)
        (bms-ble-disconnect)
})

; Scan for BMS devices and print what was found, e.g. (bms-find 5)
(defun bms-find (secs) {
        (bms-ble-scan secs)
        (loopwhile (bms-ble-scanning) (sleep 0.2))
        (var res (bms-ble-scan-results))
        (print (str-merge "Found " (str-from-n (length res)) " device(s):"))
        (loopforeach d res
            (print (str-merge (ix d 1) "  " (to-str (ix d 3)) "  rssi " (str-from-n (ix d 2)) "  " (ix d 0))))
        res
})

; Connect to the saved BMS on start
(let ((hi (eeprom-read-i eeprom-mac-hi))
      (lo (eeprom-read-i eeprom-mac-lo))
      (ty (eeprom-read-i eeprom-type)))
    (if (and hi lo) {
            (var mac (ints-to-mac hi lo))
            (print (str-merge "Connecting to saved BMS " mac))
            (bms-ble-connect mac (int-to-type (if ty ty 0)))
        }
        (print "No BMS saved, use (bms-find 5) and (bms-save mac type)")))

; Status printout
(loopwhile t {
        (if (bms-ble-connected)
            (print (str-merge
                (to-str (bms-ble-type))
                " V=" (str-from-n (bms-ble-get 'voltage) "%.2f")
                " I=" (str-from-n (bms-ble-get 'current) "%.2f")
                " SOC=" (str-from-n (* 100 (bms-ble-get 'soc)) "%.0f") "%"
                " cells=" (str-from-n (bms-ble-get 'cell-count))
                " " (str-from-n (bms-ble-get 'cell-min) "%.3f")
                "-" (str-from-n (bms-ble-get 'cell-max) "%.3f") "V"))
            (print (str-merge "BMS state: " (to-str (bms-ble-state)))))
        (sleep 5)
})
