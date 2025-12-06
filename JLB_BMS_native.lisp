; JBD BMS Native BLE Client Example - CORRECTED MAC ADDRESS
; For VESC Express with jbd_bms_ble extension
;
; IMPORTANT: Use the correct MAC address from your working ESP32 bridge!

(print "====================================")
(print "JBD BMS Native BLE Client")
(print "====================================")

; CORRECT MAC address from your ESP32 bridge code
;(def bms-mac "a5:c2:37:17:c7:1a") ;Markus
(def bms-mac "a5:c2:37:1a:ad:2a") ;David


(print (str-merge "Target BMS: " bms-mac))
(print "====================================")

; Reconnection settings
(def reconnect-interval 10.0)  ; seconds between reconnection attempts
(def poll-interval 1.0)        ; seconds between data requests when connected

; Main loop
(print "Starting main loop...")

(loopwhile t
    (if (jbd-bms-connected)
        (progn
            ; Request data from BMS
            (jbd-bms-request-basic)
            (sleep 0.3)
            (jbd-bms-request-cells)
            (sleep 0.3)

            ; Update VESC BMS structure (enables VESC Tool display)
            (jbd-bms-update-vesc-bms)

            ; Print current values
            (print (str-merge
                "V=" (str-from-n (jbd-bms-get-voltage) "%.2f")
                " I=" (str-from-n (jbd-bms-get-current) "%.2f")
                " SOC=" (str-from-n (jbd-bms-get-soc) "%d") "%"
                " Cells=" (str-from-n (jbd-bms-get-cell-count) "%d")))

            ; Print cell voltage range
            (if (> (jbd-bms-get-cell-count) 0)
                (print (str-merge
                    "Cells: " (str-from-n (jbd-bms-get-cell-min) "%.3f")
                    "V - " (str-from-n (jbd-bms-get-cell-max) "%.3f")
                    "V (delta=" (str-from-n (* 1000 (- (jbd-bms-get-cell-max) (jbd-bms-get-cell-min))) "%.0f") "mV)")))

            (sleep poll-interval)
        )
        ; Not connected - try to connect
        (progn
            (print "Not connected, attempting reconnection...")
            (print (str-merge "Connecting to BMS: " bms-mac))
            (if (jbd-bms-connect bms-mac)
                (print "*** Connected! ***")
                (print "Connection failed!")
            )
            (sleep reconnect-interval)
        )
    )
)
