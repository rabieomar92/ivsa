# VibScope Firmware Audit — AP Instability, Sensor Stalls, Total WiFi Death
**Date:** 2026-08-02 · **Firmware:** 1.8.5-live-readings · **File:** `src/main.cpp` (5,439 lines)

## Symptom map

| Your symptom | Root cause (finding #) |
|---|---|
| AP sometimes visible, sometimes not | F1, F2, F3 |
| Sensor reading "stuck", acquisition connects/disconnects | F4, F5, F6 |
| After a while: AP gone, WiFi completely dead, needs **minutes powered off** | F7 (hardware protection, firmware-driven), F8 |

---

## CRITICAL

### F1. Setup AP allows only ONE client — `max_connection = 1`
**Line 2402:**
```cpp
setupApStarted = WiFi.softAP(setupApSsid.c_str(), setupApPass.c_str(), 1, 0, 1);
```
The last argument caps the AP at **one associated station**. If a phone/laptop associated earlier and didn't cleanly deauth (very common — phones hold the association for minutes), every new connection attempt is **rejected until the stale entry times out**. This alone produces "AP sometimes available, sometimes not."
**Fix:** use `WiFi.softAP(ssid, pass, 1, 0, 4)`.

### F2. STA retry loop constantly drags the radio away from the AP — zero rest between attempts
Lines 125, 133, 2532–2540: connect timeout is **15 s**, retry interval is **10 s**. Because `lastWifiAttemptMs` is set at the *start* of each attempt, when a 15 s attempt times out the 10 s retry window has *already elapsed* — so a new `WiFi.begin()` fires **immediately**. If the saved router is off/out of range/wrong password, the ESP32 is in a *perpetual* STA connect cycle.

The ESP32 has **one radio**. Every `WiFi.begin()` scans all channels and every (re)connect drags the softAP off its channel; associated AP clients get kicked or lose packets each cycle. With a perpetual retry loop, the AP is disrupted essentially continuously. This is the classic ESP32 AP+STA instability pattern and matches your symptom exactly.
**Fix:** exponential backoff (30 s → 60 s → 5 min cap) after each failed attempt; reset backoff on success. Optionally skip STA retries entirely while an AP client is associated (`WiFi.softAPgetStationNum() > 0`).

### F3. On-demand WiFi scans take the AP off-air for seconds
Line 3394: `WiFi.scanNetworks(true, true, false, 200)` — ~13 channels × up to 200 ms = **~2.6 s of AP outage per scan**, on top of F2. Anyone pressing "Scan Nearby WiFi" on the setup page kills the very AP they're connected through.
**Fix:** reduce per-channel time to 120 ms, and warn in the UI; or scan only channels 1/6/11.

### F4. The entire firmware is one blocking loop — Modbus I/O starves the web server
`readModbusRegistersFunction()` (3626–3783) busy-waits with `delay(1)` for the full response (~60–90 ms per fast-frame read at 9600 baud; up to ~210 ms + 30 ms drain on each failure; hard ceiling 15 s). `publishData()` runs this at 10 Hz from `loop()`. Result: **60–80 % of loop time is spent blocked in serial waits**, during which `server.handleClient()` never runs. When the sensor misbehaves, each cycle blocks ~240 ms+ — the setup page and `/api/live-reading` (polled every 1 s by the live page) time out, which reads as "AP stuck / data acquisition disconnected."
**Fix (minimal):** call `server.handleClient()` (and `mqttClient.loop()`) inside the response-wait loop. **Fix (proper):** convert the Modbus transaction to a non-blocking state machine.

### F5. `calibrateSensorBaseline()` freezes everything for 15–30+ seconds, repeatedly
Lines 4196–4286: two passes × 200 samples, each sample = a full Modbus transaction + `delay(5)`, up to 1,000 attempts per pass. It feeds the watchdog, so the WDT never fires — the device just appears **dead on WiFi/HTTP for the whole duration**. Worse: while `baselineCalibrationPending` is set (flaky sensor), `loop()` re-attempts it **every 5 s** (5356–5364) — so a marginal RS485 link turns into a device that freezes for tens of seconds, over and over. Same class of problem: `probeSensorConfig()` (4447–4558) can block **~30 s at boot** (16 bauds × 4 IDs × 2 registers, each a full timeout) when the sensor doesn't answer.
**Fix:** make calibration incremental — collect a handful of samples per `loop()` pass and keep state in globals. Bound the boot probe to the 2–3 most likely bauds.

### F6. Blocking MQTT connect + 3.5 KB publishes at 10 Hz on a weak link
`mqttReconnect()` (3434) does a blocking DNS lookup + TCP connect every 5 s when the broker is unreachable (1–2 s stall each, socket timeout 1 s). Each publish is ~3.5 KB; on marginal RSSI the TCP write itself can stall the loop. Cadence backoff (5294–5313) only kicks in at >20 % failure or ≤ −80 dBm.
**Fix:** skip reconnect attempts when RSSI ≤ −85; resolve broker DNS once and cache the IP; consider halving the payload (there is heavy duplication — `quality` repeats 8 top-level fields verbatim).

---

## THE "MINUTES POWERED OFF" MYSTERY

### F7. Your recovery pattern is a hardware protection device — and the firmware is what trips it
RAM/state clears in **one second** of power-off. Needing *minutes* off means something physical must cool down or reset:

- **PTC resettable fuse (polyfuse)** on the dev board's USB/5 V rail — trips on sustained overcurrent, needs minutes to cool and re-close. This matches your description ("device protection thing which restores after a few minutes powered off") almost word for word.
- **LDO thermal shutdown** (AMS1117-class regulators overheat easily) — same recovery signature.

What the firmware contributes: F2's **perpetual STA connect loop keeps the radio at near-100 % TX/scan duty** (WiFi TX bursts are 300–500 mA), stacked on always-on AP mode (no modem sleep possible), 10 Hz Modbus TX, and 10 Hz 3.5 KB MQTT publishes. That is close to worst-case sustained current draw for an ESP32 — enough to warm a marginal PTC/LDO until it lets go. The AP dying *gradually/intermittently first, then completely* is consistent with a sagging 3V3 rail: the radio PA browns out first while the CPU keeps running.

**Actions:**
1. Fix F2 (retry backoff) — this alone drops average radio duty enormously.
2. Cap TX power: `esp_wifi_set_max_tx_power(44); // ~11 dBm` after WiFi init. The AP client is 2 m away; you do not need +20 dBm.
3. Hardware: power from a solid 5 V ≥ 1 A supply, add 470–1000 µF bulk capacitance at the ESP32 5 V input, and check whether your board/enclosure wiring has a PTC in series. Measure 3V3 during a WiFi TX burst.

### F8. You are flying completely blind — Serial logging is compiled out
Lines 5154–5156: with `SENSOR_ON_PROGRAM_UART_PINS`, `Serial.begin()` is **never called** — all several hundred log statements, including the **reset-reason print at boot (line 5164)**, go nowhere. You cannot currently distinguish brownout (reason 12) from WDT panic from a clean boot. Note the firmware *also* has two silent self-restart paths — heap-integrity failure and sustained low heap both call `esp_restart()` (5376–5394) — which you'd never see.
**Fix (quick win):** add `"resetReason": esp_reset_reason()` plus `minFreeHeap`/`minLargestFreeBlock` to the `/status` JSON, and keep a small in-RAM ring buffer of log lines exposed at `/logs`. After the next lockup-recovery, `/status` will tell you immediately whether it was brownout (→ F7 confirmed) or a hang.

---

## MODERATE / DESIGN

**F9. Sensor UART on GPIO1/GPIO3 fights the USB bridge and boot ROM.** On a WROOM-32 devkit these pins are wired to the CP2102/CH340. The bridge's TX pin actively drives GPIO3 against your RS485 converter (continuous contention current, noise — this is likely feeding your `modbusEchoCount`/`modbusNoiseByteCount`). Also, every reset the boot ROM spews 115200-baud log data out GPIO1 **onto the RS485 bus**. Prefer GPIO16/17 (the code's own defaults when the flag is absent) unless the board physically requires TX/RX pins.

**F10. Hostname failure permanently disables STA until reboot.** `networkIdentityHealthy` is set once in `startSetupPortal()` (2374); if `WiFi.setHostname()` fails, `beginStaConnection()` refuses to ever connect (2483–2487) with no retry path.

**F11. 7 s Modbus recovery pause + fast-frame fallback churn causes visible data gaps.** After 10 consecutive failures: UART restart + 7 s of no polling (lines 148–149, 3580–3585). Combined with the 30 s fast-frame retry cycle (3 failures → split reads → retry bulk every 30 s), a marginal bus produces the rhythmic connect/disconnect you're seeing in readings.

**F12. Heap fragmentation risk in web handlers.** `handleConfig()` builds a ~15–20 KB page via incremental `String` concatenation (hundreds of reallocs per request); `html.reserve(20000)`'s return value is unchecked there (it *is* checked in `handleLiveReadings`). WiFi driver allocation failures from fragmentation kill connectivity **before** the low-heap restart threshold (24 KB / 8 KB largest block) is reached. Low priority once F1–F7 are fixed, but track `minLargestFreeBlock` via F8.

**F13. Minor:** `computeVibrationRMS()` (4288–4445) is dead code — never called; remove to avoid confusion. `PUBLISH_FAST_MS` == `PUBLISH_MED_MS` (both 100 ms), so the "strong WiFi" tier is meaningless. The LED-active-LOW comment (line 47) is wrong for most WROOM devkits (GPIO2 LED is active-HIGH), so LED states may be inverted.

---

## Recommended fix order

1. **F1** — one-line fix, immediate AP improvement.
2. **F2 + TX power cap** — stops both the AP flapping and the overcurrent that trips the protection.
3. **F8** — visibility: reset reason + heap on `/status`, so the next incident is diagnosable.
4. **F5** — incremental calibration/probe (removes the 15–30 s freezes).
5. **F4** — service HTTP inside Modbus waits (or non-blocking state machine).
6. Hardware pass — supply, bulk caps, PTC check, GPIO16/17 for the sensor UART.
