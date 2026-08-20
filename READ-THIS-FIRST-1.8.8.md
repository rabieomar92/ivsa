# Flash this: 1.8.8-radio-minimal — and what went wrong with 1.8.7

## What I broke in 1.8.7

I added code that ran the web server **inside** the Modbus sensor-read wait loop,
and let time spent serving a page **extend** the sensor's response timeout.

That created a feedback loop:

```
loop() -> publishData() -> readModbus() -> server.handleClient()
                                                |
   browser (live page polls every 1 s) <--------+
                                                |
   every served request ADDED time to the Modbus timeout -> the read never ended
```

One publish cycle does 5 Modbus transactions. Each could stretch to ~2.2 s
instead of ~0.2 s, so a single cycle that should take 100 ms could take ~11 s —
and it got worse the moment you opened a page. That is exactly your report:
pages load forever, then the connection drops, and no readings arrive.

I shipped it on the strength of a clean compile. A clean compile says nothing
about runtime behaviour, and I presented that as more reassurance than it was
worth. That was my error, not yours.

## What 1.8.8 is

Rebuilt from the clean **1.8.5** baseline — the version that was producing
readings — with **only** the radio/network fixes. The sensor acquisition path is
untouched.

Verified byte-identical to 1.8.5 (MD5 of each extracted function):

| Function | Status |
|---|---|
| `readModbusRegistersFunction` | identical |
| `publishData` | identical |
| `readWitmotionFastFrame` | identical |
| `readGenericVector` | identical |
| `readWitmotionEngineeringMetrics` | identical |
| `calibrateSensorBaseline` | identical |
| `probeSensorConfig` | identical |
| `handleLiveReadingApi` | identical |
| `loop`, `setup`, `checkSetupTriggers` | identical |

Everything from 1.8.7 that touched acquisition is **gone**: HTTP-inside-Modbus,
the timeout credit-back, incremental calibration, profile-apply staging, the UART
buffer change, the probe bounding. A guard in the patch script fails the build if
any of it reappears.

### The 6 changes that remain (all outside acquisition)

1. **AP `max_connection` 1 → 4.** A phone holding a stale association blocked
   every other client. Prime cause of intermittent AP discovery.
2. **STA retry backoff 10 s → 5 min.** The 15 s connect timeout was longer than
   the 10 s retry interval, and the interval was measured from the attempt
   *start* — so a new `WiFi.begin()` fired instantly after every failure. The one
   shared radio was scanning permanently, dragging the softAP off-channel.
3. **TX power capped to 15 dBm.** Reduces 400–500 mA peaks.
4. **Scan dwell 200 → 120 ms/channel.** Cuts AP outage per scan from ~2.6 s to ~1.6 s.
5. **Hostname failure no longer disables STA permanently.**
6. **`/status` diagnostics:** `resetReason`, `uptimeSeconds`, `minFreeHeap`,
   `minLargestFreeBlock`, `staRetryBackoffMs`.

Builds clean, 0 warnings (`-Wall -Wextra -Wformat=2`). Flash 77.8%, RAM 20.7%.

## Flash it

```
cd C:\Users\Legion 5\Documents\PlatformIO\Projects\pio_testing
pio run -e esp32dev -t upload
```

## Then do this one thing

Once it has been running and misbehaves again, open:

```
http://192.168.4.1/status
```

and look at **`resetReason`**:

| Value | Meaning | What it tells us |
|---|---|---|
| **12** | **BROWNOUT** | Power problem — firmware cannot fix it; go to the hardware list |
| 6 / 5 | Watchdog | Something still blocks `loop()` > 8 s |
| 4 | Panic/crash | Firmware fault — send me the value |
| 3 | Software restart | A heap self-restart path fired |
| 1 | Power-on | Clean boot |

This single number decides whether the remaining fault is software or power, and
it ends the guessing. Serial logging is compiled out on this build (GPIO1/3 carry
the sensor), so `/status` is the only window you have.

## What this does NOT fix — please read

- **If no sensor readings appear on 1.8.8**, the cause is *not* in the changes I
  made: that code path is byte-identical to 1.8.5. Look at the RS485 wiring, the
  sensor's own baud/slave-ID, and the `lastError` / `failureCount` /
  `crcFailureCount` / `echoCount` fields already reported by `/status`.
- **GPIO1/GPIO3 for the sensor UART is a real hazard.** The USB-serial bridge
  drives GPIO3 against your RS485 converter, and the boot ROM dumps 115200-baud
  data onto the bus at every reset. This is the likely source of your echo and
  noise-byte counts. Moving to GPIO16/17 is a wiring change plus deleting the
  three `SENSOR_*_PIN` lines from `platformio.ini`.
- **Power.** 5 V ≥ 1 A supply, 470–1000 µF bulk capacitance at the ESP32 5 V
  input, and check for a PTC in series. If `resetReason` is 12, this is the whole
  problem.

## Rollback points

| Folder | Contents |
|---|---|
| `backups/current-fixed-ap-before-rollback-20260730/` | clean 1.8.5 (source of this build) |
| `backups/pre-stability-fixes-1.8.5-20260802/` | identical 1.8.5 copy |
| `backups/pre-stability-fixes-20260802/main-1.8.7-broken.cpp` | the 1.8.7 that broke it |

To rebuild 1.8.8 from scratch at any time:
`python3 scripts/apply_radio_minimal_patch.py`
