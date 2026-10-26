# Athena - IVSS ESP32 firmware

Athena is the saved IVSS Controller firmware for ESP32-WROOM and the Waveshare
TTL TO RS485 (C) converter. The default PlatformIO environment is `athena` and
the firmware identifies itself as `Athena`. See [RELEASE_ATHENA.md](RELEASE_ATHENA.md)
for the 26 October 2026 snapshot and verification details.

## Hardware

| ESP32 connection | Function |
| --- | --- |
| GPIO17 TX | Connect to Waveshare RXD |
| GPIO16 RX | Connect to Waveshare TXD |
| GPIO25 | External status LED Vin, active HIGH (3.3 V) |
| GPIO0 | BOOT button; hold for setup access |
| GPIO1 / GPIO3 | USB/serial console at 115200 baud |

The converter handles RS485 direction automatically; no DE/RE GPIO is used.
The LED ground and converter TTL-side ground connect to ESP32 GND. The setup
portal is at `http://192.168.4.1` while connected to the device's setup Wi-Fi.

## Build and upload

Install PlatformIO Core, clone the repository, then run from this directory:

```sh
pio run -e athena
```

The board platform and libraries are pinned in `platformio.ini`. The production
build uses the verified files in `backups/Achilles` plus the scripts that apply
Athena's wiring, status LED, live setup page and receive diagnostics. Keep that
archive, its `MANIFEST.json`, all overlay scripts, and the embedded font in
`assets/`; they are required build inputs. `src/main.cpp` is also preserved as
the maintained source and can be built with `pio run -e achilles_current`.
`src/main_2.cpp` is a separate diagnostic program excluded from production.

To program a device, select its actual serial port:

```sh
pio run -e athena -t upload --upload-port COM5
```

The upload hooks **erase the full flash and allocate a new serial**. Saved
Wi-Fi/MQTT settings and uploaded sensor profiles must then be configured again.
Ordinary builds reuse the existing build identity. The legacy `achilles` and
`esp32dev` environment names remain available. The `achilles_invert*` targets
are for older GPIO1/GPIO3 wiring and are not the GPIO17/GPIO16 production build.

## Sensor profiles and MQTT

Schema-3 profiles, their schema, and examples are in `sensor_profiles/`, including
`witmotion_wtvb01.json`. Upload the correct profile through the setup portal;
opening or editing a JSON file in an editor does not apply it to a device.
The WTVB01 example uses 9600 baud, 8N1, slave address 80 (`0x50`), function 3,
and a bulk read of registers `0x0034` through `0x0046`.

Configure Wi-Fi and broker credentials on the device. Firmware publishes to
`sensors/<device-id>/measurements` and subscribes to `ivss/server/heartbeat`.
The IVSS Node.js dashboard must publish live heartbeats on that broker; the
firmware timeout is 45 seconds. Broker connectivity alone does not mean the
dashboard or physical sensor is healthy. A missing server heartbeat or sensor
fault slows normal polling to at least 2 seconds; Modbus recovery can add pauses.

`/status` reports sensor success/failure counters, the last Modbus error, bounded
raw RX traces, current network health, and the independently timed GPIO25 LED.

## Status LED

Each pulse below is 500 ms ON. The pause follows the last pulse in the group.
Network faults take priority in the order shown.

| State | Pattern |
| --- | --- |
| Wi-Fi disconnected | ON 500 ms, OFF 2000 ms, ON 500 ms, OFF 1000 ms |
| MQTT disconnected | Two pulses separated by 500 ms, then OFF 2500 ms |
| Dashboard heartbeat missing | Three pulses separated by 500 ms, then OFF 2500 ms |
| Sensor communication fault | Four pulses separated by 500 ms, then OFF 2500 ms |
| All links and latest sensor acquisition healthy | ON 500 ms, OFF 500 ms continuously |

A failed acquisition switches the LED out of healthy mode even if cached sensor
values exist. A subsequent successful acquisition restores normal blinking.
The timer runs independently of the main loop. Physical LED and sensor behavior
still require verification on the installed device.

## Checks and saved artifacts

```sh
node test/generic_modbus_decoder_test.js
node test/mqtt_long_outage_recovery_test.js
node test/reliability_contract_test.js
node test/live_status_page_test.js
g++ -std=c++11 -Wall -Wextra -I include test/status_led_pattern_test.cpp -o led-pattern-test
g++ -std=c++11 -Wall -Wextra -I test/status_led_stubs -I include test/status_led_service_test.cpp -o led-service-test
```

Run the two compiled LED test executables as well. Saved application, bootloader,
partition and build metadata are in `artifacts/Athena-20261026/`. Verify the saved
source and artifacts with `python scripts/verify_athena.py`. These are build
outputs, not a flash/NVS dump: device-stored credentials, settings and uploaded
profiles are not backed up by saving the source tree.

The archived Achilles documentation describes the original base. This README
describes Athena; older pin assignments in archived documents are historical.
