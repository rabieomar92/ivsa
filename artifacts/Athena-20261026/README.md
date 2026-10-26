# Athena saved production build

Built on 26 October 2026 with `pio run -e athena` for ESP32, 4 MB flash, 80 MHz CPU.
See `build-info.json` for dependency versions, validation, and image offsets.
`build_identity.h` identifies this saved binary; `serial_registry.json` preserves
its provisioning history. Do not install this same identity on multiple devices.
Build and upload from source to provision a fresh serial for each device.

This directory contains an application image, not a device flash/NVS dump.
It does not contain saved Wi-Fi/MQTT credentials or an uploaded sensor profile.
The upload hook erases the full flash and provisions a fresh serial.

If restoring these exact images using esptool 4.x on a device reserved for this
saved identity, copy all four images together, select the correct port and run:

```sh
python -m esptool --chip esp32 --port COM5 write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

That manual command writes only those regions; it does not back up or initialize
device NVS. This release save did not execute any upload or flash command.
Verify all saved files from the repository root with `python scripts/verify_athena.py`.
