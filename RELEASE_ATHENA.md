# Athena - 26 October 2026

This snapshot preserves the current firmware, including the sensor-fault LED fix.

- GPIO17 TX -> Waveshare RXD; GPIO16 RX <- Waveshare TXD; automatic direction.
- GPIO25 active-HIGH status LED with an independent 20 ms timer and separate
  patterns for Wi-Fi, MQTT, dashboard-heartbeat and physical-sensor faults.
- Latest completed acquisition determines sensor LED health and `/status`'s
  `sensorBus.lastReadOk`; cached samples do not count as successful reads.
- Live setup status, bounded raw receive traces and the schema-3 sensor profiles.
- Existing MQTT recovery, polling limits, configuration persistence, serial
  provisioning and full-erase upload behavior retained.
- `athena` is the default build; older environment names remain compatible.

The production source is regenerated from the hash-verified `backups/Achilles`
archive and the included overlays. Both that complete dependency and the
maintained `src/main.cpp` source are included. Font assets include their licenses.
Serial registries and build identities are included to preserve provisioning
history; normal uploads still allocate a fresh identity.

Validation results and build input/output SHA-256 checksums accompany the saved
artifacts. This save operation does not flash hardware. A connected sensor's
outstanding communication fault cannot be diagnosed from source alone: its live
Modbus error and RX trace are still required. Disconnecting the sensor is expected
to report a fault. Physical GPIO25 patterns have not been verified on a device.
