# VibScope firmware 1.3.1 production release

This release targets ESP32-WROOM-32U with the sensor UART mapped to GPIO1
(TX) and GPIO3 (RX).

## Audited artifacts

- Firmware version: `1.3.1-32U-always-on-setup`
- Firmware binary SHA-256:
  `D9903A06FC54C3351CF4176A9656117149914986707D22B00AC8716662A3E702`
- Source SHA-256:
  `F68F17D867806C45846A31C0F72078C406728B93BC0A5FCF87DBD69A50E35EA3`
- PlatformIO configuration SHA-256:
  `885DF8C6176C324BED1FBC874850BD33D6DCA65E4AFD92F8E2B87183F2A5EA69`
- Platform: `espressif32@7.0.1`
- PubSubClient: `2.8.0`

## Required manufacturing checks

1. Connect the external antenna to the ESP32-WROOM-32U U.FL connector.
2. Flash and verify the firmware binary, then remove the USB data connection
   during normal sensor operation because GPIO1/GPIO3 are shared with UART0.
3. Connect to the always-on VibScope setup AP and open `http://192.168.4.1`.
   Replace the migration password `12345678` with a unique 8-63 character
   password. The portal rejects production saves while the shared fallback
   remains active.
4. Confirm `/status` reports:
   `configStorageHealthy=true`, `watchdogActive=true`,
   `mqttBufferReady=true`, and `telemetryMemoryHealthy=true`.
5. Confirm live sensor data, frequency values above 100 Hz where applicable,
   MQTT reconnect after broker/network loss, and sensor recovery after
   disconnect/reconnect.
6. Record the flashed binary hash and device ID in the customer build record.

## Hardware and infrastructure requirements

- A firmware-controlled sensor power cycle requires a load switch or MOSFET.
  Wire its enable input to a safe GPIO and define
  `SENSOR_POWER_ENABLE_PIN=<gpio>`. The default `-1` safely disables this feature.
- GPIO1/GPIO3 cannot be electrically isolated from an onboard USB-UART bridge
  by firmware. A production PCB should use a separate UART or add hardware
  isolation.
- MQTT transport encryption requires the customer's broker hostname, TLS port,
  and CA certificate. Do not send reusable production credentials over a
  public or untrusted network using plaintext port 1883.
- Secure boot, flash encryption, and signed OTA rollback require production key
  provisioning and an update service. These irreversible eFuse operations must
  be performed by the manufacturing security process, not automatically by
  application firmware.

## Validation commands

```powershell
node test\reliability_contract_test.js
$pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
& $pio run
& $pio check --environment esp32dev --skip-packages
```
