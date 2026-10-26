"""Build Athena from the verified Achilles base without changing the archive."""
Import("env")

import hashlib
import json
import runpy
import shutil
from pathlib import Path

project = Path(env["PROJECT_DIR"])
backup = project / "backups" / "Achilles"
stage = project / ".pio" / "original-achilles"
manifest = json.loads((backup / "MANIFEST.json").read_text(encoding="utf-8"))
for entry in manifest["files"]:
    source = backup / entry["path"]
    if hashlib.sha256(source.read_bytes()).hexdigest() != entry["sha256"]:
        raise RuntimeError("Original Achilles backup changed: " + entry["path"])
    target = stage / entry["path"]
    target.parent.mkdir(parents=True, exist_ok=True)
    # Preserve provisioned identities between ordinary builds.
    if entry["path"] in ("include/build_identity.h", "scripts/serial_registry.json") and target.exists():
        continue
    shutil.copy2(source, target)

# Apply hardware wiring and receive diagnostics to the staged firmware.
# The verified archive remains intact.
shutil.copy2(project / "include" / "status_led.h", stage / "include" / "status_led.h")
shutil.copy2(project / "include" / "status_led_pattern.h", stage / "include" / "status_led_pattern.h")
shutil.copy2(project / "include" / "live_status_page.h", stage / "include" / "live_status_page.h")
shutil.copy2(project / "include" / "sensor_rx_trace.h", stage / "include" / "sensor_rx_trace.h")
staged_main = stage / "src" / "main.cpp"
source = staged_main.read_text(encoding="utf-8")
version_definition = 'const char         *FIRMWARE_VERSION           = "Achilles";'
if source.count(version_definition) != 1:
    raise RuntimeError("Original Achilles version definition changed; review Athena overlay")
source = source.replace(version_definition, version_definition.replace('"Achilles"', '"Athena"'))
led_definition = "#define LED_PIN 2   // Active-LOW on most ESP32-WROOM dev boards"
if source.count(led_definition) != 1:
    raise RuntimeError("Original Achilles LED definition changed; review the hardware overlay")
source = source.replace(led_definition, '#include "status_led.h"')
source = "\n".join(
    line.replace("LOW", "STATUS_LED_ON").replace("HIGH", "STATUS_LED_OFF")
        .replace(" // off (active-low)", " // off")
    if "digitalWrite(LED_PIN," in line else line
    for line in source.split("\n")
)
# Ignore stale persisted manual-direction settings for this auto-direction module.
for following in (
    "\n      RS485_DE_RE_PIN = -1;",
    "\n  // This build is explicitly for a two-wire-TTL auto-direction converter.",
):
    source = source.replace(
        "#if defined(SENSOR_ON_PROGRAM_UART_PINS)" + following,
        "#if defined(SENSOR_ON_PROGRAM_UART_PINS) || defined(SENSOR_RS485_AUTO_DIRECTION)" + following,
    )
source = source.replace(
    "// This isolated converter labels its MCU-side UART pins by the MCU signal:\n"
    "// ESP32 TX connects to module TXD (internally the RS485 driver's DI), and\n"
    "// ESP32 RX connects to module RXD (internally the RS485 receiver's RO).",
    "// Waveshare RXD receives ESP32 TX; Waveshare TXD drives ESP32 RX.",
)
source = source.replace("// ESP32-U TX header / GPIO1 -> module TXD", "// GPIO17 -> Waveshare RXD")
source = source.replace("// ESP32-U RX header / GPIO3 <- module RXD", "// GPIO16 <- Waveshare TXD")
trace_overlay = runpy.run_path(str(project / "scripts" / "sensor_rx_trace_overlay.py"))
source = trace_overlay["apply_sensor_rx_trace"](source)
led_overlay = runpy.run_path(str(project / "scripts" / "status_led_overlay.py"))
source = led_overlay["apply_status_led"](source)
live_overlay = runpy.run_path(str(project / "scripts" / "live_status_overlay.py"))
source = live_overlay["apply_live_status"](source)
staged_main.write_text(source, encoding="utf-8")

# Keep the original embedded asset name used by the firmware's linker symbols.
asset = "assets/jetbrains_mono_regular.woff2"
if (project / asset).read_bytes() != (backup / asset).read_bytes():
    raise RuntimeError("Embedded font differs from the original Achilles backup")

env.Replace(PROJECT_SRC_DIR=str(stage / "src"), PROJECT_INCLUDE_DIR=str(stage / "include"))
env.Prepend(CPPPATH=[str(stage / "include")])
env.Replace(PROJECT_DIR=str(stage))
try:
    env.SConscript(str(stage / "scripts" / "generate_serial.py"), exports="env")
finally:
    env.Replace(PROJECT_DIR=str(project))
print("[Athena] Building verified Achilles base with current wiring, LED timer, RX trace and live status")
