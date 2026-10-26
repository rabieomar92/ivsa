#!/usr/bin/env python3
"""
Build 1.8.8-radio-minimal from the clean 1.8.5 baseline.

Applies ONLY low-risk radio/network fixes. Deliberately does NOT include the
1.8.7 changes that touched the Modbus acquisition path (servicing HTTP inside
the Modbus response wait, the response-timeout credit-back, the incremental
baseline calibration, profile-apply staging, UART RX buffer, probe bounding).
Those changes caused a mutual-starvation feedback loop between the web server
and sensor acquisition.

Every replacement is asserted to occur exactly once, so the script fails loudly
rather than silently producing a half-patched file.
"""
import sys
from pathlib import Path

SRC = Path("backups/current-fixed-ap-before-rollback-20260730/main.cpp")
DST = Path("src/main.cpp")

text = SRC.read_text(encoding="utf-8")
applied = []


def sub_once(label, old, new):
    global text
    count = text.count(old)
    if count != 1:
        sys.exit(f"FAIL [{label}]: expected exactly 1 match, found {count}")
    text = text.replace(old, new, 1)
    applied.append(label)


# --- 0. version marker -------------------------------------------------------
sub_once(
    "version string",
    'const char         *FIRMWARE_VERSION = "1.8.5-live-readings";',
    'const char         *FIRMWARE_VERSION = "1.8.8-radio-minimal";',
)

# --- 1. esp_wifi.h for the TX power API -------------------------------------
sub_once(
    "esp_wifi.h include",
    '#include "esp_heap_caps.h"',
    '#include "esp_heap_caps.h"\n#include "esp_wifi.h"',
)

# --- 2. STA retry backoff constant + state ----------------------------------
sub_once(
    "backoff constant",
    "const unsigned long WIFI_RETRY_INTERVAL_MS      = 10000;",
    "const unsigned long WIFI_RETRY_INTERVAL_MS      = 10000; // base STA retry wait, doubles on failure\n"
    "const unsigned long WIFI_RETRY_BACKOFF_MAX_MS   = 300000; // STA retry backoff cap (5 min)",
)

sub_once(
    "backoff state var",
    "bool staConnectInProgress = false;",
    "bool staConnectInProgress = false;\n"
    "// Grows after each failed STA connect attempt, resets on success or when new\n"
    "// credentials are saved. Without it the radio never rests: the 15 s attempt\n"
    "// timeout was longer than the 10 s retry interval, and the interval was\n"
    "// measured from the attempt START, so a new WiFi.begin() fired immediately\n"
    "// after every timeout. One shared radio scanning continuously drags the\n"
    "// softAP off-channel and holds current draw at worst case.\n"
    "unsigned long staRetryBackoffMs = WIFI_RETRY_INTERVAL_MS;",
)

# --- 3. TX power limiter -----------------------------------------------------
sub_once(
    "applyRadioTxPowerLimit definition",
    "void startSetupPortal() {",
    "void applyRadioTxPowerLimit() {\n"
    "  // Cap radio TX power at ~15 dBm. Full +20 dBm bursts draw 400-500 mA peaks\n"
    "  // that can sag a marginal 5V/3V3 rail and trip a PTC fuse or regulator\n"
    "  // thermal protection, which presents as the AP and STA dying until the\n"
    "  // hardware has cooled for several minutes.\n"
    "  esp_wifi_set_max_tx_power(60); // units of 0.25 dBm -> 15 dBm\n"
    "}\n\n"
    "void startSetupPortal() {",
)

sub_once(
    "applyRadioTxPowerLimit forward decl",
    "void startSetupPortal();",
    "void applyRadioTxPowerLimit();\nvoid startSetupPortal();",
)

# --- 4. AP max_connection 1 -> 4 --------------------------------------------
sub_once(
    "AP max_connection",
    "    setupApStarted = WiFi.softAP(setupApSsid.c_str(), setupApPass.c_str(), 1, 0, 1);",
    "    // channel 1, visible SSID, up to 4 clients. max_connection=1 made the AP\n"
    "    // reject every new association while a previous client's stale association\n"
    "    // lingered: the main cause of 'AP sometimes available, sometimes not'.\n"
    "    setupApStarted = WiFi.softAP(setupApSsid.c_str(), setupApPass.c_str(), 1, 0, 4);\n"
    "    if (setupApStarted) applyRadioTxPowerLimit();",
)

# --- 5. hostname failure must not permanently disable STA -------------------
sub_once(
    "hostname gate",
    "  if (!networkIdentityHealthy) {\n"
    "    staConnectInProgress = false;\n"
    '    Serial.println("[WiFi] STA held offline because interface identity setup failed.");\n'
    "    return;\n"
    "  }",
    "  // A failed setHostname() is cosmetic: the hostname is optional DHCP/mDNS\n"
    "  // metadata. Previously it latched networkIdentityHealthy false and blocked\n"
    "  // every STA connection until reboot, so one transient netif hiccup at boot\n"
    "  // cost the unit its uplink for the whole run. Still reported on /status.\n"
    "  if (!networkIdentityHealthy) {\n"
    '    Serial.println("[WiFi] Hostname was not applied; connecting STA anyway (hostname is optional metadata).");\n'
    "  }",
)

sub_once(
    "TX power after WiFi.begin",
    "  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());",
    "  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());\n  applyRadioTxPowerLimit();",
)

# --- 6. reset backoff on successful connect ---------------------------------
sub_once(
    "backoff reset on success",
    "    if (status == WL_CONNECTED) {\n      staConnectInProgress = false;",
    "    if (status == WL_CONNECTED) {\n"
    "      staConnectInProgress = false;\n"
    "      staRetryBackoffMs = WIFI_RETRY_INTERVAL_MS; // reset backoff on success",
)

# --- 7. exponential backoff on timeout --------------------------------------
sub_once(
    "backoff on timeout",
    '    Serial.println("[WiFi] STA connect attempt timed out. AP remains available; next retry is scheduled.");',
    "    // Measure the retry wait from the END of the failed attempt, and back off.\n"
    "    lastWifiAttemptMs = now;\n"
    "    staRetryBackoffMs = (staRetryBackoffMs >= WIFI_RETRY_BACKOFF_MAX_MS / 2)\n"
    "      ? WIFI_RETRY_BACKOFF_MAX_MS\n"
    "      : staRetryBackoffMs * 2;\n"
    '    Serial.printf("[WiFi] STA connect attempt timed out. Next retry in %lu s; AP remains available.\\n",\n'
    "                  staRetryBackoffMs / 1000UL);",
)

sub_once(
    "backoff used by retry gate",
    "  if (!staConnectInProgress && now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {",
    "  if (!staConnectInProgress && now - lastWifiAttemptMs >= staRetryBackoffMs) {",
)

# --- 8. reset backoff when the operator saves new credentials ---------------
sub_once(
    "backoff reset on save",
    '  beginStaConnection("settings saved");',
    "  // New credentials were just supplied, so this is not a blind retry: clear\n"
    "  // any accumulated backoff so a corrected password is acted on immediately.\n"
    "  staRetryBackoffMs = WIFI_RETRY_INTERVAL_MS;\n"
    '  beginStaConnection("settings saved");',
)

# --- 9. shorter scan dwell --------------------------------------------------
sub_once(
    "scan dwell",
    "      const int started = WiFi.scanNetworks(true, true, false, 200);",
    "      // 120 ms per channel, not 200 ms. A scan takes the single radio away from\n"
    "      // the softAP for its whole duration, so a 13-channel sweep at 200 ms was\n"
    "      // ~2.6 s of AP outage, triggered by the very client using the setup page.\n"
    "      // 120 ms still reliably catches beacons (typical interval 102 ms).\n"
    "      const int started = WiFi.scanNetworks(true, true, false, 120);",
)

# --- 10. diagnostics on /status (Serial is compiled out on this build) ------
sub_once(
    "status diagnostics",
    '  json += "\\"freeHeap\\":" + String(ESP.getFreeHeap()) + ",";',
    "  // resetReason: 1=POWERON 3=SW 4=PANIC 5=INT_WDT 6=TASK_WDT 7=RTC_WDT 12=BROWNOUT.\n"
    "  // Serial logging is unavailable on this build (GPIO1/3 carry the sensor), so\n"
    "  // this endpoint is the only place brownouts and silent restarts are visible.\n"
    '  json += "\\"resetReason\\":" + String((int)esp_reset_reason()) + ",";\n'
    '  json += "\\"uptimeSeconds\\":" + String(millis() / 1000UL) + ",";\n'
    '  json += "\\"minFreeHeap\\":" + String(minFreeHeap) + ",";\n'
    '  json += "\\"minLargestFreeBlock\\":" + String(minLargestFreeBlock) + ",";\n'
    '  json += "\\"staRetryBackoffMs\\":" + String(staRetryBackoffMs) + ",";\n'
    '  json += "\\"freeHeap\\":" + String(ESP.getFreeHeap()) + ",";',
)

DST.write_text(text, encoding="utf-8")

print(f"Wrote {DST} from {SRC}")
for i, name in enumerate(applied, 1):
    print(f"  {i:2d}. {name}")

# Guard: none of the acquisition-path changes may reappear.
forbidden = {
    "HTTP inside Modbus wait": "respTimeoutCeilingMs",
    "incremental calibration": "serviceBaselineCalibration",
    "profile apply staging": "sensorProfileApplyPending",
}
for label, needle in forbidden.items():
    if needle in text:
        sys.exit(f"FAIL: acquisition-path change leaked back in ({label})")
print("\nGuard OK: acquisition path is untouched 1.8.5 code.")
