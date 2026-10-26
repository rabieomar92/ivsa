// =============================================================================
// main_2.cpp  --  MINIMAL AP TEST FIRMWARE
//
// Purpose: prove the hardware can hold a WiFi access point, using the smallest
// possible amount of code. No sensor, no Modbus, no MQTT, no STA, no NVS.
//
// IMPORTANT: the on-board LED reports everything. You do NOT need the web page,
// a serial cable, or a working AP to read the result. Just watch the LED.
//
// HOW TO BUILD THIS INSTEAD OF main.cpp
//   VSCode: pick the environment "ap-test" in the PlatformIO toolbar, Upload
//   Terminal:  pio run -e ap-test -t upload
//   Normal firmware is still:  pio run -e esp32dev -t upload
//
// -----------------------------------------------------------------------------
// READING THE LED  (on-board blue LED, GPIO2)
//
//   AT POWER-UP, it flashes quickly a number of times, then pauses:
//        1 flash   = this is the 1st start since power was applied  (normal)
//        2 flashes = the board has RESET once since power was applied
//        3 flashes = reset twice ... and so on
//     If you power up and keep seeing the flash-burst repeat over and over,
//     THE BOARD IS RESETTING IN A LOOP. That is a power or brownout fault.
//
//   THEN IT SETTLES INTO ONE OF THESE:
//        slow steady blink, 1 per second   = AP is UP, nobody connected
//        two quick blinks then a pause     = AP is UP, a client is connected
//        very fast continuous blinking     = AP FAILED TO START
//        LED frozen on or off              = the processor has hung
//
//   THE KEY OBSERVATION:
//     If the AP disappears from your phone BUT the LED keeps blinking its
//     normal 1-per-second pattern, the processor never reset -- only the radio
//     died. That is NOT a brownout.
//     If the AP disappears AND the LED restarts its flash-burst, the board
//     reset, and the burst count tells you how many times.
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"
#include "esp_system.h"

// --- things you may want to change -------------------------------------------
static const char *AP_SSID     = "VibScope-TEST";
static const char *AP_PASSWORD = "vibscope123";   // must be 8-63 characters

// On-board LED. Most ESP32-WROOM DevKits drive it HIGH to light up.
// If your LED behaves backwards (dark when it should be lit), set this to 1.
#define LED_PIN         2
#define LED_ACTIVE_LOW  0

// Radio transmit power, units of 0.25 dBm. Range 8..84 (2 dBm .. 20 dBm).
// Lower asks less current from the supply at the moment the radio starts.
//   84 = 20 dBm (max)   60 = 15 dBm   44 = 11 dBm   34 = 8.5 dBm
#define AP_TX_POWER 44
// -----------------------------------------------------------------------------

WebServer server(80);
static const IPAddress apIp(192, 168, 4, 1);
static const IPAddress apMask(255, 255, 255, 0);

static bool     apStarted  = false;
static uint8_t  apAttempts = 0;

// RTC_NOINIT survives a reset but NOT a true power-off, which is exactly what
// we need to count resets since power was applied.
#define BOOT_MAGIC 0x42303254UL   // "B02T"
RTC_NOINIT_ATTR uint32_t bootMagic;
RTC_NOINIT_ATTR uint32_t bootsSincePower;

static inline void ledOn()  { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? LOW  : HIGH); }
static inline void ledOff() { digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW ); }

// Blocking flash burst, used once at startup before the main loop begins.
static void ledBurst(uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    ledOn();  delay(120);
    ledOff(); delay(180);
  }
  delay(900);   // clear gap so the burst is easy to count
}

static String formatUptime(uint32_t ms) {
  uint32_t s = ms / 1000UL;
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%lud %02lu:%02lu:%02lu",
           (unsigned long)(s / 86400UL), (unsigned long)((s % 86400UL) / 3600UL),
           (unsigned long)((s % 3600UL) / 60UL), (unsigned long)(s % 60UL));
  return String(buffer);
}

static const char *resetReasonText(int reason) {
  switch (reason) {
    case 1:  return "power-on";
    case 3:  return "software restart";
    case 4:  return "PANIC (crash)";
    case 5:  return "interrupt watchdog";
    case 6:  return "task watchdog";
    case 7:  return "RTC watchdog";
    case 12: return "BROWNOUT (power)";
    default: return "other";
  }
}

static void handleRoot() {
  String html;
  html.reserve(2200);
  html += "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='1'>";
  html += "<title>VibScope AP Test</title><style>";
  html += "body{margin:0;min-height:100vh;background:#07101b;color:#edf6ff;";
  html += "font-family:Consolas,monospace;display:flex;align-items:center;justify-content:center}";
  html += ".card{border:1px solid #263a50;border-radius:16px;background:#101c2a;";
  html += "padding:28px 32px;text-align:center;min-width:290px}";
  html += "h1{margin:0 0 4px;font-size:18px;color:#49b7ff}";
  html += ".sub{margin:0 0 18px;color:#9db0c3;font-size:12px}";
  html += ".clock{font-size:42px;font-weight:700}";
  html += ".label{color:#9db0c3;font-size:11px;text-transform:uppercase;";
  html += "letter-spacing:.08em;margin-top:16px}.val{font-size:19px;font-weight:600}";
  html += ".row{display:flex;gap:22px;justify-content:center;margin-top:16px}";
  html += ".row div{font-size:12px;color:#9db0c3}";
  html += ".row b{display:block;font-size:15px;color:#edf6ff;margin-top:3px}";
  html += "</style></head><body><div class='card'>";
  html += "<h1>VibScope AP Test</h1><p class='sub'>minimal firmware &mdash; AP only</p>";
  html += "<div class='clock' id='clock'>--:--:--</div>";
  html += "<div class='label'>Uptime since boot</div>";
  html += "<div class='val'>" + formatUptime(millis()) + "</div>";
  html += "<div class='row'>";
  html += "<div>Resets<b>" + String(bootsSincePower - 1) + "</b></div>";
  html += "<div>Clients<b>" + String(WiFi.softAPgetStationNum()) + "</b></div>";
  html += "<div>Heap<b>" + String(ESP.getFreeHeap() / 1024) + " KB</b></div>";
  html += "</div>";
  html += "<div class='label'>Last reset</div><div class='val'>";
  html += String(resetReasonText((int)esp_reset_reason())) + "</div>";
  html += "</div><script>function t(){var d=new Date();";
  html += "document.getElementById('clock').textContent=d.toTimeString().slice(0,8);}";
  html += "t();setInterval(t,1000);</script></body></html>";
  server.send(200, "text/html; charset=UTF-8", html);
}

static void handleNotFound() {
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  // This image is also a UART-contention isolation test. Explicitly release
  // the board pins labelled TX/RX so neither can drive the attached converter.
  Serial.end();
  pinMode(1, INPUT);
  pinMode(3, INPUT);

  // Count boots since power was applied. A true power-off wipes RTC memory,
  // so the magic will be wrong and the counter restarts at 1.
  if (bootMagic != BOOT_MAGIC) {
    bootMagic = BOOT_MAGIC;
    bootsSincePower = 1;
  } else {
    bootsSincePower++;
  }

  // Flash the boot count BEFORE touching the radio, so it is visible even if
  // WiFi bring-up is what kills the board.
  ledBurst(bootsSincePower);

  // AP only. No STA, so the radio never scans and never follows a router
  // onto another channel.
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(150);

  for (uint8_t attempt = 0; attempt < 10 && !apStarted; ++attempt) {
    if (attempt > 0) delay(150);
    apAttempts++;
    if (!WiFi.softAPConfig(apIp, apIp, apMask)) continue;
    apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4);
  }
  esp_wifi_set_max_tx_power(AP_TX_POWER);

  Serial.printf("softAP started=%d after %u attempt(s)\n", apStarted ? 1 : 0, apAttempts);
  Serial.printf("SSID: %s   IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web server up -> http://192.168.4.1");
  Serial.println("================================");
}

void loop() {
  server.handleClient();

  // ---- LED status, non-blocking -------------------------------------------
  // Patterns are described in the header comment at the top of this file.
  const uint32_t now = millis();
  if (!apStarted) {
    // AP failed: very fast continuous blink.
    if ((now / 100UL) % 2UL) ledOn(); else ledOff();
  } else if (WiFi.softAPgetStationNum() > 0) {
    // Client connected: two quick blinks, then a pause.
    const uint32_t phase = now % 1600UL;
    if (phase < 120UL || (phase >= 300UL && phase < 420UL)) ledOn(); else ledOff();
  } else {
    // AP up, idle: slow steady blink, one per second.
    const uint32_t phase = now % 1000UL;
    if (phase < 120UL) ledOn(); else ledOff();
  }

  // Serial heartbeat, for when a console IS available.
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 1000UL) {
    lastPrint = now;
    Serial.printf("[%s] boots=%lu ap=%d clients=%d heap=%u\n",
                  formatUptime(now).c_str(), (unsigned long)bootsSincePower,
                  apStarted ? 1 : 0, (int)WiFi.softAPgetStationNum(),
                  ESP.getFreeHeap());
  }

  // Match the production service ceiling so this AP-only image is a valid
  // thermal isolation test instead of a 100%-CPU busy loop.
  delay(10);
}
