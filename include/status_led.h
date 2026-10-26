#pragma once

#include <Arduino.h>
#include "status_led_pattern.h"
#include <atomic>
#include "esp_timer.h"

// External status LED: GPIO25 supplies Vin, with LED ground connected to GND.
#ifndef LED_PIN
#define LED_PIN 25
#endif
#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 0
#endif

constexpr int STATUS_LED_ON = LED_ACTIVE_LOW ? LOW : HIGH;
constexpr int STATUS_LED_OFF = LED_ACTIVE_LOW ? HIGH : LOW;

// Only loop() reads the network clients. The independent timer consumes a
// compact atomic snapshot and never touches WiFi, MQTT, Serial, or the sensor.
class StatusLedService {
 public:
  bool begin() {
    if (timerActive_) return true;
    esp_timer_create_args_t args = {};
    args.callback = &StatusLedService::callback;
    args.arg = this;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "ivss-led";
    args.skip_unhandled_events = true;
    if (esp_timer_create(&args, &timer_) != ESP_OK) return false;
    if (esp_timer_start_periodic(timer_, 20000) != ESP_OK) {
      esp_timer_delete(timer_);
      timer_ = nullptr;
      return false;
    }
    timerActive_ = true;
    return true;
  }

  void setStatus(bool wifi, bool mqtt, bool heartbeat, bool sensorOk) {
    flags_.store((wifi ? 1U : 0U) | (mqtt ? 2U : 0U) | (heartbeat ? 4U : 0U) |
                 (sensorOk ? 8U : 0U));
    if (!timerActive_) tick(); // Retain a usable fallback if timer allocation fails.
  }

  void appendStatus(String &json, unsigned long /*statusRequestMs*/) const {
    // Read timer timestamps BEFORE sampling the current clock. The HTTP handler's
    // earlier timestamp can precede a concurrent callback and underflow the age.
    const uint32_t lastService = lastServiceMs_.load();
    const uint32_t lastChange = lastChangeMs_.load();
    const uint32_t now = millis();
    const uint32_t flags = flags_.load();
    const char *mode = !(flags & 1U) ? "wifi_disconnected"
      : !(flags & 2U) ? "mqtt_disconnected"
      : !(flags & 4U) ? "heartbeat_missing"
      : !(flags & 8U) ? "sensor_fault" : "healthy";
    json += ",\"statusLed\":{\"gpio\":";
    json += String(LED_PIN);
    json += ",\"driverVersion\":4,\"mode\":\"";
    json += mode;
    json += "\",\"sensorOk\":";
    json += (flags & 8U) ? "true" : "false";
    json += ",\"timerActive\":";
    json += timerActive_ ? "true" : "false";
    json += ",\"commandedOn\":";
    json += commandedOn_.load() ? "true" : "false";
    json += ",\"serviceCount\":";
    json += String(serviceCount_.load());
    json += ",\"outputChangeCount\":";
    json += String(outputChangeCount_.load());
    json += ",\"lastOutputChangeAgeMs\":";
    json += String((uint32_t)(now - lastChange));
    json += ",\"lastServiceAgeMs\":";
    json += String((uint32_t)(now - lastService));
    json += '}';
  }

 private:
  static void callback(void *arg) {
    static_cast<StatusLedService *>(arg)->tick();
  }
  void tick() {
    const uint32_t now = millis();
    const uint32_t flags = flags_.load();
    const bool on = pattern_.update(now, flags & 1U, flags & 2U, flags & 4U, flags & 8U);
    digitalWrite(LED_PIN, on ? STATUS_LED_ON : STATUS_LED_OFF);
    if (commandedOn_.exchange(on) != on) {
      lastChangeMs_.store(now);
      outputChangeCount_.fetch_add(1);
    }
    lastServiceMs_.store(now);
    serviceCount_.fetch_add(1);
  }
  StatusLedPattern pattern_;
  esp_timer_handle_t timer_ = nullptr;
  bool timerActive_ = false;
  std::atomic<uint32_t> flags_{0}, lastServiceMs_{0}, serviceCount_{0};
  std::atomic<uint32_t> lastChangeMs_{0}, outputChangeCount_{0};
  std::atomic<bool> commandedOn_{false};
};

static StatusLedService statusLedService;
