#pragma once

#include <stdint.h>

enum class StatusLedMode : uint8_t {
  Healthy,
  WifiDisconnected,
  MqttDisconnected,
  HeartbeatMissing,
  SensorFault
};

// Cooperative, non-blocking patterns. Each new mode starts with an ON pulse.
class StatusLedPattern {
 public:
  bool update(uint32_t now, bool wifi, bool mqtt, bool heartbeat, bool sensorOk) {
    const StatusLedMode next = !wifi ? StatusLedMode::WifiDisconnected
      : !mqtt ? StatusLedMode::MqttDisconnected
      : !heartbeat ? StatusLedMode::HeartbeatMissing
      : !sensorOk ? StatusLedMode::SensorFault
      : StatusLedMode::Healthy;
    if (!initialized_ || next != mode_) {
      initialized_ = true;
      mode_ = next;
      startedMs_ = now;
    }

    // Unsigned subtraction preserves phase across millis() wraparound.
    const uint32_t elapsed = now - startedMs_;
    switch (mode_) {
      case StatusLedMode::WifiDisconnected: {
        // 500 ON, 2000 OFF, 500 ON, 1000 OFF.
        const uint32_t phase = elapsed % 4000U;
        return phase < 500U || (phase >= 2500U && phase < 3000U);
      }
      case StatusLedMode::MqttDisconnected: {
        // Two (500 ON, 500 OFF) pulses, then an additional 2000 OFF.
        const uint32_t phase = elapsed % 4000U;
        return phase < 500U || (phase >= 1000U && phase < 1500U);
      }
      case StatusLedMode::HeartbeatMissing: {
        // Three (500 ON, 500 OFF) pulses, then an additional 2000 OFF.
        const uint32_t phase = elapsed % 5000U;
        return phase < 3000U && phase % 1000U < 500U;
      }
      case StatusLedMode::SensorFault: {
        // Four (500 ON, 500 OFF) pulses, then an additional 2000 OFF.
        const uint32_t phase = elapsed % 6000U;
        return phase < 4000U && phase % 1000U < 500U;
      }
      case StatusLedMode::Healthy:
        return elapsed % 1000U < 500U;
    }
    return false;
  }

 private:
  StatusLedMode mode_ = StatusLedMode::Healthy;
  uint32_t startedMs_ = 0;
  bool initialized_ = false;
};
