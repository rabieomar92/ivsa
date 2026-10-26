#pragma once

#include <Arduino.h>
#include <cstring>

// Capture bytes before echo removal and frame scanning. No allocation or serial
// logging occurs in the transaction; hex formatting happens only in /status.
struct SensorRxTrace {
  uint8_t request[8] = {};
  uint8_t bytes[64] = {};
  size_t captured = 0;
  uint32_t total = 0;
  unsigned long startedMs = 0;
  bool present = false;

  void record(uint8_t value) {
    if (captured < sizeof(bytes)) bytes[captured++] = value;
    if (total < UINT32_MAX) ++total;
  }
};

static SensorRxTrace sensorLatestRxTrace;
static SensorRxTrace sensorLastNonemptyRxTrace;

struct SensorRxTraceScope {
  explicit SensorRxTraceScope(const uint8_t (&request)[8]) {
    sensorLatestRxTrace = SensorRxTrace{};
    memcpy(sensorLatestRxTrace.request, request, sizeof(request));
    sensorLatestRxTrace.startedMs = millis();
    sensorLatestRxTrace.present = true;
  }

  ~SensorRxTraceScope() {
    // Preserve evidence through subsequent timeouts and UART recovery pauses.
    if (sensorLatestRxTrace.total > 0) {
      sensorLastNonemptyRxTrace = sensorLatestRxTrace;
    }
  }
};

inline void appendSensorTraceHex(String &json, const uint8_t *bytes, size_t count) {
  constexpr char hex[] = "0123456789ABCDEF";
  json += '"';
  for (size_t i = 0; i < count; ++i) {
    if (i) json += ' ';
    json += hex[bytes[i] >> 4];
    json += hex[bytes[i] & 15];
  }
  json += '"';
}

inline void appendSensorTrace(String &json, const SensorRxTrace &trace) {
  if (!trace.present) {
    json += "null";
    return;
  }
  json += "{\"ageMs\":";
  json += String((unsigned long)(millis() - trace.startedMs));
  json += ",\"requestHex\":";
  appendSensorTraceHex(json, trace.request, sizeof(trace.request));
  json += ",\"rxHex\":";
  appendSensorTraceHex(json, trace.bytes, trace.captured);
  json += ",\"rxByteCount\":";
  json += String(trace.total);
  json += ",\"truncated\":";
  json += trace.total > trace.captured ? "true" : "false";
  json += '}';
}

inline void appendSensorRxTraceStatus(String &json) {
  json += ",\"rxTraceVersion\":1,\"latestTransaction\":";
  appendSensorTrace(json, sensorLatestRxTrace);
  json += ",\"lastNonemptyTransaction\":";
  appendSensorTrace(json, sensorLastNonemptyRxTrace);
}
