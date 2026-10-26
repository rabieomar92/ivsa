#include "status_led_pattern.h"
#include <assert.h>
#include <stdio.h>

int main() {
  // Expected state in each 500 ms slot, including the additional fault pauses.
  const bool wifi[] = {true, false, false, false, false, true, false, false};
  const bool mqtt[] = {true, false, true, false, false, false, false, false};
  const bool heartbeat[] = {true, false, true, false, true, false, false, false, false, false};
  const bool sensor[] = {true, false, true, false, true, false, true, false, false, false, false, false};
  const bool healthy[] = {true, false};
  struct Scenario { bool wifi; bool mqtt; bool heartbeat; bool sensorOk; const bool *slots; unsigned count; };
  const Scenario scenarios[] = {
    {false, true, true, false, wifi, 8}, // WiFi failure takes priority.
    {true, false, true, false, mqtt, 8}, // MQTT failure takes priority.
    {true, true, false, false, heartbeat, 10},
    {true, true, true, false, sensor, 12},
    {true, true, true, true, healthy, 2}
  };
  for (const auto &scenario : scenarios) {
    StatusLedPattern pattern;
    // Exercise every millisecond, two full repeats, including timer rollover.
    const uint32_t start = UINT32_MAX - 1700U;
    for (uint32_t elapsed = 0; elapsed < scenario.count * 1000U; ++elapsed) {
      assert(pattern.update(start + elapsed, scenario.wifi, scenario.mqtt,
                            scenario.heartbeat, scenario.sensorOk) ==
             scenario.slots[(elapsed / 500U) % scenario.count]);
    }
  }
  StatusLedPattern transitions;
  assert(transitions.update(100U, true, true, true, true));
  assert(!transitions.update(600U, true, true, true, true));
  assert(transitions.update(750U, false, false, false, false));
  assert(!transitions.update(1250U, false, false, false, false));
  assert(transitions.update(1251U, true, false, false, false));
  assert(!transitions.update(1751U, true, false, false, false));
  assert(transitions.update(1752U, true, true, false, false));
  assert(!transitions.update(2252U, true, true, false, false));
  assert(transitions.update(2253U, true, true, true, false));
  assert(!transitions.update(2753U, true, true, true, false));
  assert(transitions.update(2754U, true, true, true, true));
  puts("PASS: LED timing, repeat cycles, fault priority, transitions and timer rollover");
}
