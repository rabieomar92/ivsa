#include "status_led.h"
#include <assert.h>
#include <stdio.h>

unsigned long testNow = 0;
int testPin = -1, testLevel = -1;
TestTimer testTimer = {};
bool testCreateFails = false, testStartFails = false, testDeleted = false;

int main() {
  assert(statusLedService.begin());
  statusLedService.setStatus(true, true, true, true);
  // No further calls from loop(): independently dispatched timer keeps blinking.
  for (testNow = 0; testNow < 12000; testNow += 20) {
    testTimer.callback(testTimer.arg);
    assert(testPin == 25);
    assert(testLevel == (testNow % 1000 < 500 ? STATUS_LED_ON : STATUS_LED_OFF));
  }
  String status;
  statusLedService.appendStatus(status, testNow);
  assert(status.value.find("\"mode\":\"healthy\"") != std::string::npos);
  assert(status.value.find("\"timerActive\":true") != std::string::npos);
  assert(status.value.find("\"serviceCount\":600") != std::string::npos);
  assert(status.value.find("\"lastServiceAgeMs\":20") != std::string::npos);
  assert(status.value.find("\"outputChangeCount\":24") != std::string::npos);
  assert(status.value.find("\"lastOutputChangeAgeMs\":500") != std::string::npos);
  String concurrentStatus;
  // HTTP captured its clock 7ms before the most recent timer callback.
  statusLedService.appendStatus(concurrentStatus, 11973);
  assert(concurrentStatus.value.find("\"lastServiceAgeMs\":20") != std::string::npos);
  // A physical sensor loss must override the healthy pattern even with all
  // network links up. It persists across timer ticks until a successful read.
  statusLedService.setStatus(true, true, true, false);
  const unsigned long faultStart = testNow;
  const bool sensorSlots[] = {true, false, true, false, true, false,
                            true, false, false, false, false, false};
  for (unsigned slot = 0; slot < 24; ++slot) {
    testNow = faultStart + slot * 500;
    testTimer.callback(testTimer.arg);
    assert(testLevel == (sensorSlots[slot % 12] ? STATUS_LED_ON : STATUS_LED_OFF));
  }
  String faultStatus;
  statusLedService.appendStatus(faultStatus, testNow);
  assert(faultStatus.value.find("\"mode\":\"sensor_fault\"") != std::string::npos);
  assert(faultStatus.value.find("\"sensorOk\":false") != std::string::npos);
  statusLedService.setStatus(true, true, true, true);
  testTimer.callback(testTimer.arg); assert(testLevel == STATUS_LED_ON);
  testNow += 500; testTimer.callback(testTimer.arg); assert(testLevel == STATUS_LED_OFF);
  String recoveredStatus;
  statusLedService.appendStatus(recoveredStatus, testNow);
  assert(recoveredStatus.value.find("\"mode\":\"healthy\"") != std::string::npos);
  // Network-state changes are picked up by the timer, not driven by loop timing.
  statusLedService.setStatus(false, false, false, false);
  testTimer.callback(testTimer.arg); assert(testLevel == STATUS_LED_ON);
  testNow += 500; testTimer.callback(testTimer.arg); assert(testLevel == STATUS_LED_OFF);
  StatusLedService creationFailure;
  testCreateFails = true; assert(!creationFailure.begin());
  creationFailure.setStatus(true, true, true, true); assert(testLevel == STATUS_LED_ON);
  testNow += 500;
  creationFailure.setStatus(true, true, true, true); assert(testLevel == STATUS_LED_OFF);
  testCreateFails = false; testStartFails = true;
  StatusLedService startFailure;
  assert(!startFailure.begin()); assert(testDeleted);
  StatusLedService rollover;
  testNow = UINT32_MAX - 10U;
  rollover.setStatus(true, true, true, true);
  testNow = 9;
  String rolloverStatus;
  rollover.appendStatus(rolloverStatus, 0);
  assert(rolloverStatus.value.find("\"lastServiceAgeMs\":20") != std::string::npos);
  puts("PASS: independent LED timer, output transitions, concurrent status timestamps, clock rollover and failure fallback");
}
