#pragma once
#include <stdint.h>
#define ESP_OK 0
#define ESP_TIMER_TASK 0
struct esp_timer_create_args_t {
  void (*callback)(void *); void *arg; int dispatch_method;
  const char *name; bool skip_unhandled_events;
};
struct TestTimer { void (*callback)(void *); void *arg; };
using esp_timer_handle_t = TestTimer *;
extern TestTimer testTimer;
extern bool testCreateFails, testStartFails, testDeleted;
inline int esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *timer) {
  if (testCreateFails) return -1;
  testTimer = {args->callback, args->arg}; *timer = &testTimer; return ESP_OK;
}
inline int esp_timer_start_periodic(esp_timer_handle_t, uint64_t interval) {
  return interval == 20000 && !testStartFails ? ESP_OK : -1;
}
inline int esp_timer_delete(esp_timer_handle_t) { testDeleted = true; return ESP_OK; }
