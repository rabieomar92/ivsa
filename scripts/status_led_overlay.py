"""Share the status LED implementation with the immutable Achilles build."""

import re


def apply_status_led(source):
    start = source.index('void updateLed() {')
    end = source.index('\nuint64_t monotonicMillis()', start)
    source = source[:start] + '''void updateLed() {
  const unsigned long now = millis();
#if defined(ACHILLES_INVERT_DIAGNOSTIC)
  // UART-paused hardware diagnostics retain an independent loop heartbeat.
  statusLedService.setStatus(true, true, true, true);
#else
  statusLedService.setStatus(
    WiFi.status() == WL_CONNECTED, mqttClient.connected(),
    nodeServerHeartbeatFresh(now), lastSensorAcquisitionOk);
#endif
}
''' + source[end:]
    # Remove state and timing constants belonging to the replaced LED routine.
    names = ('LED_SLOW_BLINK_MS', 'LED_MQTT_BLINK_MS',
             'LED_DOUBLE_BLINK_PERIOD_MS', 'LED_CONNECTED_INTERVAL_MS',
             'LED_CONNECTED_PULSE_MS', 'lastLedToggleMs', 'ledPulseEndMs',
             'nextConnectedPulse', 'ledState')
    for name in names:
        source, count = re.subn(
            r'^.*\b' + name + r'\s*=.*\n', '', source, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError('Status LED overlay needs review: ' + name)
    return apply_led_timer_hooks(source)


def apply_led_timer_hooks(source):
    replacements = (
        ('unsigned long lastSensorReadOkMs    = 0;\n',
         'unsigned long lastSensorReadOkMs    = 0;\n'
         '// Latest completed acquisition, including failures; cached data is not success.\n'
         'bool lastSensorAcquisitionOk = false;\n'),
        ('  if (!ok) {\n    if (sensorReadFailCount < UINT8_MAX)',
         '  lastSensorAcquisitionOk = ok;\n'
         '  if (!ok) {\n    if (sensorReadFailCount < UINT8_MAX)'),
        ('    lastSensorReadOkMs = 0;\n',
         '    lastSensorReadOkMs = 0;\n    lastSensorAcquisitionOk = false;\n'),
        ('  digitalWrite(LED_PIN, STATUS_LED_OFF); // off\n',
         '  digitalWrite(LED_PIN, STATUS_LED_OFF); // off\n'
         '  statusLedService.begin();\n'),
        ('  json += ",\\\"reliability\\\":{";',
         '  statusLedService.appendStatus(json, statusNow);\n'
         '  json += ",\\\"reliability\\\":{";'),
    )
    for old, new in replacements:
        if source.count(old) != 1:
            raise RuntimeError('LED timer overlay needs review: ' + old)
        source = source.replace(old, new)
    return source
