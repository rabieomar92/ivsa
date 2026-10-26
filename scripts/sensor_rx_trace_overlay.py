"""Apply receive diagnostics to both maintained and staged Achilles sources."""


def apply_sensor_rx_trace(source):
    replacements = (
        ('#include "sensor_profile.h"',
         '#include "sensor_profile.h"\n#include "sensor_rx_trace.h"'),
        ('  rs485SetTransmit(true);\n  delayMicroseconds(200);',
         '  SensorRxTraceScope sensorRxTraceScope(req);\n'
         '  rs485SetTransmit(true);\n  delayMicroseconds(200);'),
        ('      rx[rxLen++] = (uint8_t)Serial2.read();',
         '      const uint8_t receivedByte = (uint8_t)Serial2.read();\n'
         '      sensorLatestRxTrace.record(receivedByte);\n'
         '      rx[rxLen++] = receivedByte;'),
        ('  json += "\\\"lastError\\\":\\\"" + jsonEscape(String(lastModbusErrorReason)) + "\\\"";',
         '  json += "\\\"lastError\\\":\\\"" + jsonEscape(String(lastModbusErrorReason)) + "\\\"";\n'
         '  appendSensorRxTraceStatus(json);'),
    )
    for old, new in replacements:
        if source.count(old) != 1:
            raise RuntimeError("Sensor RX trace overlay needs review: " + old)
        source = source.replace(old, new)
    return source
