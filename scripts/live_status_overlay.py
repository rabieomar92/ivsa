"""Live setup status for the maintained source and staged Achilles snapshot."""


def apply_live_status(source):
    replacements = (
        ('#include "status_led.h"', '#include "status_led.h"\n#include "live_status_page.h"'),
        ("<section class='section'><h2>Live Status</h2><div class='status'>",
         "<section class='section' id='liveStatus'><h2>Live Status<small id='liveStatusNote' role='status' aria-live='polite'>Connecting to live status...</small></h2><div class='status'>"),
        ('          "</script></main></body></html>";',
         '          "</script>";\n  html += FPSTR(IVSS_LIVE_STATUS_SCRIPT);\n'
         '  html += "</main></body></html>";'),
        ('  json += "\\\"rssi\\\":";',
         '  json += "\\\"lastDisconnectReason\\\":" + String((unsigned)lastStaDisconnectReason) + ",";\n'
         '  json += "\\\"lastDisconnectText\\\":\\\"" + jsonEscape(staDisconnectReasonText(lastStaDisconnectReason)) + "\\\",";\n'
         '  json += "\\\"rssi\\\":";'),
        ('  json += "\\\"uart\\\":2,";',
         '  json += "\\\"pollIntervalMs\\\":" + String(sensorPollIntervalMs) + ",";\n'
         '  json += "\\\"actualIntervalMs\\\":" + String(lastActualSensorPollIntervalMs) + ",";\n'
         '  json += "\\\"lastReadAgeMs\\\":";\n'
         '  json += lastSensorReadOkMs ? String(statusNow - lastSensorReadOkMs) : String("null");\n'
         '  json += ",\\\"lastReadOk\\\":";\n'
         '  json += lastSensorAcquisitionOk ? "true" : "false";\n'
         '  json += ",\\\"uart\\\":2,";'),
        ('  server.send(200, "application/json", json);\n}\n\nvoid handleNotFound()',
         '  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");\n'
         '  server.send(200, "application/json", json);\n}\n\nvoid handleNotFound()'),
    )
    for old, new in replacements:
        if source.count(old) != 1:
            raise RuntimeError('Live status overlay needs review: ' + old)
        source = source.replace(old, new)
    return source
