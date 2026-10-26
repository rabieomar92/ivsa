#pragma once

#include <Arduino.h>

static const char IVSS_LIVE_STATUS_SCRIPT[] PROGMEM = R"IVSSJS(
<script>
(() => {
  const section = document.getElementById('liveStatus');
  const grid = section.querySelector('.status');
  const note = document.getElementById('liveStatusNote');
  const rows = new Map();
  for (const row of grid.querySelectorAll('.kv')) {
    rows.set(row.querySelector('b').textContent, row);
  }
  let timer, inFlight = false, lastUpdate = '';
  function field(label, value, tone) {
    let row = rows.get(label);
    if (!row) {
      row = document.createElement('div'); row.className = 'kv';
      const title = document.createElement('b'); title.textContent = label;
      row.appendChild(title); row.appendChild(document.createElement('span'));
      grid.appendChild(row); rows.set(label, row);
    }
    row.hidden = false;
    const span = row.querySelector('span');
    span.textContent = '';
    if (tone) {
      const dot = document.createElement('i'); dot.className = 'dot ' + tone;
      span.appendChild(dot);
    }
    span.appendChild(document.createTextNode(value));
  }
  function render(s) {
    const ap = s.ap, sta = s.sta, mqtt = s.mqtt, bus = s.sensorBus;
    field('Sensor', s.sensorModel + ' | ' + s.protocol);
    field('Sensor Profile', s.sensorProfile + ' | ' + s.registerMapVersion + ' | ' + s.sensorProfileSource);
    field('Setup AP', ap.ssid + ' | ' + (ap.ip || '-') + ' | clients ' + ap.clients,
      ap.enabled ? 'ok' : 'bad');
    field('STA WiFi', (sta.connected ? 'connected' : sta.configured ? 'disconnected/retrying' : 'not configured') +
      ' | ' + sta.ssid + ' | ' + (sta.ip || '-'), sta.connected ? 'ok' : sta.configured ? 'bad' : 'warn');
    field('RSSI', sta.connected ? sta.rssi + ' dBm' : '-');
    field('MQTT', (!mqtt.broker ? 'not configured' : mqtt.connected ? 'broker connected' : 'disconnected/retrying') +
      ' | ' + mqtt.broker + ':' + mqtt.port, mqtt.connected ? 'ok' : mqtt.broker ? 'bad' : 'warn');
    field('Server Heartbeat', (mqtt.serverHeartbeatFresh ? 'online' : mqtt.serverHeartbeatSeen ? 'overdue' : 'waiting') +
      (mqtt.serverHeartbeatAgeMs == null ? '' : ' | received ' + (mqtt.serverHeartbeatAgeMs / 1000).toFixed(1) + 's ago'),
      mqtt.serverHeartbeatFresh ? 'ok' : 'warn');
    const fresh = bus.lastReadOk && bus.lastReadAgeMs != null &&
      bus.lastReadAgeMs <= Math.max(5000, bus.pollIntervalMs * 3, bus.actualIntervalMs * 3);
    field('Sensor Communication', (fresh ? 'online' : bus.successCount ? 'stale / communication fault' : 'waiting / communication fault') +
      ' | reads ' + bus.successCount + ' | failures ' + bus.failureCount +
      (bus.lastReadAgeMs == null ? '' : ' | last good read ' + (bus.lastReadAgeMs / 1000).toFixed(1) + 's ago') +
      (bus.lastError === 'none' ? '' : ' | ' + bus.lastError), fresh ? 'ok' : 'bad');
    field('Acquisition', 'Target ' + bus.pollIntervalMs + ' ms | actual ' + bus.actualIntervalMs + ' ms');
    field('Last WiFi Failure', sta.lastDisconnectReason ? 'reason ' + sta.lastDisconnectReason + ' - ' + sta.lastDisconnectText :
      'no association attempt has failed yet', 'warn');
    rows.get('Last WiFi Failure').hidden = sta.connected || !sta.configured;
    document.getElementById('currentProfile').value = s.sensorProfile;
    lastUpdate = new Date().toLocaleTimeString();
    note.textContent = 'Live | updated ' + lastUpdate + ' | refreshes every 2 seconds';
  }
  async function refresh() {
    clearTimeout(timer);
    if (inFlight) return;
    if (document.hidden || document.getElementById('configButton').disabled ||
        document.getElementById('profileButton').disabled) {
      timer = setTimeout(refresh, 2000); return;
    }
    inFlight = true;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 8000);
    let retryMs = 2000;
    try {
      const response = await fetch('/status', {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw new Error('Status unavailable');
      render(await response.json());
    } catch (_) {
      note.textContent = 'Live status unavailable | ' + (lastUpdate ? 'last update ' + lastUpdate : 'waiting for device') +
        ' | displayed values may be stale; reconnecting automatically';
      for (const dot of grid.querySelectorAll('.dot')) dot.className = 'dot warn';
      retryMs = 5000;
    } finally {
      clearTimeout(timeout); inFlight = false;
      timer = setTimeout(refresh, retryMs);
    }
  }
  document.addEventListener('visibilitychange', () => { if (!document.hidden) refresh(); });
  refresh();
})();
</script>
)IVSSJS";
