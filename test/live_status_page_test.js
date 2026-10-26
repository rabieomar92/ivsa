const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

class Element {
  constructor(tag) { this.tag = tag; this.children = []; this.textContent = ''; this.className = ''; }
  set textContent(value) { this._text = value; this.children = []; }
  get textContent() { return this._text + this.children.map(child => child.textContent).join(''); }
  appendChild(child) { this.children.push(child); return child; }
  querySelector(selector) { return this.querySelectorAll(selector)[0]; }
  querySelectorAll(selector) {
    return this.children.flatMap(child => [
      ...(selector[0] === '.' ? child.className.split(' ').includes(selector.slice(1)) : child.tag === selector) ? [child] : [],
      ...child.querySelectorAll(selector)
    ]);
  }
}

async function main() {
  const grid = new Element('div'); grid.className = 'status';
  const section = new Element('section'); section.appendChild(grid);
  const ids = {liveStatus: section, liveStatusNote: new Element('small'),
    configButton: {disabled: false}, profileButton: {disabled: false},
    currentProfile: {value: ''}, ssid: {value: 'unsaved user edit'}};
  const listeners = {}, timers = new Map(); let nextTimer = 0, calls = 0, fail = false;
  const status = {sensorModel: 'WTVB01', protocol: 'Modbus', sensorProfile: 'witmotion',
    registerMapVersion: 'v1', sensorProfileSource: 'uploaded',
    ap: {enabled: true, ssid: '<script>test</script>', ip: '192.168.4.1', clients: 1},
    sta: {connected: true, configured: true, ssid: 'wifi', ip: '192.168.0.26', rssi: -50},
    mqtt: {broker: 'host', port: 1883, connected: true, serverHeartbeatFresh: true, serverHeartbeatAgeMs: 1000},
    sensorBus: {lastReadOk: true, lastReadAgeMs: 150, pollIntervalMs: 100, actualIntervalMs: 100,
      successCount: 10, failureCount: 0, lastError: 'none'}};
  const document = {hidden: false, getElementById: id => ids[id],
    createElement: tag => new Element(tag), createTextNode: text => Object.assign(new Element('#text'), {textContent: text}),
    addEventListener: (event, handler) => { listeners[event] = handler; }};
  const context = {document, Map, Date, Math, AbortController,
    setTimeout: (fn, delay) => { const id = ++nextTimer; timers.set(id, {fn, delay}); return id; },
    clearTimeout: id => timers.delete(id),
    fetch: async (url, options) => {
      calls++; assert.equal(url, '/status'); assert.equal(options.cache, 'no-store');
      if (fail) throw new Error('offline');
      return {ok: true, json: async () => status};
    }};
  const source = fs.readFileSync('include/live_status_page.h', 'utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
  vm.runInNewContext(source, context);
  const settle = () => new Promise(resolve => setImmediate(resolve));
  await settle();
  const row = label => grid.children.find(child => child.querySelector('b').textContent === label);
  const text = label => row(label).querySelector('span').children.map(child => child.textContent).join('');
  async function tick() {
    assert.equal(timers.size, 1, 'only one scheduled request');
    const [id, timer] = [...timers][0]; timers.delete(id); timer.fn(); await settle();
  }
  assert.match(ids.liveStatusNote.textContent, /^Live \| updated/);
  assert.match(text('Sensor Communication'), /online.*reads 10/);
  assert.match(text('Setup AP'), /<script>test<\/script>/); // rendered as a text node
  assert.equal(row('Last WiFi Failure').hidden, true);
  status.sensorBus.successCount = 20; status.mqtt.serverHeartbeatFresh = false;
  status.mqtt.serverHeartbeatSeen = true; status.mqtt.serverHeartbeatAgeMs = 50000;
  await tick();
  assert.match(text('Sensor Communication'), /reads 20/);
  assert.doesNotMatch(text('Sensor Communication'), /reads 10/);
  assert.match(text('Server Heartbeat'), /overdue/);
  status.sensorBus.lastReadOk = false;
  await tick(); assert.match(text('Sensor Communication'), /stale/);
  fail = true; await tick();
  assert.match(ids.liveStatusNote.textContent, /unavailable.*may be stale/);
  assert.equal([...timers.values()][0].delay, 5000);
  fail = false; await tick(); assert.match(ids.liveStatusNote.textContent, /^Live \| updated/);
  let previousCalls = calls; document.hidden = true; await tick(); assert.equal(calls, previousCalls);
  document.hidden = false; listeners.visibilitychange(); await settle(); assert.equal(calls, previousCalls + 1);
  previousCalls = calls; ids.configButton.disabled = true; await tick(); assert.equal(calls, previousCalls);
  assert.equal(ids.ssid.value, 'unsaved user edit');
  console.log('PASS: live updates, sensor/heartbeat faults, stale/recovery, single polling timer, visibility and edit preservation');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
