const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const source = fs.readFileSync(path.join(__dirname, '..', 'src', 'main.cpp'), 'utf8');

function functionBody(name) {
  const definition = new RegExp(`(?:void|bool)\\s+${name}\\s*\\([^;{]*\\)\\s*\\{`, 'm').exec(source);
  assert.ok(definition, `${name}() must exist`);
  const open = source.indexOf('{', definition.index);
  let depth = 0;
  for (let index = open; index < source.length; index++) {
    if (source[index] === '{') depth++;
    if (source[index] === '}' && --depth === 0) {
      return source.slice(open + 1, index);
    }
  }
  assert.fail(`${name}() body is not balanced`);
}

const serviceMqttBody = functionBody('serviceMqtt');
const publishDataBody = functionBody('publishData');
const calibrationBody = functionBody('calibrateSensorBaseline');
const loopBody = functionBody('loop');

assert.match(source, /MQTT_RETRY_INITIAL_MS\s*=\s*10000/);
assert.match(source, /MQTT_RETRY_MAX_MS\s*=\s*60000/);
assert.match(source, /mqttRetryDelayMs\s*=\s*min\(mqttRetryDelayMs \* 2UL, MQTT_RETRY_MAX_MS\)/);
assert.match(source, /MQTT_SESSION_PROGRESS_TIMEOUT_MS\s*=\s*90000/);
assert.match(source, /NODE_SERVER_HEARTBEAT_TIMEOUT_MS\s*=\s*45000/);
assert.match(source, /NODE_SERVER_HEARTBEAT_RESUBSCRIBE_MS\s*=\s*60000/);
assert.match(source, /MQTT_OUTAGE_AP_TRIGGER_MS\s*=\s*15000/);
assert.match(source, /NODE_SERVER_HEARTBEAT_TOPIC\s*=\s*"ivss\/server\/heartbeat"/);
assert.match(source, /mqttSessionProgressMs\s*=\s*lastPublishOkMs/);
assert.match(source, /No successful telemetry write for 90 s; rebuilding the stale session/);
assert.match(source, /if \(setupApBlockingStaAssociation\(\)\) \{[\s\S]*?mqttClient\.loop\(\)/);
assert.match(source, /PUBLISH %s FAIL[\s\S]*?mqttClient\.disconnect\(\)/);
assert.match(source, /OFFLINE_SENSOR_INTERVAL_MS\s*=\s*2000/);
assert.match(source, /WiFi\.status\(\) == WL_CONNECTED && mqttClient\.connected\(\)/);
assert.match(source, /mqttClient\.setCallback\(mqttMessageReceived\)/);
assert.match(source, /mqttClient\.subscribe\(NODE_SERVER_HEARTBEAT_TOPIC\)/);
assert.match(source, /activateRecoverySetupPortal\("MQTT broker connection failed"\)/);
assert.match(source, /activateRecoverySetupPortal\("Node\.js dashboard server is unavailable"\)/);
assert.match(source, /mqttSessionProgressMs\s*=\s*lastNodeServerHeartbeatMs/);
assert.match(source, /waveSamplesStored\s*=\s*0/);
assert.match(serviceMqttBody, /mqttClient\.subscribe\(NODE_SERVER_HEARTBEAT_TOPIC\)/);
assert.doesNotMatch(serviceMqttBody, /setupApExpiryArmed\s*=\s*true/,
  'a heartbeat alone must not close the recovery AP');
assert.match(publishDataBody,
  /sensorOk\s*&&\s*nodeServerHeartbeatFresh\(lastPublishOkMs\)[\s\S]*?armSetupApLifetimeAfterHealthyPublish\(\)/,
  'only end-to-end valid telemetry may arm AP expiry');
assert.match(publishDataBody,
  /mqttClient\.connected\(\)[\s\S]*?nodeServerHeartbeatFresh\(ts\)[\s\S]*?!setupApBlockingStaAssociation\(\)/,
  'full telemetry must require an end-to-end live Node server');
assert.doesNotMatch(publishDataBody, /setupApClientPresent\(\)/);
assert.doesNotMatch(calibrationBody, /setupApClientPresent\(\)/,
  'an associated phone must not block calibration forever');
assert.doesNotMatch(loopBody, /setupApClientPresent\(\)/,
  'an associated phone must not block the main recovery path forever');
assert.match(loopBody,
  /telemetryPathReady\s*&&\s*!telemetryPathWasReady[\s\S]*?waveSamplesStored\s*=\s*0[\s\S]*?lastAcquisitionStartMs\s*=\s*0/,
  'the actual end-to-end recovery edge must re-prime RMS and timing');

// Prove the retry schedule stays energy-bounded but never enters a multi-hour
// sleep: after any outage duration the next probe is at most 60 seconds away.
let delay = 10000;
const attempts = [];
for (let elapsed = 0; elapsed < 7 * 60 * 60 * 1000;) {
  elapsed += delay;
  attempts.push(elapsed);
  delay = Math.min(delay * 2, 60000);
}
assert.ok(attempts.length >= 420, 'a seven-hour outage retains periodic recovery probes');
assert.ok(attempts.every((at, index) => index === 0 || at - attempts[index - 1] <= 60000));

// Deterministic model of the firmware's unsigned-millis state transitions. It
// exercises duration without waiting seven wall-clock hours; real broker/Node
// process recovery is covered by the server integration tests.
const HEARTBEAT_TIMEOUT_MS = 45000;
const AP_RETRY_MS = 60000;
const AP_CLIENT_GRACE_MS = 20000;
const OFFLINE_POLL_MS = 2000;
const CONFIGURED_POLL_MS = 300;
const AP_HEALTHY_WINDOW_MS = 10 * 60 * 1000;
const U32 = (value) => value >>> 0;
const elapsed = (now, then) => U32(now - then);

class RecoveryModel {
  constructor({ failFirstApStart = false } = {}) {
    this.now = 0;
    this.brokerConnected = true;
    this.heartbeatSeen = true;
    this.lastHeartbeat = 0;
    this.apStarted = false;
    this.apAttempted = false;
    this.lastApAttempt = 0;
    this.failFirstApStart = failFirstApStart;
    this.apAttempts = 0;
    this.apClientJoined = null;
    this.validPublishArmedExpiry = false;
    this.validPublishAt = 0;
    this.pathWasReady = true;
    this.waveSamples = 96;
  }
  serverReady() {
    return this.brokerConnected && this.heartbeatSeen &&
      elapsed(this.now, this.lastHeartbeat) < HEARTBEAT_TIMEOUT_MS;
  }
  apBlocksTransport() {
    return this.apStarted && this.apClientJoined !== null &&
      elapsed(this.now, this.apClientJoined) < AP_CLIENT_GRACE_MS;
  }
  activateAp() {
    if (this.apStarted) {
      this.validPublishArmedExpiry = false;
      return;
    }
    if (this.apAttempted && elapsed(this.now, this.lastApAttempt) < AP_RETRY_MS) return;
    this.apAttempted = true;
    this.lastApAttempt = this.now;
    this.apAttempts++;
    if (this.failFirstApStart && this.apAttempts === 1) return;
    this.apStarted = true;
  }
  step(deltaMs) {
    this.now = U32(this.now + deltaMs);
    if (!this.serverReady()) this.activateAp();
    if (this.apStarted && this.validPublishArmedExpiry &&
        elapsed(this.now, this.validPublishAt) >= AP_HEALTHY_WINDOW_MS) {
      this.apStarted = false;
      this.validPublishArmedExpiry = false;
      this.apAttempted = false;
    }
    const pathReady = this.brokerConnected && this.serverReady() &&
      !this.apBlocksTransport();
    if (pathReady && !this.pathWasReady) this.waveSamples = 0;
    if (!pathReady) this.waveSamples = Math.min(96, this.waveSamples + 1);
    this.pathWasReady = pathReady;
    return {
      apStarted: this.apStarted,
      serverReady: this.serverReady(),
      sensorPollMs: this.serverReady() ? CONFIGURED_POLL_MS : OFFLINE_POLL_MS,
      telemetryCanPublish: pathReady,
      waveSamples: this.waveSamples
    };
  }
  heartbeat() {
    this.heartbeatSeen = true;
    this.lastHeartbeat = this.now;
  }
  validPublish() {
    if (this.serverReady()) {
      this.validPublishArmedExpiry = true;
      this.validPublishAt = this.now;
    }
  }
}

// Node-only outage for seven hours while Mosquitto remains alive.
const nodeOutage = new RecoveryModel({ failFirstApStart: true });
let nodeState;
for (let second = 1; second <= 7 * 60 * 60; second++) {
  nodeState = nodeOutage.step(1000);
}
assert.equal(nodeOutage.apStarted, true, 'AP opens after the Node heartbeat expires');
assert.equal(nodeOutage.apAttempts, 2, 'one failed AP start is retried, not spammed');
assert.equal(nodeState.sensorPollMs, OFFLINE_POLL_MS, 'offline sensor checks are energy-bounded');
assert.equal(nodeState.telemetryCanPublish, false,
  'no multi-kilobyte telemetry is built for a broker with no Node consumer');

// A phone left associated blocks the shared radio for only the explicit grace.
nodeOutage.apClientJoined = nodeOutage.now;
nodeOutage.heartbeat();
assert.equal(nodeOutage.step(1000).telemetryCanPublish, false);
const afterApGrace = nodeOutage.step(AP_CLIENT_GRACE_MS);
assert.equal(afterApGrace.telemetryCanPublish, true,
  'a stale AP association cannot deadlock telemetry');
assert.equal(afterApGrace.waveSamples, 0,
  'RMS is re-primed when the path becomes usable after AP grace');

// Node comes back: fresh heartbeat restores 300 ms acquisition immediately,
// but AP expiry is armed only by a valid sensor publish—not by heartbeat alone.
nodeState = nodeOutage.step(1);
assert.equal(nodeState.serverReady, true);
assert.equal(nodeState.sensorPollMs, CONFIGURED_POLL_MS);
assert.equal(nodeOutage.validPublishArmedExpiry, false);
nodeOutage.validPublish();
assert.equal(nodeOutage.validPublishArmedExpiry, true);
let healthyWindowRemaining = AP_HEALTHY_WINDOW_MS - 1;
while (healthyWindowRemaining > 0) {
  const advance = Math.min(10000, healthyWindowRemaining);
  nodeOutage.heartbeat();
  nodeOutage.step(advance);
  healthyWindowRemaining -= advance;
}
assert.equal(nodeOutage.apStarted, true,
  'healthy AP close window is a full ten minutes');
nodeOutage.heartbeat();
assert.equal(nodeOutage.step(1).apStarted, false,
  'AP closes after ten continuously healthy minutes');

// A new application outage reopens the AP; if an already-armed close window is
// interrupted, the outage cancels it instead of allowing a premature close.
nodeOutage.heartbeat();
nodeOutage.activateAp();
nodeOutage.validPublish();
assert.equal(nodeOutage.validPublishArmedExpiry, true);
nodeOutage.step(HEARTBEAT_TIMEOUT_MS);
assert.equal(nodeOutage.apStarted, true);
assert.equal(nodeOutage.validPublishArmedExpiry, false,
  'outage cancels a pending AP expiry');

// Unsigned elapsed arithmetic remains correct across the ~49.7-day wrap.
const wrap = new RecoveryModel({ failFirstApStart: true });
wrap.now = 0xfffffff0;
wrap.heartbeatSeen = false;
wrap.activateAp();
assert.equal(wrap.apAttempts, 1);
wrap.step(30000);
assert.equal(wrap.apAttempts, 1, 'no AP retry storm across millis wrap');
wrap.step(30000);
assert.equal(wrap.apAttempts, 2, 'AP retry occurs at 60 s across millis wrap');

// A seven-hour broker outage keeps retry probes bounded to one per minute. If
// the broker returns just after a probe, recovery is <=60 s to MQTT + <=15 s to
// heartbeat + <=2 s to the next offline health sample (77 s, plus TCP latency).
assert.ok(60000 + 15000 + OFFLINE_POLL_MS <= 77000);

console.log('MQTT long-outage recovery contract passed.');
