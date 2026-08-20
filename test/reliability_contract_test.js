"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");

const projectRoot = path.resolve(__dirname, "..");
const source = fs.readFileSync(path.join(projectRoot, "src", "main.cpp"), "utf8");
const apTestSource = fs.readFileSync(
  path.join(projectRoot, "src", "main_2.cpp"), "utf8");
const ini = fs.readFileSync(path.join(projectRoot, "platformio.ini"), "utf8");
const profile = fs.readFileSync(path.join(projectRoot, "include", "sensor_profile.h"), "utf8");
const identity = fs.readFileSync(path.join(projectRoot, "include", "build_identity.h"), "utf8");
const serialGenerator = fs.readFileSync(path.join(projectRoot, "scripts", "generate_serial.py"), "utf8");
const exampleProfile = JSON.parse(fs.readFileSync(
  path.join(projectRoot, "sensor_profiles", "witmotion_wtvb01.json"), "utf8"));
const bxProfile = JSON.parse(fs.readFileSync(
  path.join(projectRoot, "sensor_profiles", "bx2601a_wz01m.json"), "utf8"));
const bydrProfile = JSON.parse(fs.readFileSync(
  path.join(projectRoot, "sensor_profiles", "bydr3016zw.json"), "utf8"));
const bannerProfile = JSON.parse(fs.readFileSync(
  path.join(projectRoot, "sensor_profiles", "banner_qm30vt3.json"), "utf8"));
const schemaV3 = JSON.parse(fs.readFileSync(
  path.join(projectRoot, "sensor_profiles", "schema-v3.json"), "utf8"));

function rolloverElapsed(now, start) {
  return (now - start) >>> 0;
}

function unsignedConstant(name) {
  const match = source.match(new RegExp(
    `const unsigned long ${name}\\s*=\\s*(\\d+)`
  ));
  assert.ok(match, `Missing ${name}`);
  return Number(match[1]);
}

const recoveryStart = 0xffffff00;
assert.equal(rolloverElapsed(recoveryStart, recoveryStart) < 7000, true);
assert.equal(rolloverElapsed(100, recoveryStart) < 7000, true);
assert.equal(rolloverElapsed(6743, recoveryStart) < 7000, true);
assert.equal(rolloverElapsed(6744, recoveryStart) < 7000, false);

const wifiRetryInitial = unsignedConstant("WIFI_RETRY_INITIAL_MS");
const wifiRetryMaximum = unsignedConstant("WIFI_RETRY_MAX_MS");
const simulatedTimeoutAt =
  unsignedConstant("WIFI_CONNECT_ATTEMPT_TIMEOUT_MS");
const simulatedLastAttempt = simulatedTimeoutAt;
let simulatedWifiRetry = Math.min(
  wifiRetryInitial * 2, wifiRetryMaximum
);
assert.equal(
  rolloverElapsed(simulatedTimeoutAt, simulatedLastAttempt) >=
    simulatedWifiRetry,
  false,
  "A timed-out STA attempt must not restart in the same service iteration."
);
for (let i = 0; i < 20; i++) {
  simulatedWifiRetry = Math.min(simulatedWifiRetry * 2, wifiRetryMaximum);
}
assert.equal(simulatedWifiRetry, wifiRetryMaximum);

assert.match(source, /FIRMWARE_VERSION\s*=\s*"1\.8\.6-server-recovery5"/);
assert.match(source, /millis\(\) - modbusRecoveryStartedMs/);
assert.doesNotMatch(source, /return\s+millis\(\)\s*<\s*modbusRecovery/);
assert.doesNotMatch(source, /eng\.freqX\s*=\s*eng\.freqY\s*=\s*eng\.freqZ\s*=\s*0/);
assert.match(source, /monotonicMillis\(\)/);
assert.match(source, /"cfg_a"/);
assert.match(source, /"cfg_b"/);
assert.match(source, /"prof_a"/);
assert.match(source, /"prof_b"/);
assert.match(source, /configCrc32/);
assert.doesNotMatch(source, /prefs\.put(?:String|Int|UInt|ULong|UChar|UShort|Bool)\s*\(/);
assert.match(
  source,
  /loadPreferences\(\);[\s\S]*?WiFi\.persistent\(false\);[\s\S]*?WiFi\.onEvent\(onWifiEvent\);[\s\S]*?startSetupPortal\(\);/,
  "Preferences and WiFi event instrumentation must be initialized before the setup AP."
);
assert.doesNotMatch(source, /requestAuthentication|SETUP_AP_WINDOW_MS/);
assert.match(source, /IPAddress\s+setupApIp\(192,\s*168,\s*4,\s*1\)/);
assert.match(source, /const char \*SETUP_PIN\s*=\s*VIBSCOPE_BUILD_SUFFIX/);
assert.match(source, /ACTIVE_SENSOR_PROFILE\.frequencyFields/);
assert.match(source, /StaticJsonDocument<12288>\s+profileJsonDocument/);
assert.match(source, /deserializeJson\(profileJsonDocument,\s*json,\s*DeserializationOption::NestingLimit\(12\)\)/);
assert.match(source, /validateSensorProfile/);
assert.match(source, /persistSensorProfile/);
assert.match(source, /SENSOR_PROFILE_UPLOAD_MAX_BYTES\s*=\s*12288/);
assert.match(source, /sensor-profile\.json/);
assert.match(source, /handleProfileUploadStream/);
assert.match(source, /server\.on\("\/api\/sensor-profile",\s*HTTP_POST/);
assert.match(source, /server\.on\("\/sensor-profile\/",\s*HTTP_POST/);
assert.match(source, /server\.on\("\/sensor-profile",\s*HTTP_GET,\s*handleConfig\)/);
assert.match(source, /xhr\.open\('POST','\/api\/sensor-profile'\)/);
assert.match(source, /xhr\.upload\.onprogress/);
assert.match(source, /Connection lost during upload/);
assert.match(source, /Profile Not Applied/);
assert.match(source, /configForm'\)\.addEventListener\('submit',function\(e\)\{e\.preventDefault\(\)/);
assert.match(source, /body\.set\('response','json'\)/);
assert.match(source, /id='configState' class='upload-state'/);
assert.match(source, /Settings saved successfully/);
assert.match(source, /application\/json; charset=UTF-8/);
assert.match(source, /static ProfilePersistenceWorkspace profilePersistenceWorkspace/);
assert.match(source, /StoredSensorProfile &slotA = profilePersistenceWorkspace\.slotA/);
assert.match(source, /StoredSensorProfile &verify = profilePersistenceWorkspace\.verify/);
const persistProfileStart = source.indexOf(
  "bool persistSensorProfile(const ModbusSensorProfile &profile) {"
);
const persistProfileEnd = source.indexOf("\nvoid loadSensorProfile()", persistProfileStart);
assert.ok(persistProfileStart >= 0 && persistProfileEnd > persistProfileStart);
const persistProfileSource = source.slice(persistProfileStart, persistProfileEnd);
assert.doesNotMatch(
  persistProfileSource,
  /StoredSensorProfile\s+slotA\s*=\s*\{\}\s*,\s*slotB\s*=\s*\{\}/,
  "Profile persistence must not put two schema-3 storage records on the loop stack."
);
assert.match(source, /verifyUploadedProfile\(expectedKey\)/);
assert.match(source, /The upload response was interrupted, but the saved profile was verified/);
assert.match(source, /@font-face\{font-family:'JetBrains Mono'/);
assert.match(source, /server\.on\("\/assets\/jetbrains-mono\.woff2",\s*HTTP_GET,\s*handleJetBrainsMonoFont\)/);
assert.match(source, /server\.send_P\(200,\s*PSTR\("font\/woff2"\)/);
assert.match(source, /\.btn\{[^}]*font-family:'JetBrains Mono',monospace/);
assert.match(source, /body\{[^}]*font-family:'JetBrains Mono','Cascadia Mono','SFMono-Regular',Consolas,monospace/);
assert.match(source, /\.brand-word\{font-family:'JetBrains Mono',monospace;font-weight:700\}/);
assert.match(source, /<h1><span class='brand-word'>VibScope<\/span> Setup<\/h1>/);
assert.match(source, /profileStorageHealthy/);
assert.match(source, /sensorProfileLoadedFromStorage/);
assert.match(source, /const char \*mirrorKey/);
assert.match(source, /memcmp\(&slotA\.profile,\s*&slotB\.profile/);
assert.match(source, /profileFloatJson/);
assert.match(source, /decodeGenericField/);
assert.match(source, /profileDataTypeBytes/);
assert.match(source, /UInt32Mod10000/);
assert.match(source, /Float64/);
assert.match(source, /bitFieldSigned/);
assert.match(source, /hasInvalidRaw/);
assert.match(source, /hasValidRange/);
assert.match(source, /readModbusRegistersFunction/);
assert.match(source, /activeSensorSerialConfig/);
assert.match(source, /sensor_reported_magnitude/);
assert.match(source, /sensor_reported_unspecified/);
assert.match(source, /RMS alarm thresholds are disabled/);
assert.match(source, /minimumPollIntervalMs/);
assert.match(source, /name='poll_ms'/);
assert.match(source, /String newPollMs = server\.arg\("poll_ms"\)/);
assert.match(source, /requested < profilePollMinimum/);
assert.match(source, /requested > SENSOR_READ_MAX_INTERVAL_MS/);
assert.match(source, /sensorPollIntervalMs = parsedPollInterval/);
assert.match(source, /sensorPollIntervalMs = oldSensorPollIntervalMs/);
assert.match(source, /<b>Acquisition<\/b><span>Target/);
assert.match(source, /lastWifiAttemptMs\s*=\s*now;\s*wifiRetryDelayMs\s*=\s*min/);
assert.match(source, /now\s*-\s*lastWifiAttemptMs\s*>=\s*wifiRetryDelayMs/);
assert.doesNotMatch(source, /WIFI_RETRY_INTERVAL_MS|MQTT_RETRY_INTERVAL_MS/);
assert.match(source, /setCpuFrequencyMhz\(RUNTIME_CPU_FREQUENCY_MHZ\)/);
assert.match(source, /RUNTIME_CPU_FREQUENCY_MHZ\s*=\s*80/);
assert.match(source, /MQTT_PUBLISH_RATE_HZ\s*=\s*10\.0f/);
assert.match(source, /PUBLISH_FAST_MS\s*=\s*SENSOR_READ_MIN_INTERVAL_MS/);
assert.match(source, /WiFi\.setSleep\(WIFI_PS_MIN_MODEM\)/);
assert.match(source, /WiFi\.getSleep\(\)\s*==\s*WIFI_PS_MIN_MODEM/);
assert.match(source, /WiFi\.setTxPower\(WIFI_POWER_11dBm\)/);
assert.match(source, /SETUP_AP_HEALTHY_WINDOW_MS\s*=\s*10UL\s*\*\s*60UL\s*\*\s*1000UL/);
assert.match(source, /if \(sensorOk && nodeServerHeartbeatFresh\(lastPublishOkMs\)\)\s*\{\s*armSetupApLifetimeAfterHealthyPublish\(\);/);
assert.match(source, /server\.stop\(\)/);
assert.match(source, /WiFi\.softAPdisconnect\(true\)/);
assert.match(source, /WiFi\.mode\(setupApStarted \? WIFI_AP_STA : WIFI_STA\)/);
assert.match(source, /LOOP_IDLE_DELAY_MS\s*=\s*10/);
assert.match(source, /OFFLINE_SENSOR_INTERVAL_MS\s*=\s*2000/);
assert.match(source, /SENSOR_FAULT_INTERVAL_MS\s*=\s*2000/);
assert.match(source, /consecutiveModbusFailures\s*>\s*0\s*\|\|\s*modbusRecoveryActive\(\)/);
assert.match(source, /staSettingsApplyPending/);
assert.match(source, /beginStaConnection\("saved settings"\)/);
const setupStart = source.indexOf("void setup() {");
const setupEnd = source.indexOf("\nvoid checkSetupTriggers()", setupStart);
assert.ok(setupStart >= 0 && setupEnd > setupStart);
assert.doesNotMatch(
  source.slice(setupStart, setupEnd),
  /probeSensorConfig\(/,
  "Production boot must not sweep sensor baud/ID/register candidates."
);
assert.doesNotMatch(source, /StoredSensorProfileV1|StoredSensorProfileV2/);
assert.doesNotMatch(source, /LEGACY_PROFILE_STORAGE_NAMESPACE/);
assert.match(source, /Only sensor profile schemaVersion 3 is supported/);
assert.match(source, /PROFILE_SCHEMA_VERSION\s*=\s*3/);
assert.match(source, /sizeof\(StoredSensorProfile\)\s*<=\s*4096/);
assert.match(source, /profileObjectHasOnlyKeys/);
assert.match(source, /activeModbusInterFrameGapMs\(\)/);
assert.match(source, /requiredGapMs/);
assert.match(source, /uint64Json/);
assert.match(source, /9007199254740991/);
assert.match(source, /snprintf\(buffer,\s*sizeof\(buffer\),\s*"%.9g"/);
assert.match(source, /NestingLimit\(12\)/);
assert.match(source, /baselineCalibrationPending/);
assert.match(source, /latestChipTime\s*=\s*\{\}/);
assert.match(source, /sensorPowerCycleAttempted\s*=\s*false;\s*lastModbusSuccessMs\s*=\s*millis\(\);\s*lastSensorReadOkMs\s*=\s*0;/);
assert.match(source, /respTimeoutMs\s*>\s*15000UL/);
assert.match(source, /strcmp\(ACTIVE_SENSOR_PROFILE\.serialFormat,\s*"8E2"\)/);
assert.match(source, /bitsPerCharacter\s*=\s*12U/);
assert.match(source, /errno\s*=\s*0/);
assert.match(source, /errno\s*==\s*ERANGE/);
assert.match(source, /text\[0\]\s*==\s*'-'/);
assert.match(source, /Disabled vendorClock must have registerCount 0/);
assert.match(source, /Disabled bulkRead must have registerCount 0/);
assert.match(source, /exceeds the 12288-byte limit/);
assert.match(source, /Frequency must provide all X\/Y\/Z fields or explicitly disable all three/);
assert.match(source, /Displacement must provide all X\/Y\/Z fields or explicitly disable all three/);
assert.match(source, /eng\.frequencyValid && isValidSensorFrequency\(eng\.freqX\)/);
assert.match(source, /requireSetupApRequest\(\)/);
assert.match(source, /\\"networkIdentityHealthy\\":/);
assert.match(source, /cache:'no-store'/);
const profileActivation = source.indexOf("ACTIVE_SENSOR_PROFILE = candidate;");
const profileReconfigureQueued = source.indexOf(
  "sensorProfileReconfigurePending = true;", profileActivation
);
assert.ok(
  profileActivation >= 0 &&
  profileReconfigureQueued > profileActivation,
  "A saved profile must queue sensor reconfiguration after activation."
);
for (const handler of [
  "handleConfig", "handleSave", "handleProfileDownload",
  "handleStatus", "handleNotFound"
]) {
  assert.match(source, new RegExp(
    `void ${handler}\\(\\)\\s*\\{\\s*if \\(!requireSetupApRequest\\(\\)\\) return;`
  ));
}
assert.match(source,
  /void handleProfileUploadComplete\(\)\s*\{\s*if \(!requireSetupApRequest\(\)\)\s*\{/);
const persistGuard = source.indexOf("if (!persistSensorProfile(candidate))");
const activateUploaded = source.indexOf("ACTIVE_SENSOR_PROFILE = candidate", persistGuard);
assert.ok(persistGuard >= 0 && activateUploaded > persistGuard,
  "An uploaded profile must be verified in storage before activation.");
assert.match(source, /loadSensorProfile\(\);\s*PersistentConfig/);
assert.match(source, /ACTIVE_SENSOR_PROFILE\s*=\s*DEFAULT_SENSOR_PROFILE/);
assert.match(source, /WiFi\.setHostname\(networkHostname\.c_str\(\)\)/);
assert.match(source, /WiFi\.softAPsetHostname\(networkHostname\.c_str\(\)\)/);
assert.doesNotMatch(source, /\\"board\\":/);
assert.doesNotMatch(source, /macAddress/);
const customerVisibleLiterals = source.match(/"(?:\\.|[^"\\])*"/g) || [];
assert.deepEqual(
  customerVisibleLiterals.filter((literal) => /esp32/i.test(literal)),
  [],
  "Customer-visible firmware string literals must not identify the controller platform."
);
assert.doesNotMatch(source, /readModbusRegisters\(0x003A/);
assert.doesNotMatch(source, /readModbusRegisters\(0x0040/);
assert.doesNotMatch(source, /readModbusRegisters\(0x0044/);
assert.match(source, /requiredPacketBytes\s*<\s*MQTT_PACKET_BUFFER_SIZE/);
assert.match(ini, /platform\s*=\s*espressif32@7\.0\.1/);
assert.match(ini, /board_build\.f_cpu\s*=\s*80000000L/);
assert.match(apTestSource, /pinMode\(1,\s*INPUT\)/);
assert.match(apTestSource, /pinMode\(3,\s*INPUT\)/);
assert.match(apTestSource, /delay\(10\);\s*\}/);
assert.match(ini, /PubSubClient\s*@\s*2\.8\.0/);
assert.match(ini, /ArduinoJson\s*@\s*6\.21\.5/);
assert.match(ini, /pre:scripts\/generate_serial\.py/);
assert.match(ini, /\[env:esp32dev-custom-profile\]/);
assert.match(identity, /VIBSCOPE_BUILD_SERIAL "VS-2607-12-\d{12}"/);
assert.match(serialGenerator, /secrets\.randbelow\(1_000_000_000_000\):012d/);
assert.match(source, /String\s+newIdentificationName\s*=\s*server\.arg\("identification_name"\)/);
assert.match(source, /payload \+= "\\"identificationName\\":\\""/);
assert.match(serialGenerator, /upload_requested = "upload" in COMMAND_LINE_TARGETS/);
assert.match(serialGenerator, /if upload_requested or not header_path\.exists\(\):\s*allocate_serial\(\)/);
assert.match(serialGenerator, /VS-2607-12-\{suffix\}/);
assert.match(profile, /struct ModbusSensorProfile/);
assert.match(profile, /struct ModbusFieldProfile/);
assert.match(profile, /enum class ModbusProfileDataType/);
assert.match(profile, /SENSOR_PROFILE_CUSTOM/);
assert.equal(exampleProfile.schemaVersion, 3);
assert.equal(exampleProfile.modbus.slaveId, 80);
assert.equal(exampleProfile.modbus.addressConvention, "pdu_zero_based");
assert.equal(exampleProfile.measurements.frequency.x.address, 68);
assert.equal(exampleProfile.measurements.frequency.x.scale, 0.1);
assert.equal(exampleProfile.measurements.frequency.x.dataType, "uint16");
assert.equal(exampleProfile.measurements.acceleration.x.dataType, "int16");
assert.equal(exampleProfile.frequencyRangeHz.maximum, 6553.5);
assert.equal(bxProfile.schemaVersion, 3);
assert.ok(bxProfile.registerMapVersion.length <= 31);
assert.equal(bxProfile.modbus.slaveId, 1);
assert.deepEqual(
  [
    bxProfile.measurements.acceleration.x.address,
    bxProfile.measurements.acceleration.y.address,
    bxProfile.measurements.acceleration.z.address
  ],
  [19, 14, 9]
);
assert.deepEqual(
  [
    bxProfile.measurements.frequency.x.address,
    bxProfile.measurements.frequency.y.address,
    bxProfile.measurements.frequency.z.address
  ],
  [21, 16, 11]
);
assert.equal(bxProfile.measurements.acceleration.x.scale, 0.01 * 9.80665);
assert.equal(bxProfile.measurements.acceleration.x.dataType, "uint16");
assert.equal(bxProfile.measurements.temperature.value.dataType, "int16");
assert.equal(bxProfile.frequencyRangeHz.maximum, 5000);
assert.equal(bxProfile.acquisition.minimumPollIntervalMs, 350);
assert.equal(bxProfile.measurements.acceleration.semantics, "sensor_reported_magnitude");
assert.equal(bxProfile.measurements.velocity.semantics, "sensor_reported_unspecified");
assert.deepEqual(
  [bxProfile.bulkRead.startAddress, bxProfile.bulkRead.registerCount],
  [0, 0]
);
assert.equal(bxProfile.bulkRead.enabled, false);
assert.equal(exampleProfile.modbus.maximumReadRegisters, 19);
assert.equal(bxProfile.modbus.maximumReadRegisters, 4);
assert.equal(bydrProfile.schemaVersion, 3);
assert.equal(bydrProfile.modbus.slaveId, 1);
assert.equal(bydrProfile.measurements.velocity.semantics, "sensor_reported_rms");
assert.equal(bydrProfile.measurements.acceleration.x.scale, 0.01);
assert.equal(bydrProfile.measurements.temperature.value.scale, 0.01);
assert.equal(bydrProfile.measurements.displacement.x.scale, 1);
assert.deepEqual(
  ["x", "y", "z"].map(axis =>
    bydrProfile.measurements.frequency[axis].enabled),
  [false, false, false]
);
assert.deepEqual(
  [bydrProfile.frequencyRangeHz.minimum, bydrProfile.frequencyRangeHz.maximum],
  [0, 0]
);
assert.equal(bydrProfile.bulkRead.enabled, false);
assert.equal(bannerProfile.schemaVersion, 3);
assert.equal(bannerProfile.modbus.baud, 19200);
assert.equal(bannerProfile.measurements.velocity.semantics, "sensor_reported_rms");
assert.equal(bannerProfile.measurements.acceleration.x.address, 33);
assert.equal(bannerProfile.measurements.velocity.z.address, 40);
assert.equal(bannerProfile.measurements.frequency.x.address, 25);
assert.equal(bannerProfile.measurements.temperature.value.address, 42);
assert.equal(bannerProfile.measurements.displacement.x.enabled, false);
assert.equal(bannerProfile.bulkRead.enabled, false);
assert.equal(schemaV3.properties.schemaVersion.const, 3);
assert.equal(schemaV3.properties.modbus.properties.maximumReadRegisters.maximum, 125);
assert.equal(schemaV3.$defs.field.properties.registerCount.maximum, 4);
const exampleScaleFloat32 = Math.fround(exampleProfile.measurements.acceleration.x.scale);
assert.equal(
  Math.fround(Number(exampleScaleFloat32.toPrecision(9))),
  exampleScaleFloat32,
  "Nine significant digits must round-trip the stored float32 sensor scale."
);

function generationIsNewer(a, b) {
  return ((a - b) | 0) > 0;
}
assert.equal(generationIsNewer(12, 11), true);
assert.equal(generationIsNewer(0, 0xffffffff), true);
assert.equal(generationIsNewer(11, 12), false);

console.log("Reliability contract tests passed.");
