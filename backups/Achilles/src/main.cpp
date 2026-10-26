// =============================================================================
// IVSS ESP32 Firmware
//
// Owner:           Akiharu Scientific and Engineering
// Copyright:       (c) 2026 Akiharu Scientific and Engineering. All rights reserved.
// Firmware creator: M. R. Omar
//
// Maintenance markers used throughout this file:
//   [WIFI AP]        SoftAP, AP+STA coexistence, setup portal, or WiFi scanning.
//   [SENSOR PROFILE] Profile validation, persistence, register maps, or bus setup.
//   [CALCULATIONS]   Engineering values, RMS, confidence, alarms, or conversions.
//   [MQTT]           Broker lifecycle, packet construction, or publication.
//   [HIGH RISK]      A change can disrupt boot, AP visibility, sensor acquisition,
//                    persisted compatibility, telemetry meaning, or field operation.
//
// Documentation rule: category and risk comments describe the existing behaviour.
// They are not permission to change timing, call order, constants, or control flow.
//
// Boot flow:
//   1. Load WiFi, MQTT, and internal sensor-bus settings.
//   2. Apply the immutable serial embedded by the upload build.
//   3. Start the WPA2-protected setup AP for commissioning.
//   4. If saved WiFi exists, begin a non-blocking STA connection attempt.
//   5. Publish telemetry over STA/MQTT whenever those links are available.
//
// Setup AP:
//   - SSID:     TR-2607-12-############ by default; user-editable.
//   - Password: the 12-digit suffix by default; user-editable.
//   - URL:      http://192.168.4.1  or  http://sensor.local
//   - The setup network runs its own captive-portal DNS (all names resolve to
//     the AP) so phones/laptops auto-open the setup page on connect, the same
//     way a hotel or airport WiFi portal does.
//
// Runtime:
//   - Setup web server, STA reconnects, MQTT reconnects, and acquisition all run
//     from loop() without blocking each other.
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "esp_wpa2.h"
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <cerrno>
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "build_identity.h"
#include "sensor_profile.h"

extern const uint8_t jetbrainsMonoRegularStart[]
  asm("_binary_assets_jetbrains_mono_regular_woff2_start");
extern const uint8_t jetbrainsMonoRegularEnd[]
  asm("_binary_assets_jetbrains_mono_regular_woff2_end");

// [SENSOR PROFILE] The built-in audited profile is copied into RAM at boot. A
// validated profile stored through the setup AP may replace it before the
// sensor UART starts.
ModbusSensorProfile ACTIVE_SENSOR_PROFILE = DEFAULT_SENSOR_PROFILE;

// ---------------------------------------------------------------------------
// Hardware [HIGH RISK]
// ---------------------------------------------------------------------------
// GPIO and UART assignments are board- and wiring-dependent. Changing these
// can prevent boot, interfere with GPIO0 strapping, or disconnect the RS485 bus.
#define LED_PIN 2   // Active-LOW on most ESP32-WROOM dev boards
#ifndef SENSOR_UART_TX_PIN
#define SENSOR_UART_TX_PIN 17
#endif
#ifndef SENSOR_UART_RX_PIN
#define SENSOR_UART_RX_PIN 16
#endif
// This isolated converter labels its MCU-side UART pins by the MCU signal:
// ESP32 TX connects to module TXD (internally the RS485 driver's DI), and
// ESP32 RX connects to module RXD (internally the RS485 receiver's RO).
const int UART2_TX_PIN = SENSOR_UART_TX_PIN;  // ESP32-U TX header / GPIO1 -> module TXD
const int UART2_RX_PIN = SENSOR_UART_RX_PIN;  // ESP32-U RX header / GPIO3 <- module RXD
static_assert(SENSOR_UART_TX_PIN != SENSOR_UART_RX_PIN, "Sensor UART TX and RX must use different GPIOs");
#if defined(SENSOR_ON_PROGRAM_UART_PINS)
static_assert(SENSOR_UART_TX_PIN == 1 && SENSOR_UART_RX_PIN == 3,
              "Controller TX/RX build must map TX=GPIO1 and RX=GPIO3");
#endif
#ifndef SENSOR_POWER_ENABLE_PIN
#define SENSOR_POWER_ENABLE_PIN -1
#endif
const int SENSOR_POWER_PIN       = SENSOR_POWER_ENABLE_PIN;
int       RS485_DE_RE_PIN        = -1;    // -1 means converter with auto-direction (no DE/RE control)
bool      RS485_DIR_ACTIVE_HIGH  = true;  // true: HIGH=TX, false: LOW=TX
const int SETUP_BUTTON_PIN       = 0;     // BOOT button (GPIO0): hold to enter WiFi setup
const unsigned long SETUP_BUTTON_HOLD_MS = 2000;  // hold this long to trigger setup AP

// [SENSOR PROFILE] Profile-driven Modbus RTU defaults [HIGH RISK]
// Adjust only against a verified sensor datasheet/profile and physical bus.
// PREFERRED_SENSOR_BAUD: the baud the sensor should already be configured to.
// Changing this does NOT change the sensor itself — it only sets what the ESP32 tries first.
// Merely changing the ESP32 UART does not reconfigure the sensor; the active
// profile supplies the expected factory communication settings.
const uint32_t PREFERRED_SENSOR_BAUD          = DEFAULT_SENSOR_PROFILE.defaultBaud;
const uint32_t SENSOR_BAUD_DEFAULT            = PREFERRED_SENSOR_BAUD;
const uint8_t  SENSOR_MODBUS_ID_DEFAULT       = DEFAULT_SENSOR_PROFILE.defaultSlaveId;
const uint16_t SENSOR_REG_ACCEL_BASE_DEFAULT = DEFAULT_SENSOR_PROFILE.acceleration.startRegister;
const uint8_t  SENSOR_READ_FUNC_DEFAULT       = DEFAULT_SENSOR_PROFILE.readFunction;
const uint16_t SENSOR_BASELINE_SAMPLES        = 50;
const uint32_t MODBUS_RESPONSE_TIMEOUT_MS     = 150;    // Sensor processing/turnaround allowance beyond wire time
const unsigned long RMS_WINDOW_MS             = 250;    // Rolling history window length for waveform context.
const bool          SENSOR_DIAGNOSTIC_MODE    = true;   // true: displayRms = true RMS; no deadband/noise-floor hiding
const float         RMS_DEADBAND_MS2           = 0.005f; // used only when SENSOR_DIAGNOSTIC_MODE is false
const float         NOISE_FLOOR_MULTIPLIER    = 0.25f;  // used only when SENSOR_DIAGNOSTIC_MODE is false
const uint8_t       RMS_MIN_RELIABLE_SAMPLES = 4;
// This is the ESP32 Modbus polling target, not the sensor's internal detection
// cycle or the physical vibration frequency reported by the active profile.
// [CALCULATIONS] Acquisition cadence and alarm thresholds [HIGH RISK]. These
// constants affect telemetry meaning, confidence, and field alarm decisions.
const float         TARGET_SAMPLE_RATE_HZ     = 10.0f;
const uint32_t      TARGET_SAMPLE_PERIOD_US = (uint32_t)(1000000.0f / TARGET_SAMPLE_RATE_HZ);
const unsigned long TARGET_SAMPLE_PERIOD_MS = (unsigned long)(1000.0f / TARGET_SAMPLE_RATE_HZ);
// Keep sensor acquisition capable of 10 Hz, but cap MQTT document construction
// and radio transmission at 10 Hz while the link is healthy. The adaptive
// cadence below can still back off under weak or lossy conditions.
const float         MQTT_PUBLISH_RATE_HZ      = 10.0f;
const unsigned long MQTT_PUBLISH_PERIOD_MS    = (unsigned long)(1000.0f / MQTT_PUBLISH_RATE_HZ);
const unsigned long SENSOR_READ_MIN_INTERVAL_MS = 100; // Default: do not poll computed registers faster than 10 Hz.
const unsigned long SENSOR_READ_TEST_INTERVAL_MS = 500; // Serial command "poll 500" gives a conservative overnight test rate.
const unsigned long SENSOR_READ_MAX_INTERVAL_MS = 60000;
const float         VELOCITY_ALERT_MM_S   = 2.8f;   // conservative defaults; tune per machine
#define SENSOR_VELOCITY_LSB_MM_S ((float)fabs(ACTIVE_SENSOR_PROFILE.velocityFields[0].scale))
#define SENSOR_DISPLACEMENT_LSB_UM ((float)fabs(ACTIVE_SENSOR_PROFILE.displacementFields[0].scale))
const float         VELOCITY_ALARM_MM_S   = 4.5f;
const float         VELOCITY_CRITICAL_MM_S = 7.1f;
#define SENSOR_FREQUENCY_SCALE ((float)fabs(ACTIVE_SENSOR_PROFILE.frequencyFields[0].scale))
// Profile-defined frequency fields contain sensor-reported values. Do not
// clamp them to the acquisition/output rate; the active profile defines the
// valid engineering range and the firmware passes through the full report.
#define SENSOR_FREQUENCY_MIN_HZ (ACTIVE_SENSOR_PROFILE.frequencyMinimum)
#define SENSOR_FREQUENCY_MAX_HZ (ACTIVE_SENSOR_PROFILE.frequencyMaximum)
const char         *FIRMWARE_NAME              = "IVSS Controller";
const char         *FIRMWARE_VERSION           = "Achilles";
const char         *BUILD_SERIAL_NUMBER        = IVSS_BUILD_SERIAL;
const char         *SENSOR_MODEL_NAME          = ACTIVE_SENSOR_PROFILE.modelName;
const char         *DEVICE_PROTOCOL_NAME       = "RS485-Modbus";
const char         *SENSOR_REGISTER_MAP_VERSION = ACTIVE_SENSOR_PROFILE.registerMapVersion;
const float         BASELINE_FALLBACK_X         = 0.0f;
const float         BASELINE_FALLBACK_Y         = 0.0f;
const float         BASELINE_FALLBACK_Z         = 9.80665f;
const uint32_t     *KNOWN_SENSOR_BAUDS          = SENSOR_PROFILE_KNOWN_BAUDS;
const size_t KNOWN_SENSOR_BAUD_COUNT =
  sizeof(SENSOR_PROFILE_KNOWN_BAUDS) / sizeof(SENSOR_PROFILE_KNOWN_BAUDS[0]);

// ---------------------------------------------------------------------------
// Timing (ms)
// ---------------------------------------------------------------------------
const unsigned long WIFI_CONNECT_ATTEMPT_TIMEOUT_MS = 10000;
// WPA2-Enterprise adds a full PEAP/TLS exchange with a RADIUS server before
// DHCP even starts. On real corporate networks that regularly exceeds the 10 s
// budget that is ample for WPA2-PSK, so the attempt was being abandoned while
// it was still legitimately progressing.
const unsigned long WIFI_ENTERPRISE_CONNECT_ATTEMPT_TIMEOUT_MS = 25000;
// Sensor acquisition can run at 10 Hz. Full MQTT document construction and
// transmission is independently capped by MQTT_PUBLISH_PERIOD_MS.
const unsigned long PUBLISH_FAST_MS             = SENSOR_READ_MIN_INTERVAL_MS; // 10 Hz (strong WiFi)
const unsigned long PUBLISH_MED_MS              = 200;   // 5 Hz when the link is marginal
const unsigned long PUBLISH_SLOW_MS             = 500;   // 2 Hz when the link is weak/lossy
const unsigned long CADENCE_EVAL_INTERVAL_MS    = 1000;  // react to a bad link within 1 s
const unsigned long WIFI_RETRY_INITIAL_MS       = 15000;
// A 5-minute ceiling meant that after a few early failures (a router still
// booting, a technician mistyping once) a correctly configured sensor would sit
// silent for five minutes per attempt, which reads in the field as "it never
// connects". 60 s still keeps association attempts and active scanning rare
// enough to protect SoftAP responsiveness and sensor acquisition.
const unsigned long WIFI_RETRY_MAX_MS           = 60000;
const unsigned long MQTT_RETRY_INITIAL_MS       = 10000;
// A sleeping/restarting factory computer must be rediscovered promptly after it
// returns. A five-minute ceiling made a healthy recovery look like a permanent
// failure. One bounded TCP probe per minute is still a very small energy cost.
const unsigned long MQTT_RETRY_MAX_MS           = 60000;
const unsigned long MQTT_DOWN_LOG_INTERVAL_MS   = 20000;
// PubSubClient can briefly retain a logically-connected TCP socket after the
// broker host disappears. If no telemetry write succeeds for this long, rebuild
// the MQTT session instead of trusting stale socket state indefinitely.
const unsigned long MQTT_SESSION_PROGRESS_TIMEOUT_MS = 90000;
// The local Node.js service publishes a lightweight heartbeat every 15 s. This
// is the end-to-end proof that distinguishes "broker accepts packets" from
// "the dashboard service is actually alive and subscribed". Three missed
// heartbeats open the recovery AP without repeatedly scanning or reconnecting.
const unsigned long NODE_SERVER_HEARTBEAT_TIMEOUT_MS = 45000;
const unsigned long NODE_SERVER_HEARTBEAT_RESUBSCRIBE_MS = 60000;
const unsigned long RECOVERY_AP_RETRY_INTERVAL_MS    = 60000;
const unsigned long MQTT_OUTAGE_AP_TRIGGER_MS        = 15000;
const char         *NODE_SERVER_HEARTBEAT_TOPIC      = "ivss/server/heartbeat";
const unsigned long LED_SLOW_BLINK_MS           = 1000;  // AP active + STA disconnected
const unsigned long LED_MQTT_BLINK_MS           = 90;    // MQTT connecting: faster blink
const unsigned long LED_DOUBLE_BLINK_PERIOD_MS  = 1200;
const unsigned long LED_CONNECTED_INTERVAL_MS   = 10000; // pulse every 10 s
const unsigned long LED_CONNECTED_PULSE_MS      = 120;   // pulse width
const unsigned long SENSOR_LOG_INTERVAL_MS      = 1000;  // avoid Serial blocking the MQTT/sensor loop
const unsigned long FAST_FRAME_RETRY_INTERVAL_MS = 30000; // retry optimized bulk read after split-read fallback
const uint8_t       FAST_FRAME_FAILURES_BEFORE_FALLBACK = 3;
const unsigned long MODBUS_MIN_INTER_REQUEST_GAP_MS = 5;  // Modbus RTU quiet time between complete transactions
const unsigned long MODBUS_FAILURE_BACKOFF_MS    = 250;   // bounded pause after a failed request before retrying
const unsigned long MODBUS_POST_FAIL_DRAIN_MS    = 30;    // absorb a late response before the next transaction
const uint8_t       MODBUS_FAILURES_BEFORE_RECOVERY = 10;
const unsigned long MODBUS_RECOVERY_PAUSE_MS     = 7000;  // stop polling, restart UART, then retry
// Uploaded profiles without a working bulk read can require several Modbus
// transactions for one sample. Limit that transaction budget so a profile
// cannot monopolize the UART/CPU and starve WiFi/web servicing.
const uint8_t       MODBUS_SPLIT_MAX_REQUESTS_PER_SECOND = 20;
const unsigned long SENSOR_POWER_CYCLE_OFF_MS    = 1000;
const unsigned long SENSOR_POWER_CYCLE_AFTER_MS  = 60000;
const unsigned long SENSOR_BOOT_SETTLE_MS        = 800;   // allow sensor power-up before the first request
const unsigned long BASELINE_RETRY_INTERVAL_MS   = 60000; // failed calibration must not become a continuous Modbus loop
const unsigned long AP_CLIENT_SENSOR_INTERVAL_MS = 500;   // prioritize setup HTTP while a technician is connected
const unsigned long OFFLINE_SENSOR_INTERVAL_MS   = 2000;  // no telemetry path: retain health sampling without needless bus/CPU load
const unsigned long SENSOR_FAULT_INTERVAL_MS     = 2000;  // failed bus: prevent 10 Hz invalid-payload and retry churn
const unsigned long SETUP_AP_HEALTHY_WINDOW_MS   = 10UL * 60UL * 1000UL; // close AP 10 min after first valid MQTT publish
const bool          VERBOSE_SERIAL              = false; // true only for deep debug. Keeps serial quiet + loop smooth.
const unsigned long HEARTBEAT_INTERVAL_MS       = 5000;  // serial heartbeat: uptime/heap/status
const uint32_t      WDT_TIMEOUT_S               = 8;     // hardware watchdog: reset if loop() hangs > 8 s
const uint32_t      MIN_SAFE_FREE_HEAP_BYTES     = 24000;
const uint32_t      MIN_SAFE_LARGEST_BLOCK_BYTES = 8192;
const unsigned long LOOP_IDLE_DELAY_MS           = 10;    // 100 Hz service ceiling; still far faster than HTTP/MQTT/sensor deadlines
const uint32_t      RUNTIME_CPU_FREQUENCY_MHZ    = 80;    // lower CPU power; AP/STA radio policy remains unchanged

// ---------------------------------------------------------------------------
// [WIFI AP] Fixed setup credentials [HIGH RISK]
// ---------------------------------------------------------------------------
// The 12-digit serial suffix is the factory WPA2 setup credential.
// Changing this affects field access to every provisioned controller.
const char *SETUP_PIN    = IVSS_BUILD_SUFFIX;

// ---------------------------------------------------------------------------
// [MQTT] Packet and raw-window constants [HIGH RISK]
// ---------------------------------------------------------------------------
// The packet buffer must remain larger than the complete telemetry topic and
// payload. RAW_WINDOW_SIZE also affects RAM consumption and calculation input.
const int MQTT_PACKET_BUFFER_SIZE   = 4096;
const int RAW_WINDOW_SIZE           = 96; // rolling waveform samples for backend FFT

// ---------------------------------------------------------------------------
// Runtime configuration loaded from Preferences
// ---------------------------------------------------------------------------
Preferences prefs;
String      deviceId;
String      identificationName;
String      wifiSsid;
String      wifiPass;
// [WIFI AP] Optional. Blank = ordinary WPA2-Personal, using wifiPass as the
// PSK exactly as before. Non-blank = WPA2-Enterprise (802.1X, PEAP/MSCHAPv2),
// using wifiPass as the Enterprise account password instead of a PSK.
String      wifiEnterpriseUser;
bool        wifiEnterpriseActive   = false; // tracks which mode is currently applied to the STA driver
String      mqttBroker;
String      mqttUser;
String      mqttPass;
int         mqttPort           = 1883;
String      setupApSsid;
String      setupApPass;
uint32_t    sensorBaud         = SENSOR_BAUD_DEFAULT;
uint8_t     sensorModbusId     = SENSOR_MODBUS_ID_DEFAULT;
uint16_t    sensorAccelRegBase = SENSOR_REG_ACCEL_BASE_DEFAULT;
uint8_t     sensorReadFunction = SENSOR_READ_FUNC_DEFAULT;

// ---------------------------------------------------------------------------
// [WIFI AP] Time-limited setup web server through the WPA2-protected AP
// ---------------------------------------------------------------------------
WebServer    server(80);
DNSServer   dnsServer;
bool        setupApDnsStarted       = false;
bool        setupApMdnsStarted      = false;
IPAddress   setupApIp(192, 168, 4, 1);
bool        setupApStarted          = false;
bool        webServerStarted        = false;
bool        webServerRoutesConfigured = false;
bool        setupApRestartPending      = false;
unsigned long setupApRestartAfterMs    = 0;
bool        setupApExpiryArmed      = false;
unsigned long setupApHealthyPublishMs = 0;
bool        recoverySetupApLatched  = false;
bool        recoverySetupApAttempted = false;
unsigned long lastRecoverySetupApAttemptMs = 0;
bool        networkIdentityHealthy  = false;
bool        staConnectInProgress    = false;
bool        staSettingsApplyPending = false;
// [WIFI AP] millis() timestamp of when the currently-associated setup-AP
// client(s) were last (re)confirmed to justify pausing STA. Reset to 0 when
// no client is associated. See setupApBlockingStaAssociation() below.
unsigned long setupApClientJoinedMs    = 0;
wl_status_t lastStaStatus           = WL_IDLE_STATUS;
unsigned long staConnectStartedMs   = 0;
// [WIFI AP] STA association diagnostics. The radio reports a numeric
// 802.11/EAP reason for every failed association. Without capturing it, a wrong
// password, an out-of-range AP, and a rejected 802.1X account are all
// indistinguishable from a generic "connect attempt timed out", which makes
// field commissioning failures unresolvable from the serial log or the portal.
uint8_t     lastStaDisconnectReason = 0;
uint32_t    staDisconnectCount      = 0;
bool        staEverGotIp            = false;
bool        lastMqttConnected       = false;

// ---------------------------------------------------------------------------
// [MQTT] Client transport (uses STA when connected)
// ---------------------------------------------------------------------------
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ---------------------------------------------------------------------------
// Chip time (on-chip RTC, read once per publish)
// ---------------------------------------------------------------------------
struct SensorClockTime {
  uint16_t year;   // raw register — if < 100 add 2000 for full year (e.g. 24 → 2024)
  uint8_t  month;  // 1–12
  uint8_t  day;    // 1–31
  uint8_t  hour;   // 0–23
  uint8_t  minute; // 0–59
  uint8_t  sec;    // 0–59
  uint16_t ms;     // 0–999
  bool     valid;
};

struct SensorEngineeringMetrics {
  bool  velocityValid;
  bool  displacementValid;
  bool  frequencyValid;
  bool  temperatureValid;
  uint64_t rawVelocityX;
  uint64_t rawVelocityY;
  uint64_t rawVelocityZ;
  uint64_t rawDisplacementX;
  uint64_t rawDisplacementY;
  uint64_t rawDisplacementZ;
  float velocityRmsX;       // mm/s, sensor-provided vibration speed
  float velocityRmsY;
  float velocityRmsZ;
  float displacementX;      // um, sensor-provided vibration displacement
  float displacementY;
  float displacementZ;
  uint64_t rawFreqX;
  uint64_t rawFreqY;
  uint64_t rawFreqZ;
  float freqX;              // Hz, profile-defined X frequency field
  float freqY;              // Hz, profile-defined Y frequency field
  float freqZ;              // Hz, profile-defined Z frequency field
  float dominantFreqX;      // Hz, backward-compatible alias for freqX
  float dominantFreqY;
  float dominantFreqZ;
  bool  frequencySuspicious;
  float temperatureC;       // deg C
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
unsigned long lastPublishMs       = 0;
unsigned long lastWifiAttemptMs   = 0;
unsigned long lastMqttAttemptMs   = 0;
unsigned long wifiRetryDelayMs     = WIFI_RETRY_INITIAL_MS;
unsigned long mqttRetryDelayMs     = MQTT_RETRY_INITIAL_MS;
unsigned long mqttDisconnectedSinceMs = 0;
unsigned long mqttSessionProgressMs = 0;
unsigned long mqttConnectedSinceMs = 0;
unsigned long lastNodeServerHeartbeatMs = 0;
unsigned long lastNodeHeartbeatSubscribeAttemptMs = 0;
bool          nodeServerHeartbeatSeen = false;
bool          nodeServerHeartbeatTimedOut = false;
unsigned long lastLedToggleMs     = 0;
unsigned long ledPulseEndMs       = 0;
unsigned long nextConnectedPulse  = 0;
bool          ledState            = false;
uint8_t       sensorReadFailCount = 0;
uint8_t       consecutiveFastFrameFailures = 0;
bool          useSplitModbusReads = !ACTIVE_SENSOR_PROFILE.supportsContiguousFastFrame;
unsigned long lastFastFrameAttemptMs = 0;
uint32_t      modbusFailCount     = 0;
uint32_t      modbusTimeoutCount  = 0;
uint32_t      modbusCrcFailCount  = 0;
uint32_t      modbusExceptionCount = 0;
uint32_t      modbusBadFrameCount = 0;
uint32_t      modbusEchoCount     = 0;
uint32_t      modbusNoiseByteCount = 0;
uint32_t      modbusSuccessCount  = 0;
uint8_t       consecutiveModbusFailures = 0;
unsigned long lastModbusFailureMs = 0;
unsigned long lastModbusSuccessMs = 0;
unsigned long modbusRecoveryStartedMs = 0;
bool          modbusRecoveryPending = false;
bool          sensorPowerCycleAttempted = false;
unsigned long lastModbusRequestEndMs = 0;
unsigned long lastSensorPollStartMs = 0;
unsigned long lastActualSensorPollIntervalMs = 0;
char          lastModbusErrorReason[96] = "none";
float         baselineX           = BASELINE_FALLBACK_X;
float         baselineY           = BASELINE_FALLBACK_Y;
float         baselineZ           = BASELINE_FALLBACK_Z;
float         noiseFloorX         = 0.0f;
float         noiseFloorY         = 0.0f;
float         noiseFloorZ         = 0.0f;
bool          baselineCalibrated  = false;
bool          baselineCalibrationPending = false;
unsigned long lastBaselineCalibrationAttemptMs = 0;
float         lastRawX            = 0.0f;
float         lastRawY            = 0.0f;
float         lastRawZ            = 0.0f;
bool          hasLastRaw          = false;
float         rawWaveX[RAW_WINDOW_SIZE] = {};
float         rawWaveY[RAW_WINDOW_SIZE] = {};
float         rawWaveZ[RAW_WINDOW_SIZE] = {};
float         dynWaveX[RAW_WINDOW_SIZE] = {};
float         dynWaveY[RAW_WINDOW_SIZE] = {};
float         dynWaveZ[RAW_WINDOW_SIZE] = {};
uint16_t      waveWriteIndex      = 0;
uint16_t      waveSamplesStored   = 0;
SensorClockTime latestChipTime    = {};      // cached sensor clock, updated periodically
unsigned long lastChipTimeReadMs  = 0;
const unsigned long CHIP_TIME_READ_INTERVAL_MS = 60000;
unsigned long lastSensorLogMs      = 0;
unsigned long lastAcquisitionStartMs = 0;
// --- reliability / heartbeat counters ---
uint32_t      publishSuccessCount  = 0;
uint32_t      publishFailCount      = 0;
unsigned long lastHeartbeatMs       = 0;
unsigned long lastSensorReadOkMs    = 0;
unsigned long lastPublishOkMs       = 0;
unsigned long lastTelemetryAttemptMs = 0;
// Tracks the end-to-end consumer path, not merely WiFi/MQTT transport. Its
// rising edge re-primes waveform RMS after any slow outage-health sampling.
bool          telemetryPathWasReady  = false;
uint32_t      minFreeHeap           = 0xFFFFFFFFUL;
uint32_t      minLargestFreeBlock   = 0xFFFFFFFFUL;
bool          watchdogReady         = false;
bool          mqttBufferReady       = false;
bool          configStorageHealthy  = true;
String        lastConfigStorageError = "none";
bool          profileStorageHealthy = true;
String        lastProfileStorageError = "none";
bool          sensorProfileLoadedFromStorage = false;
bool          telemetryMemoryHealthy = true;
uint8_t       lowHeapHeartbeatCount = 0;
unsigned long setupBtnDownMs        = 0;       // BOOT-button hold timer
unsigned long currentPublishIntervalMs = PUBLISH_MED_MS; // adaptive; starts conservatively at 5 Hz
unsigned long sensorPollIntervalMs = SENSOR_READ_MIN_INTERVAL_MS;
unsigned long lastCadenceEvalMs     = 0;
uint32_t      recentPublishAttempts = 0;
uint32_t      recentPublishFails    = 0;
String        serialCmd             = "";      // serial command line buffer
String        profileUploadJson;
String        profileUploadError;
StaticJsonDocument<12288> profileJsonDocument;
bool          profileUploadReady    = false;
bool          sensorProfileReconfigurePending = false;
unsigned long sensorProfileReconfigureAfterMs = 0;
const size_t  SENSOR_PROFILE_UPLOAD_MAX_BYTES = 12288;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void loadPreferences();
bool persistCurrentConfig();
void loadSensorProfile();
bool persistSensorProfile(const ModbusSensorProfile &profile);
bool validateSensorProfile(const ModbusSensorProfile &profile, String &error);
bool parseSensorProfileJson(const String &json, ModbusSensorProfile &profile, String &error);
String sensorProfileJson(const ModbusSensorProfile &profile);
void generateDeviceId();
void startSetupPortal();
void activateRecoverySetupPortal(const char *reason);
void serviceSetupApLifetime();
void armSetupApLifetimeAfterHealthyPublish();
void announceSetupPortal(const char *reason);
void beginStaConnection(const char *reason);
void serviceWiFi();
void serviceMqtt();
void mqttMessageReceived(char *topic, uint8_t *payload, unsigned int length);
bool nodeServerHeartbeatFresh(unsigned long now);
String jsonEscape(const String &value);
String htmlEscape(const String &value);
String sanitizeIdentifier(const String &value);
String buildSetupApSsid();
bool requireSetupApRequest();
void handleConfig();
void handleSave();
void handleStatus();
void handleProfileUploadStream();
void handleProfileUploadComplete();
void handleProfileDownload();
void handleJetBrainsMonoFont();
bool profileUploadIsApiRequest();
void sendProfileUploadFailure(int statusCode, const String &message);
void handleNotFound();
void mqttReconnect();
void publishData();
void updateLed();
uint16_t modbusCrc16(const uint8_t *data, size_t len);
void logModbusError(const char *label, const char *reason, uint16_t detail = 0);
void flushModbusRx(uint16_t settleMs = 2);
bool modbusRecoveryActive();
unsigned long modbusRecoveryRemainingMs();
void restartModbusUart(const char *reason);
void powerCycleSensor();
void feedWatchdog();
uint64_t monotonicMillis();
void setSensorPollInterval(unsigned long intervalMs, bool persist);
bool readModbusRegisters(uint16_t startReg, uint16_t count, uint8_t *data, size_t dataLen, const char *label);
bool readModbusRegistersFunction(uint8_t readFunction, uint16_t startReg,
                                 uint16_t count, uint8_t *data,
                                 size_t dataLen, const char *label);
bool readSensorAccelerationRaw(float &ax, float &ay, float &az);
SensorEngineeringMetrics readSensorEngineeringMetrics();
bool readSensorClock(SensorClockTime &t);
bool calibrateSensorBaseline();
bool isKnownSensorBaud(uint32_t baud);
unsigned long rmsWindowMsForBaud(uint32_t baud);
uint8_t computeDiagnosticConfidence(uint16_t sampleCount, unsigned long windowMs, bool sensorOk, bool rmsReliable);
String healthStatusFromVelocity(float velocityOverall, bool velocityValid, uint8_t confidence);
void appendWaveformSample(float rawX, float rawY, float rawZ, float dynX, float dynY, float dynZ);
void rs485SetTransmit(bool tx);

// =============================================================================
// Preferences and persistent records
// =============================================================================
// [SENSOR PROFILE] [HIGH RISK]
// Stored structure layouts, schema versions, CRC coverage, slot selection, and
// Reordering stored fields or changing their types invalidates schema-3
// profiles/configuration already in NVS.
static const uint32_t CONFIG_MAGIC = 0x56494245UL; // "VIBE"
static const uint16_t CONFIG_SCHEMA_VERSION = 1;
static const uint32_t PROFILE_MAGIC = 0x56505246UL; // "VPRF"
static const uint16_t PROFILE_SCHEMA_VERSION = 3;
static const char    *PROFILE_STORAGE_NAMESPACE = "vibescope";

struct StoredSensorProfile {
  uint32_t magic;
  uint16_t schemaVersion;
  uint16_t structSize;
  uint32_t generation;
  ModbusSensorProfile profile;
  uint32_t crc;
};

static_assert(sizeof(StoredSensorProfile) <= 4096,
              "Sensor profile storage record is unexpectedly large for NVS");

// These records are deliberately static. A schema-3 profile is large enough
// that keeping several records on the 8 KB Arduino loop-task stack while an
// HTTP handler also owns a candidate profile can overflow the stack and reset
// the controller before the upload response is sent.
struct ProfilePersistenceWorkspace {
  StoredSensorProfile slotA;
  StoredSensorProfile slotB;
  StoredSensorProfile next;
  StoredSensorProfile verify;
};
static ProfilePersistenceWorkspace profilePersistenceWorkspace = {};

struct PersistentConfig {
  uint32_t magic;
  uint16_t schemaVersion;
  uint16_t structSize;
  uint32_t generation;
  char deviceId[33];
  char wifiSsid[33];
  char wifiPass[65];
  char mqttBroker[129];
  char mqttUser[65];
  char mqttPass[65];
  char setupApPass[64];
  int32_t mqttPort;
  uint32_t sensorBaud;
  uint32_t sensorPollIntervalMs;
  uint16_t sensorAccelRegBase;
  int8_t rs485DirPin;
  uint8_t sensorModbusId;
  uint8_t sensorReadFunction;
  uint8_t rs485DirActiveHigh;
  uint32_t crc;
};

uint32_t configCrc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (len-- > 0) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
    }
  }
  return ~crc;
}

uint16_t profileVectorRegister(const ModbusVectorProfile &vector, uint8_t axis) {
  if (axis == 0) return vector.startRegister;
  if (vector.usesExplicitAxisRegisters) {
    return axis == 1 ? vector.yRegister : vector.zRegister;
  }
  return vector.startRegister + axis;
}

uint8_t profileDataTypeBytes(ModbusProfileDataType type) {
  switch (type) {
    case ModbusProfileDataType::UInt16:
    case ModbusProfileDataType::Int16:
    case ModbusProfileDataType::Bcd16:
    case ModbusProfileDataType::Boolean:
      return 2;
    case ModbusProfileDataType::UInt32:
    case ModbusProfileDataType::Int32:
    case ModbusProfileDataType::Float32:
    case ModbusProfileDataType::Bcd32:
    case ModbusProfileDataType::UInt32Mod10000:
    case ModbusProfileDataType::Int32Mod10000:
      return 4;
    case ModbusProfileDataType::UInt64:
    case ModbusProfileDataType::Int64:
    case ModbusProfileDataType::Float64:
      return 8;
  }
  return 0;
}

bool profileDataTypeIsInteger(ModbusProfileDataType type) {
  return type != ModbusProfileDataType::Float32 &&
         type != ModbusProfileDataType::Float64;
}

void initializeBuiltInField(ModbusFieldProfile &field, uint16_t address,
                            float scale, bool signedRaw, uint8_t readFunction) {
  field = {};
  field.enabled = true;
  field.readFunction = readFunction;
  field.address = address;
  field.registerCount = 1;
  field.dataType = signedRaw
    ? ModbusProfileDataType::Int16 : ModbusProfileDataType::UInt16;
  snprintf(field.wireByteOrder, sizeof(field.wireByteOrder), "AB");
  field.scale = scale;
  field.offset = 0.0;
}

void initializeBuiltInGenericFields(ModbusSensorProfile &profile) {
  if (profile.usesGenericFieldMap) return;
  snprintf(profile.serialFormat, sizeof(profile.serialFormat), "8N1");
  profile.maximumReadRegisters = 32;
  profile.bulkReadFunction = profile.readFunction;
  for (uint8_t axis = 0; axis < 3; axis++) {
    initializeBuiltInField(
      profile.accelerationFields[axis],
      profileVectorRegister(profile.acceleration, axis),
      profile.acceleration.scale, profile.acceleration.signedRaw,
      profile.readFunction);
    initializeBuiltInField(
      profile.velocityFields[axis],
      profileVectorRegister(profile.velocity, axis),
      profile.velocity.scale, profile.velocity.signedRaw,
      profile.readFunction);
    initializeBuiltInField(
      profile.displacementFields[axis],
      profileVectorRegister(profile.displacement, axis),
      profile.displacement.scale, profile.displacement.signedRaw,
      profile.readFunction);
    initializeBuiltInField(
      profile.frequencyFields[axis],
      profileVectorRegister(profile.frequency, axis),
      profile.frequency.scale, profile.frequency.signedRaw,
      profile.readFunction);
  }
  initializeBuiltInField(
    profile.temperatureField, profile.temperature.registerAddress,
    profile.temperature.scale, profile.temperature.signedRaw,
    profile.readFunction);
  profile.usesGenericFieldMap = true;
}

bool profileWireByteOrderValid(const ModbusFieldProfile &field) {
  const uint8_t byteCount = profileDataTypeBytes(field.dataType);
  if (byteCount == 0 || strlen(field.wireByteOrder) != byteCount) return false;
  uint16_t seen = 0;
  for (uint8_t index = 0; index < byteCount; index++) {
    const char symbol = field.wireByteOrder[index];
    if (symbol < 'A' || symbol >= 'A' + byteCount) return false;
    const uint16_t bit = (uint16_t)1U << (symbol - 'A');
    if ((seen & bit) != 0) return false;
    seen |= bit;
  }
  return seen == ((uint16_t)1U << byteCount) - 1U;
}

bool validateProfileField(const ModbusFieldProfile &field,
                          const char *fieldName, String &error) {
  if (!field.enabled) return true;
  const uint8_t byteCount = profileDataTypeBytes(field.dataType);
  const uint8_t expectedRegisters = byteCount / 2U;
  if ((field.readFunction != 0x03 && field.readFunction != 0x04) ||
      field.registerCount != expectedRegisters ||
      (uint32_t)field.address + field.registerCount > 65536UL ||
      !profileWireByteOrderValid(field)) {
    error = String(fieldName) +
      " has an invalid function, address, registerCount, datatype, or byteOrder.";
    return false;
  }
  if (!isfinite(field.scale) || !isfinite(field.offset) ||
      field.scale == 0.0 || fabs(field.scale) > 1.0e12 ||
      fabs(field.offset) > 1.0e15) {
    error = String(fieldName) + " scale/offset is invalid or unsafe.";
    return false;
  }
  if (field.hasBitField) {
    const uint64_t widthMask = byteCount == 8
      ? UINT64_MAX : ((1ULL << (byteCount * 8U)) - 1ULL);
    const uint64_t shiftedMask = field.bitMask >> field.bitShift;
    const uint64_t discardedLowBits = field.bitShift == 0
      ? 0ULL : (field.bitMask & ((1ULL << field.bitShift) - 1ULL));
    if (!profileDataTypeIsInteger(field.dataType) ||
        field.bitMask == 0 || field.bitShift >= byteCount * 8U ||
        shiftedMask == 0 || discardedLowBits != 0 ||
        (field.bitMask & ~widthMask) != 0) {
      error = String(fieldName) + " has an invalid integer bit field.";
      return false;
    }
    if (field.bitFieldSigned &&
        (shiftedMask & (shiftedMask + 1ULL)) != 0) {
      error = String(fieldName) +
        " signed bitField mask must be contiguous after shift.";
      return false;
    }
  }
  if (field.hasInvalidRaw && byteCount < 8 &&
      field.invalidRaw >= (1ULL << (byteCount * 8U))) {
    error = String(fieldName) + ".invalidRaw exceeds the datatype width.";
    return false;
  }
  if (field.hasValidRange &&
      (!isfinite(field.validMinimum) || !isfinite(field.validMaximum) ||
       field.validMaximum < field.validMinimum)) {
    error = String(fieldName) + " has an invalid engineering validRange.";
    return false;
  }
  return true;
}

bool profileFieldFitsFrame(const ModbusSensorProfile &profile,
                           const ModbusFieldProfile &field) {
  if (!field.enabled) return true;
  const uint32_t frameStart = profile.fastFrameStartRegister;
  const uint32_t frameEnd = frameStart + profile.fastFrameRegisterCount;
  return field.readFunction == profile.bulkReadFunction &&
         field.address >= frameStart &&
         (uint32_t)field.address + field.registerCount <= frameEnd;
}

bool validateSensorProfile(const ModbusSensorProfile &profile, String &error) {
  const auto terminated = [](const char *value, size_t size) {
    return value[size - 1] == '\0' && value[0] != '\0';
  };
  if (!terminated(profile.profileKey, sizeof(profile.profileKey)) ||
      !terminated(profile.sensorType, sizeof(profile.sensorType)) ||
      !terminated(profile.modelName, sizeof(profile.modelName)) ||
      !terminated(profile.registerMapVersion, sizeof(profile.registerMapVersion))) {
    error = "Profile identity fields are missing, empty, or too long.";
    return false;
  }
  if (!isKnownSensorBaud(profile.defaultBaud)) {
    error = "modbus.baud is not in the supported baud list.";
    return false;
  }
  if (profile.defaultSlaveId < 1 || profile.defaultSlaveId > 247) {
    error = "modbus.slaveId must be between 1 and 247.";
    return false;
  }
  if (profile.readFunction != 0x03 && profile.readFunction != 0x04) {
    error = "modbus.readFunction must be 3 or 4.";
    return false;
  }
  if (!isfinite(profile.frequencyMinimum) || !isfinite(profile.frequencyMaximum) ||
      profile.frequencyMinimum < 0.0f ||
      profile.frequencyMaximum < profile.frequencyMinimum) {
    error = "frequencyRangeHz is invalid.";
    return false;
  }
  if (strcmp(profile.serialFormat, "8N1") != 0 &&
      strcmp(profile.serialFormat, "8N2") != 0 &&
      strcmp(profile.serialFormat, "8E1") != 0 &&
      strcmp(profile.serialFormat, "8E2") != 0 &&
      strcmp(profile.serialFormat, "8O1") != 0 &&
      strcmp(profile.serialFormat, "8O2") != 0) {
    error = "modbus.serialFormat must be 8N1, 8N2, 8E1, 8E2, 8O1, or 8O2.";
    return false;
  }
  if (profile.maximumReadRegisters < 1 ||
      profile.maximumReadRegisters > 125) {
    error = "modbus.maximumReadRegisters must be between 1 and 125.";
    return false;
  }
  const ModbusFieldProfile *fieldGroups[] = {
    profile.accelerationFields, profile.velocityFields,
    profile.displacementFields, profile.frequencyFields
  };
  const char *groupNames[] = {
    "acceleration", "velocity", "displacement", "frequency"
  };
  for (uint8_t group = 0; group < 4; group++) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      String fieldName = String("measurements.") + groupNames[group] +
                         "." + (axis == 0 ? "x" : axis == 1 ? "y" : "z");
      if (!validateProfileField(fieldGroups[group][axis],
                                fieldName.c_str(), error)) {
        return false;
      }
    }
  }
  if (!validateProfileField(profile.temperatureField,
                            "measurements.temperature.value", error)) {
    return false;
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    if (!profile.accelerationFields[axis].enabled ||
        !profile.velocityFields[axis].enabled) {
      error =
        "IVSS schema 3 requires X/Y/Z fields for acceleration and velocity.";
      return false;
    }
  }
  const bool displacementEnabled = profile.displacementFields[0].enabled;
  for (uint8_t axis = 1; axis < 3; axis++) {
    if (profile.displacementFields[axis].enabled != displacementEnabled) {
      error =
        "Displacement must provide all X/Y/Z fields or explicitly disable all three.";
      return false;
    }
  }
  const bool frequencyEnabled = profile.frequencyFields[0].enabled;
  for (uint8_t axis = 1; axis < 3; axis++) {
    if (profile.frequencyFields[axis].enabled != frequencyEnabled) {
      error =
        "Frequency must provide all X/Y/Z fields or explicitly disable all three.";
      return false;
    }
  }
  if (!frequencyEnabled &&
      (profile.frequencyMinimum != 0.0f ||
       profile.frequencyMaximum != 0.0f)) {
    error =
      "A disabled frequency group must use frequencyRangeHz minimum=0 and maximum=0.";
    return false;
  }
  if (!profile.temperatureField.enabled) {
    error = "IVSS schema 3 currently requires a temperature field.";
    return false;
  }
  if (profile.supportsVendorClock &&
      (profile.clockRegisterCount != 4 || profile.clockStartRegister > 0xFFFC)) {
    error = "vendorClock must use exactly four registers within the address range.";
    return false;
  }
  if (!profile.supportsVendorClock && profile.clockRegisterCount != 0) {
    error = "Disabled vendorClock must have registerCount 0.";
    return false;
  }
  if (profile.bulkReadFunction != 0x03 &&
      profile.bulkReadFunction != 0x04) {
    error = "bulkRead.function must be 3 or 4.";
    return false;
  }
  if (profile.supportsContiguousFastFrame) {
    const uint32_t frameEnd =
      (uint32_t)profile.fastFrameStartRegister + profile.fastFrameRegisterCount;
    bool allFieldsFit = profileFieldFitsFrame(profile, profile.temperatureField);
    for (uint8_t group = 0; group < 4 && allFieldsFit; group++) {
      for (uint8_t axis = 0; axis < 3; axis++) {
        if (!profileFieldFitsFrame(profile, fieldGroups[group][axis])) {
          allFieldsFit = false;
          break;
        }
      }
    }
    if (profile.fastFrameRegisterCount == 0 ||
        profile.fastFrameRegisterCount > profile.maximumReadRegisters ||
        profile.fastFrameRegisterCount > 125 ||
        frameEnd > 65536U ||
        !allFieldsFit) {
      error =
        "bulkRead must be 1-125 registers, respect maximumReadRegisters, and contain every enabled field using one function.";
      return false;
    }
  } else if (profile.fastFrameRegisterCount != 0) {
    error = "Disabled bulkRead must have registerCount 0.";
    return false;
  }
  if (profile.minimumPollIntervalMs < SENSOR_READ_MIN_INTERVAL_MS ||
      profile.minimumPollIntervalMs > SENSOR_READ_MAX_INTERVAL_MS) {
    error = "acquisition.minimumPollIntervalMs must be between 100 and 60000.";
    return false;
  }
  error = "";
  return true;
}

bool storedSensorProfileValid(const StoredSensorProfile &stored) {
  if (stored.magic != PROFILE_MAGIC ||
      stored.schemaVersion != PROFILE_SCHEMA_VERSION ||
      stored.structSize != sizeof(StoredSensorProfile) ||
      stored.crc != configCrc32(reinterpret_cast<const uint8_t *>(&stored),
                                offsetof(StoredSensorProfile, crc))) {
    return false;
  }
  String error;
  return validateSensorProfile(stored.profile, error);
}

bool readProfileSlot(Preferences &storage, const char *key, StoredSensorProfile &stored) {
  stored = {};
  return storage.getBytesLength(key) == sizeof(stored) &&
         storage.getBytes(key, &stored, sizeof(stored)) == sizeof(stored) &&
         storedSensorProfileValid(stored);
}

bool persistSensorProfile(const ModbusSensorProfile &profile) {
  String error;
  if (!validateSensorProfile(profile, error)) {
    profileStorageHealthy = false;
    lastProfileStorageError = "Profile validation failed: " + error;
    return false;
  }
  // Use a handle owned by this operation. A shared Preferences handle can be
  // left in an invalid state after an interrupted
  // HTTP request, making an otherwise valid profile write fail unpredictably.
  Preferences profileStorage;
  if (!profileStorage.begin(PROFILE_STORAGE_NAMESPACE, false)) {
    profileStorageHealthy = false;
    nvs_handle_t diagnosticHandle = 0;
    const esp_err_t openError = nvs_open(
      PROFILE_STORAGE_NAMESPACE, NVS_READWRITE, &diagnosticHandle);
    if (openError == ESP_OK) nvs_close(diagnosticHandle);
    nvs_stats_t stats = {};
    const esp_err_t statsError = nvs_get_stats(nullptr, &stats);
    lastProfileStorageError =
      "NVS namespace could not be opened: " +
      String(esp_err_to_name(openError));
    if (statsError == ESP_OK) {
      lastProfileStorageError +=
        " (used=" + String(stats.used_entries) +
        ", free=" + String(stats.free_entries) +
        ", total=" + String(stats.total_entries) + ")";
    }
    Serial.println("[Profile] " + lastProfileStorageError);
    return false;
  }
  StoredSensorProfile &slotA = profilePersistenceWorkspace.slotA;
  StoredSensorProfile &slotB = profilePersistenceWorkspace.slotB;
  slotA = {};
  slotB = {};
  const bool validA = readProfileSlot(profileStorage, "prof_a", slotA);
  const bool validB = readProfileSlot(profileStorage, "prof_b", slotB);
  const StoredSensorProfile *newest = nullptr;
  const char *targetKey = "prof_a";
  if (validA && validB) {
    newest = ((int32_t)(slotA.generation - slotB.generation) > 0) ? &slotA : &slotB;
    targetKey = newest == &slotA ? "prof_b" : "prof_a";
  } else if (validA) {
    newest = &slotA;
    targetKey = "prof_b";
  } else if (validB) {
    newest = &slotB;
    targetKey = "prof_a";
  }

  StoredSensorProfile &next = profilePersistenceWorkspace.next;
  next = {};
  next.magic = PROFILE_MAGIC;
  next.schemaVersion = PROFILE_SCHEMA_VERSION;
  next.structSize = sizeof(next);
  next.generation = newest ? newest->generation + 1U : 1U;
  next.profile = profile;
  next.crc = configCrc32(reinterpret_cast<const uint8_t *>(&next),
                         offsetof(StoredSensorProfile, crc));
  bool written =
    profileStorage.putBytes(targetKey, &next, sizeof(next)) == sizeof(next);
  if (!written) {
    // The target is the missing, invalid, or older redundant slot. Removing it
    // before one retry lets NVS reclaim a malformed/differently-sized blob
    // without sacrificing the newest valid copy.
    profileStorage.remove(targetKey);
    written =
      profileStorage.putBytes(targetKey, &next, sizeof(next)) == sizeof(next);
  }
  StoredSensorProfile &verify = profilePersistenceWorkspace.verify;
  verify = {};
  const size_t primaryLength = profileStorage.getBytesLength(targetKey);
  const bool primaryVerified =
    written && readProfileSlot(profileStorage, targetKey, verify) &&
    verify.generation == next.generation && verify.crc == next.crc &&
    memcmp(&verify.profile, &profile, sizeof(profile)) == 0;
  bool redundancyVerified = false;
  if (primaryVerified) {
    // Mirror the newly verified profile into the other slot. This leaves one
    // valid current copy if power fails during the mirror write, and two
    // independently verified current copies after a normal save.
    const char *mirrorKey = strcmp(targetKey, "prof_a") == 0 ? "prof_b" : "prof_a";
    next.generation++;
    next.crc = configCrc32(reinterpret_cast<const uint8_t *>(&next),
                           offsetof(StoredSensorProfile, crc));
    verify = {};
    bool mirrorWritten =
      profileStorage.putBytes(mirrorKey, &next, sizeof(next)) == sizeof(next);
    if (!mirrorWritten) {
      // The primary copy is already verified, so replacing the stale mirror is
      // power-failure safe and cannot discard the newly uploaded profile.
      profileStorage.remove(mirrorKey);
      mirrorWritten =
        profileStorage.putBytes(mirrorKey, &next, sizeof(next)) == sizeof(next);
    }
    redundancyVerified = mirrorWritten &&
      readProfileSlot(profileStorage, mirrorKey, verify) &&
      verify.generation == next.generation &&
      verify.crc == next.crc &&
      memcmp(&verify.profile, &profile, sizeof(profile)) == 0;
  }
  profileStorage.end();
  profileStorageHealthy = redundancyVerified;
  if (!primaryVerified) {
    if (!written) {
      lastProfileStorageError = "NVS write failed after retry";
    } else if (primaryLength != sizeof(next)) {
      lastProfileStorageError =
        "NVS record length mismatch: " + String((unsigned)primaryLength) +
        "/" + String((unsigned)sizeof(next));
    } else {
      lastProfileStorageError = "NVS read-back CRC/profile verification failed";
    }
    Serial.println("[Profile] Save verification failed: " +
                   lastProfileStorageError);
  } else if (!redundancyVerified) {
    lastProfileStorageError =
      "Primary profile saved; redundant mirror verification failed";
    Serial.println("[Profile] " + lastProfileStorageError);
  } else {
    lastProfileStorageError = "none";
  }
  return primaryVerified;
}

void loadSensorProfile() {
  ACTIVE_SENSOR_PROFILE = DEFAULT_SENSOR_PROFILE;
  initializeBuiltInGenericFields(ACTIVE_SENSOR_PROFILE);
  sensorProfileLoadedFromStorage = false;
  useSplitModbusReads = !ACTIVE_SENSOR_PROFILE.supportsContiguousFastFrame;

  StoredSensorProfile &slotA = profilePersistenceWorkspace.slotA;
  StoredSensorProfile &slotB = profilePersistenceWorkspace.slotB;
  slotA = {};
  slotB = {};
  bool validA = false;
  bool validB = false;
  bool anyProfileRecordPresent = false;

  Preferences storage;
  if (storage.begin(PROFILE_STORAGE_NAMESPACE, true)) {
    const size_t lengthA = storage.getBytesLength("prof_a");
    const size_t lengthB = storage.getBytesLength("prof_b");
    anyProfileRecordPresent = lengthA > 0 || lengthB > 0;
    validA = readProfileSlot(storage, "prof_a", slotA);
    validB = readProfileSlot(storage, "prof_b", slotB);
    storage.end();
  }

  if (validA || validB) {
    const StoredSensorProfile &selected =
      (validA && validB)
        ? (((int32_t)(slotA.generation - slotB.generation) > 0) ? slotA : slotB)
        : (validA ? slotA : slotB);
    ACTIVE_SENSOR_PROFILE = selected.profile;
    sensorProfileLoadedFromStorage = true;
    useSplitModbusReads = !ACTIVE_SENSOR_PROFILE.supportsContiguousFastFrame;
    profileStorageHealthy = validA && validB &&
      memcmp(&slotA.profile, &slotB.profile, sizeof(slotA.profile)) == 0;
    Serial.printf("[Profile] Loaded '%s' generation %lu.\n",
                  ACTIVE_SENSOR_PROFILE.profileKey,
                  (unsigned long)selected.generation);
    if (!profileStorageHealthy) {
      Serial.println(
        "[Profile] Profile storage is degraded; repairing redundancy.");
      persistSensorProfile(ACTIVE_SENSOR_PROFILE);
    }
  } else {
    // Empty or corrupted profile storage never prevents operation: the
    // firmware's audited profile remains active.
    profileStorageHealthy = !anyProfileRecordPresent;
    Serial.printf("[Profile] No validated uploaded profile; using built-in '%s'.\n",
                  ACTIVE_SENSOR_PROFILE.profileKey);
  }
}

bool persistentConfigValid(const PersistentConfig &cfg) {
  if (cfg.magic != CONFIG_MAGIC ||
      cfg.schemaVersion != CONFIG_SCHEMA_VERSION ||
      cfg.structSize != sizeof(PersistentConfig) ||
      cfg.crc != configCrc32(reinterpret_cast<const uint8_t *>(&cfg), offsetof(PersistentConfig, crc))) {
    return false;
  }
  if (cfg.deviceId[sizeof(cfg.deviceId) - 1] != '\0' ||
      cfg.wifiSsid[sizeof(cfg.wifiSsid) - 1] != '\0' ||
      cfg.wifiPass[sizeof(cfg.wifiPass) - 1] != '\0' ||
      cfg.mqttBroker[sizeof(cfg.mqttBroker) - 1] != '\0' ||
      cfg.mqttUser[sizeof(cfg.mqttUser) - 1] != '\0' ||
      cfg.mqttPass[sizeof(cfg.mqttPass) - 1] != '\0' ||
      cfg.setupApPass[sizeof(cfg.setupApPass) - 1] != '\0') {
    return false;
  }
  const size_t apPassLen = strnlen(cfg.setupApPass, sizeof(cfg.setupApPass));
  return cfg.mqttPort >= 1 && cfg.mqttPort <= 65535 &&
         isKnownSensorBaud(cfg.sensorBaud) &&
         cfg.sensorPollIntervalMs >= SENSOR_READ_MIN_INTERVAL_MS &&
         cfg.sensorPollIntervalMs <= SENSOR_READ_MAX_INTERVAL_MS &&
         (cfg.sensorReadFunction == 0x03 || cfg.sensorReadFunction == 0x04) &&
         (apPassLen == 0 || (apPassLen >= 8 && apPassLen <= 63));
}

void applyPersistentConfig(const PersistentConfig &cfg) {
  deviceId = cfg.deviceId;
  wifiSsid = cfg.wifiSsid;
  wifiPass = cfg.wifiPass;
  mqttBroker = cfg.mqttBroker;
  mqttUser = cfg.mqttUser;
  mqttPass = cfg.mqttPass;
  setupApPass = cfg.setupApPass;
  mqttPort = cfg.mqttPort;
  sensorBaud = cfg.sensorBaud;
  sensorPollIntervalMs = cfg.sensorPollIntervalMs;
  sensorAccelRegBase = cfg.sensorAccelRegBase;
  RS485_DE_RE_PIN = cfg.rs485DirPin;
  sensorModbusId = cfg.sensorModbusId;
  sensorReadFunction = cfg.sensorReadFunction;
  RS485_DIR_ACTIVE_HIGH = cfg.rs485DirActiveHigh != 0;
}

bool readConfigSlot(Preferences &storage, const char *key, PersistentConfig &cfg) {
  memset(&cfg, 0, sizeof(cfg));
  if (storage.getBytesLength(key) != sizeof(cfg)) return false;
  return storage.getBytes(key, &cfg, sizeof(cfg)) == sizeof(cfg) &&
         persistentConfigValid(cfg);
}

bool persistCurrentConfig() {
  // Use an operation-local handle so a stale handle from profile migration or
  // an interrupted HTTP operation cannot make configuration writes fail.
  Preferences configStorage;
  PersistentConfig slotA = {}, slotB = {};
  if (!configStorage.begin("vibescope", false)) {
    configStorageHealthy = false;
    lastConfigStorageError = "NVS namespace could not be opened";
    Serial.println("[Config] NVS open failed.");
    return false;
  }
  const bool validA = readConfigSlot(configStorage, "cfg_a", slotA);
  const bool validB = readConfigSlot(configStorage, "cfg_b", slotB);
  const PersistentConfig *newest = nullptr;
  const char *targetKey = "cfg_a";
  if (validA && validB) {
    newest = ((int32_t)(slotA.generation - slotB.generation) > 0) ? &slotA : &slotB;
    targetKey = (newest == &slotA) ? "cfg_b" : "cfg_a";
  } else if (validA) {
    newest = &slotA;
    targetKey = "cfg_b";
  } else if (validB) {
    newest = &slotB;
    targetKey = "cfg_a";
  }

  PersistentConfig next = {};
  next.magic = CONFIG_MAGIC;
  next.schemaVersion = CONFIG_SCHEMA_VERSION;
  next.structSize = sizeof(next);
  next.generation = newest ? newest->generation + 1U : 1U;
  deviceId.toCharArray(next.deviceId, sizeof(next.deviceId));
  wifiSsid.toCharArray(next.wifiSsid, sizeof(next.wifiSsid));
  wifiPass.toCharArray(next.wifiPass, sizeof(next.wifiPass));
  mqttBroker.toCharArray(next.mqttBroker, sizeof(next.mqttBroker));
  mqttUser.toCharArray(next.mqttUser, sizeof(next.mqttUser));
  mqttPass.toCharArray(next.mqttPass, sizeof(next.mqttPass));
  setupApPass.toCharArray(next.setupApPass, sizeof(next.setupApPass));
  next.mqttPort = mqttPort;
  next.sensorBaud = sensorBaud;
  next.sensorPollIntervalMs = sensorPollIntervalMs;
  next.sensorAccelRegBase = sensorAccelRegBase;
  next.rs485DirPin = (int8_t)RS485_DE_RE_PIN;
  next.sensorModbusId = sensorModbusId;
  next.sensorReadFunction = sensorReadFunction;
  next.rs485DirActiveHigh = RS485_DIR_ACTIVE_HIGH ? 1 : 0;
  next.crc = configCrc32(reinterpret_cast<const uint8_t *>(&next), offsetof(PersistentConfig, crc));

  bool written =
    configStorage.putBytes(targetKey, &next, sizeof(next)) == sizeof(next);
  if (!written) {
    // targetKey is always the missing, invalid, or older redundant slot. It is
    // therefore safe to remove and retry without destroying the newest valid
    // configuration record.
    configStorage.remove(targetKey);
    written =
      configStorage.putBytes(targetKey, &next, sizeof(next)) == sizeof(next);
  }
  bool identificationNameWritten = true;
  if (identificationName.length() == 0) {
    configStorage.remove("asset_name");
  } else {
    identificationNameWritten =
      configStorage.putString("asset_name", identificationName) == identificationName.length();
  }
  const bool setupApSsidWritten =
    configStorage.putString("ap_ssid", setupApSsid) == setupApSsid.length();
  bool wifiEntUserWritten = true;
  if (wifiEnterpriseUser.length() == 0) {
    configStorage.remove("wifi_ent_user");
  } else {
    wifiEntUserWritten =
      configStorage.putString("wifi_ent_user", wifiEnterpriseUser) == wifiEnterpriseUser.length();
  }
  PersistentConfig verify = {};
  const size_t storedLength = configStorage.getBytesLength(targetKey);
  const bool verified = written &&
                        readConfigSlot(configStorage, targetKey, verify) &&
                         verify.generation == next.generation && verify.crc == next.crc &&
                         identificationNameWritten &&
                         configStorage.getString("asset_name", "") == identificationName &&
                         setupApSsidWritten &&
                         configStorage.getString("ap_ssid", "") == setupApSsid &&
                         wifiEntUserWritten &&
                         configStorage.getString("wifi_ent_user", "") == wifiEnterpriseUser;
  configStorage.end();
  configStorageHealthy = verified;
  if (verified) {
    lastConfigStorageError = "none";
  } else if (!written) {
    lastConfigStorageError = "NVS write failed after retry";
  } else if (storedLength != sizeof(next)) {
    lastConfigStorageError =
      "NVS record length mismatch: " + String((unsigned)storedLength) +
      "/" + String((unsigned)sizeof(next));
  } else {
    lastConfigStorageError = "NVS read-back CRC verification failed";
  }
  if (!verified) {
    Serial.println("[Config] Save verification failed: " +
                   lastConfigStorageError);
  }
  return verified;
}

void loadPreferences() {
  loadSensorProfile();
  PersistentConfig slotA = {}, slotB = {};
  if (prefs.begin("vibescope", true)) {
    const bool validA = readConfigSlot(prefs, "cfg_a", slotA);
    const bool validB = readConfigSlot(prefs, "cfg_b", slotB);
    identificationName = prefs.getString("asset_name", "");
    setupApSsid = prefs.getString("ap_ssid", "");
    wifiEnterpriseUser = prefs.getString("wifi_ent_user", "");
    prefs.end();
    if (validA || validB) {
      const PersistentConfig &selected =
        (validA && validB)
          ? (((int32_t)(slotA.generation - slotB.generation) > 0) ? slotA : slotB)
          : (validA ? slotA : slotB);
      applyPersistentConfig(selected);
#if defined(SENSOR_ON_PROGRAM_UART_PINS)
      RS485_DE_RE_PIN = -1;
#endif
      const unsigned long profilePollMinimum =
        max((unsigned long)SENSOR_READ_MIN_INTERVAL_MS,
            (unsigned long)ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs);
      const bool pollIntervalChanged =
        sensorPollIntervalMs < profilePollMinimum ||
        sensorPollIntervalMs > SENSOR_READ_MAX_INTERVAL_MS;
      if (pollIntervalChanged) sensorPollIntervalMs = profilePollMinimum;
      const String storedDeviceId = deviceId;
      const String storedSetupApSsid = setupApSsid;
      const String storedSetupApPass = setupApPass;
      generateDeviceId();
      const bool configChanged =
        deviceId != storedDeviceId ||
        setupApSsid != storedSetupApSsid ||
        setupApPass != storedSetupApPass ||
        sensorBaud != ACTIVE_SENSOR_PROFILE.defaultBaud ||
        sensorModbusId != ACTIVE_SENSOR_PROFILE.defaultSlaveId ||
        sensorAccelRegBase != ACTIVE_SENSOR_PROFILE.acceleration.startRegister ||
        sensorReadFunction != ACTIVE_SENSOR_PROFILE.readFunction ||
        pollIntervalChanged;
      sensorBaud = ACTIVE_SENSOR_PROFILE.defaultBaud;
      sensorModbusId = ACTIVE_SENSOR_PROFILE.defaultSlaveId;
      sensorAccelRegBase = ACTIVE_SENSOR_PROFILE.acceleration.startRegister;
      sensorReadFunction = ACTIVE_SENSOR_PROFILE.readFunction;
      if (configChanged && !persistCurrentConfig()) {
        Serial.println("[Config] ERROR: build identity/profile could not be persisted.");
      }
      Serial.printf("[Config] Loaded validated configuration generation %lu.\n",
                    (unsigned long)selected.generation);
      return;
    }
  } else {
    configStorageHealthy = false;
  }

  // One-time migration path for firmware versions that stored separate keys.
  if (!prefs.begin("vibescope", false)) {
    configStorageHealthy = false;
    generateDeviceId();
    sensorBaud = ACTIVE_SENSOR_PROFILE.defaultBaud;
    sensorModbusId = ACTIVE_SENSOR_PROFILE.defaultSlaveId;
    sensorAccelRegBase = ACTIVE_SENSOR_PROFILE.acceleration.startRegister;
    sensorReadFunction = ACTIVE_SENSOR_PROFILE.readFunction;
    return;
  }
  deviceId   = prefs.getString("device_id",    "");
  identificationName = prefs.getString("asset_name", "");
  setupApSsid = prefs.getString("ap_ssid", "");
  wifiSsid   = prefs.getString("wifi_ssid",    "");
  wifiPass   = prefs.getString("wifi_pass",    "");
  wifiEnterpriseUser = prefs.getString("wifi_ent_user", "");
  mqttBroker = prefs.getString("mqtt_broker",  "");
  mqttUser   = prefs.getString("mqtt_user",    "");
  mqttPass   = prefs.getString("mqtt_pass",    "");
  mqttPort   = prefs.getInt   ("mqtt_port",    1883);
  setupApPass = prefs.getString("setup_ap_pass", SETUP_PIN);
  sensorBaud = prefs.getUInt  ("sensor_baud",  SENSOR_BAUD_DEFAULT);
  sensorModbusId = (uint8_t)prefs.getUChar("sensor_id", SENSOR_MODBUS_ID_DEFAULT);
  sensorAccelRegBase = (uint16_t)prefs.getUShort("sensor_reg", SENSOR_REG_ACCEL_BASE_DEFAULT);
  sensorReadFunction = (uint8_t)prefs.getUChar("sensor_fcode", SENSOR_READ_FUNC_DEFAULT);
  RS485_DE_RE_PIN = prefs.getInt("rs485_dir_pin", -1);
  RS485_DIR_ACTIVE_HIGH = prefs.getBool("rs485_dir_hi", true);
  sensorPollIntervalMs = prefs.getULong("sensor_poll_ms", SENSOR_READ_MIN_INTERVAL_MS);
  prefs.end();
  generateDeviceId();
  sensorBaud = ACTIVE_SENSOR_PROFILE.defaultBaud;
  sensorModbusId = ACTIVE_SENSOR_PROFILE.defaultSlaveId;
  sensorAccelRegBase = ACTIVE_SENSOR_PROFILE.acceleration.startRegister;
  sensorReadFunction = ACTIVE_SENSOR_PROFILE.readFunction;

#if defined(SENSOR_ON_PROGRAM_UART_PINS)
  // This build is explicitly for a two-wire-TTL auto-direction converter.
  // Ignore any stale manual DE/RE setting retained in NVS by older firmware.
  RS485_DE_RE_PIN = -1;
#endif

  if (mqttPort < 1 || mqttPort > 65535) {
    Serial.printf("[Config] Saved MQTT port %d is invalid. Falling back to 1883.\n", mqttPort);
    mqttPort = 1883;
  }

  const unsigned long profilePollMinimum =
    max((unsigned long)SENSOR_READ_MIN_INTERVAL_MS,
        (unsigned long)ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs);
  if (sensorPollIntervalMs < profilePollMinimum ||
      sensorPollIntervalMs > SENSOR_READ_MAX_INTERVAL_MS) {
    Serial.printf("[Sensor] Saved poll interval %lu ms is invalid for this profile. Falling back to %lu ms.\n",
                  sensorPollIntervalMs, profilePollMinimum);
    sensorPollIntervalMs = profilePollMinimum;
  }

  if (!persistCurrentConfig()) {
    Serial.println("[Config] ERROR: could not create a verified redundant configuration record.");
  }
  Serial.println("[Config] Device ID: " + deviceId);
  Serial.printf("[Sensor] Modbus id=0x%02X baud=%lu fcode=0x%02X accelReg=0x%04X rs485DirPin=%d dirHigh=%d\n",
                sensorModbusId, (unsigned long)sensorBaud, sensorReadFunction,
                sensorAccelRegBase, RS485_DE_RE_PIN, RS485_DIR_ACTIVE_HIGH ? 1 : 0);
  Serial.printf("[Sensor] Preferred baud: %lu. Sensor must already be configured to this baud.\n",
                (unsigned long)ACTIVE_SENSOR_PROFILE.defaultBaud);
}

void generateDeviceId() {
  const String previousDeviceId = deviceId;
  const int suffixSeparator = previousDeviceId.lastIndexOf('-');
  const String previousSuffix = suffixSeparator >= 0
    ? previousDeviceId.substring(suffixSeparator + 1)
    : String("");
  const bool ssidWasFactoryDefault =
    setupApSsid.length() == 0 || setupApSsid.length() > 32 ||
    setupApSsid == previousDeviceId ||
    (previousDeviceId.length() > 0 && setupApSsid.endsWith(previousDeviceId));
  const bool passWasFactoryDefault =
    setupApPass.length() < 8 || setupApPass.length() > 63 ||
    setupApPass == previousDeviceId || setupApPass == previousSuffix;

  deviceId = BUILD_SERIAL_NUMBER;
  if (ssidWasFactoryDefault) setupApSsid = deviceId;
  if (passWasFactoryDefault) setupApPass = SETUP_PIN;
  Serial.println("[Identity] Firmware serial/device ID: " + deviceId);
}

bool isKnownSensorBaud(uint32_t baud) {
  // The UART accepts non-standard vendor rates too. Auto-probing remains
  // bounded to KNOWN_SENSOR_BAUDS, but an explicit audited profile may use
  // any practical sensor rate in this range.
  return baud >= 300U && baud <= 2000000U;
}

// ---------------------------------------------------------------------------
// [CALCULATIONS] Engineering quality, validity, and waveform helpers
// ---------------------------------------------------------------------------
// [HIGH RISK] These functions define confidence, alarm-state interpretation,
// accepted numeric ranges, and the samples later exposed to data consumers.
unsigned long rmsWindowMsForBaud(uint32_t baud) {
  if (baud <= 4800) return 500;
  if (baud <= 9600) return RMS_WINDOW_MS;
  return RMS_WINDOW_MS;
}

uint8_t computeDiagnosticConfidence(uint16_t sampleCount, unsigned long windowMs, bool sensorOk, bool rmsReliable) {
  int score = 100;
  if (!sensorOk) score -= 50;
  if (!baselineCalibrated) score -= 15;
  if (!rmsReliable) score -= 20;
  if (sampleCount == 0) score -= 30;
  if (sensorBaud != ACTIVE_SENSOR_PROFILE.defaultBaud) score -= 8;
  if (modbusTimeoutCount > 0) score -= (int)min<uint32_t>(20, modbusTimeoutCount);
  if (modbusCrcFailCount > 0) score -= (int)min<uint32_t>(15, modbusCrcFailCount * 2);
  if (modbusExceptionCount > 0) score -= (int)min<uint32_t>(20, modbusExceptionCount * 3);
  if (windowMs > 0 && sampleCount > 0) {
    float rate = 1000.0f * sampleCount / windowMs;
    const float expectedRate =
      1000.0f / (float)max((unsigned long)SENSOR_READ_MIN_INTERVAL_MS,
                           (unsigned long)ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs);
    if (rate < expectedRate * 0.8f) score -= 20;
    else if (rate < expectedRate * 0.95f) score -= 10;
  }
  if (score < 0) score = 0;
  if (score > 100) score = 100;
  return (uint8_t)score;
}

String healthStatusFromVelocity(float velocityOverall, bool velocityValid, uint8_t confidence) {
  if (confidence < 45) return "LowConfidence";
  if (!velocityValid || !ACTIVE_SENSOR_PROFILE.velocityIsRms) {
    return "DiagnosticOnly";
  }
  if (velocityOverall >= VELOCITY_CRITICAL_MM_S) return "Critical";
  if (velocityOverall >= VELOCITY_ALARM_MM_S) return "Alarm";
  if (velocityOverall >= VELOCITY_ALERT_MM_S) return "Alert";
  return "Normal";
}

bool isFiniteSensorFloat(float value) {
  return (value == value) && value > -3.4028234e38f && value < 3.4028234e38f;
}

bool isValidSensorFrequency(float value) {
  return isFiniteSensorFloat(value) &&
         value >= SENSOR_FREQUENCY_MIN_HZ &&
         value <= SENSOR_FREQUENCY_MAX_HZ;
}

const char *sensorAxisQualityFlag(bool valid, float value, float lsb) {
  if (!valid) return "missing";
  if (!isFiniteSensorFloat(value) || value < 0.0f) return "invalid";
  if (value <= 0.0f && lsb > 0.0f) return "below_resolution";
  return "valid";
}

String jsonNumberOrNull(bool valid, float value, uint8_t decimals) {
  return (valid && isFiniteSensorFloat(value)) ? String(value, (unsigned int)decimals) : String("null");
}

String uint64Json(uint64_t value) {
  char buffer[21];
  snprintf(buffer, sizeof(buffer), "%llu",
           (unsigned long long)value);
  // JSON/JavaScript numbers are exact only through 2^53-1. Preserve existing
  // numeric telemetry for normal 16/32-bit registers and quote larger raw
  // values so dashboards cannot silently round a uint64 sensor register.
  if (value <= 9007199254740991ULL) return String(buffer);
  return String("\"") + buffer + "\"";
}

String sensorAxisJsonValue(bool valid, float value, float lsb, uint8_t decimals) {
  // Zero is a real engineering value (stationary/below sensor resolution), not
  // missing data. Preserve it numerically and describe its quality separately.
  (void)lsb; // resolution is carried separately in measurementQuality
  if (!valid || !isFiniteSensorFloat(value) || value < 0.0f) return String("null");
  return String(value, (unsigned int)decimals);
}

void appendWaveformSample(float rawX, float rawY, float rawZ, float dynX, float dynY, float dynZ) {
  rawWaveX[waveWriteIndex] = rawX;
  rawWaveY[waveWriteIndex] = rawY;
  rawWaveZ[waveWriteIndex] = rawZ;
  dynWaveX[waveWriteIndex] = dynX;
  dynWaveY[waveWriteIndex] = dynY;
  dynWaveZ[waveWriteIndex] = dynZ;
  waveWriteIndex = (waveWriteIndex + 1) % RAW_WINDOW_SIZE;
  if (waveSamplesStored < RAW_WINDOW_SIZE) {
    waveSamplesStored++;
  }
}

// =============================================================================
// [WIFI AP] AP+STA, setup portal, and non-blocking link services [HIGH RISK]
// =============================================================================
// ESP32 SoftAP and STA share one physical radio/channel. Changes to WiFi.mode(),
// WiFi.disconnect(), WiFi.begin(), scanning, or their call timing can make the
// setup SSID intermittent even when WiFi.softAP() initially reports success.
String jsonEscape(const String &value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); i++) {
    char c = value.charAt(i);
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if ((uint8_t)c < 0x20) {
      char buf[7];
      snprintf(buf, sizeof(buf), "\\u%04X", (uint8_t)c);
      out += buf;
    } else {
      out += c;
    }
  }
  return out;
}

String htmlEscape(const String &value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); i++) {
    char c = value.charAt(i);
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else if (c == '\'') out += "&#39;";
    else out += c;
  }
  return out;
}

bool copyProfileText(JsonVariantConst value, char *destination, size_t capacity,
                     const char *fieldName, String &error) {
  if (!value.is<const char *>()) {
    error = String(fieldName) + " must be a string.";
    return false;
  }
  const char *text = value.as<const char *>();
  const size_t length = strlen(text);
  if (length == 0 || length >= capacity) {
    error = String(fieldName) + " is empty or exceeds its maximum length.";
    return false;
  }
  for (size_t i = 0; i < length; i++) {
    if ((uint8_t)text[i] < 0x20) {
      error = String(fieldName) + " contains a control character.";
      return false;
    }
  }
  memcpy(destination, text, length + 1);
  return true;
}

bool parseProfileU16(JsonVariantConst value, uint16_t &result) {
  if (value.is<const char *>()) {
    const char *text = value.as<const char *>();
    char *end = nullptr;
    const bool hexadecimal =
      strlen(text) >= 2 && text[0] == '0' &&
      (text[1] == 'x' || text[1] == 'X');
    unsigned long parsed = strtoul(text, &end, hexadecimal ? 16 : 10);
    if (text == end || *end != '\0' || parsed > 0xFFFFUL) return false;
    result = (uint16_t)parsed;
    return true;
  }
  if (!value.is<unsigned long>() && !value.is<long>()) return false;
  const long parsed = value.as<long>();
  if (parsed < 0 || parsed > 65535L) return false;
  result = (uint16_t)parsed;
  return true;
}

bool parseProfileU64(JsonVariantConst value, uint64_t &result) {
  if (value.is<const char *>()) {
    const char *text = value.as<const char *>();
    if (!text || text[0] == '\0' || text[0] == '-' || text[0] == '+') {
      return false;
    }
    char *end = nullptr;
    const bool hexadecimal =
      strlen(text) >= 2 && text[0] == '0' &&
      (text[1] == 'x' || text[1] == 'X');
    errno = 0;
    unsigned long long parsed =
      strtoull(text, &end, hexadecimal ? 16 : 10);
    if (errno == ERANGE || text == end || *end != '\0') return false;
    result = (uint64_t)parsed;
    return true;
  }
  if (!value.is<unsigned long>()) return false;
  result = value.as<unsigned long>();
  return true;
}

bool parseProfileDataType(const char *name, ModbusProfileDataType &type) {
  if (!name) return false;
  struct TypeName {
    const char *name;
    ModbusProfileDataType type;
  };
  static const TypeName types[] = {
    {"uint16", ModbusProfileDataType::UInt16},
    {"int16", ModbusProfileDataType::Int16},
    {"uint32", ModbusProfileDataType::UInt32},
    {"int32", ModbusProfileDataType::Int32},
    {"uint64", ModbusProfileDataType::UInt64},
    {"int64", ModbusProfileDataType::Int64},
    {"float32", ModbusProfileDataType::Float32},
    {"float64", ModbusProfileDataType::Float64},
    {"bcd16", ModbusProfileDataType::Bcd16},
    {"bcd32", ModbusProfileDataType::Bcd32},
    {"uint32_mod10000", ModbusProfileDataType::UInt32Mod10000},
    {"int32_mod10000", ModbusProfileDataType::Int32Mod10000},
    {"boolean", ModbusProfileDataType::Boolean}
  };
  for (const auto &candidate : types) {
    if (strcmp(name, candidate.name) == 0) {
      type = candidate.type;
      return true;
    }
  }
  return false;
}

const char *profileDataTypeName(ModbusProfileDataType type) {
  switch (type) {
    case ModbusProfileDataType::UInt16: return "uint16";
    case ModbusProfileDataType::Int16: return "int16";
    case ModbusProfileDataType::UInt32: return "uint32";
    case ModbusProfileDataType::Int32: return "int32";
    case ModbusProfileDataType::UInt64: return "uint64";
    case ModbusProfileDataType::Int64: return "int64";
    case ModbusProfileDataType::Float32: return "float32";
    case ModbusProfileDataType::Float64: return "float64";
    case ModbusProfileDataType::Bcd16: return "bcd16";
    case ModbusProfileDataType::Bcd32: return "bcd32";
    case ModbusProfileDataType::UInt32Mod10000: return "uint32_mod10000";
    case ModbusProfileDataType::Int32Mod10000: return "int32_mod10000";
    case ModbusProfileDataType::Boolean: return "boolean";
  }
  return "unknown";
}

template <size_t N>
bool profileObjectHasOnlyKeys(
  JsonObjectConst object, const char *objectName,
  const char *const (&allowed)[N], String &error
) {
  for (JsonPairConst pair : object) {
    bool known = false;
    for (size_t index = 0; index < N; index++) {
      if (strcmp(pair.key().c_str(), allowed[index]) == 0) {
        known = true;
        break;
      }
    }
    if (!known) {
      error = String(objectName) + " contains unknown key '" +
              pair.key().c_str() + "'.";
      return false;
    }
  }
  return true;
}

bool parseGenericProfileField(JsonObjectConst object, const char *fieldName,
                              ModbusFieldProfile &field, String &error) {
  field = {};
  static const char *const allowedFieldKeys[] = {
    "enabled", "function", "address", "registerCount", "dataType",
    "byteOrder", "scale", "offset", "bitField", "invalidRaw", "validRange"
  };
  if (object.isNull() ||
      !profileObjectHasOnlyKeys(
        object, fieldName, allowedFieldKeys, error) ||
      !object["enabled"].is<bool>()) {
    if (error.length() > 0) return false;
    error = String(fieldName) + ".enabled must be boolean.";
    return false;
  }
  field.enabled = object["enabled"].as<bool>();
  if (!field.enabled) return true;
  if (!object["function"].is<unsigned int>() ||
      !parseProfileU16(object["address"], field.address) ||
      !object["registerCount"].is<unsigned int>() ||
      !object["dataType"].is<const char *>() ||
      !object["byteOrder"].is<const char *>() ||
      !object["scale"].is<double>() ||
      !object["offset"].is<double>()) {
    error = String(fieldName) +
      " requires function, address, registerCount, dataType, byteOrder, scale, and offset.";
    return false;
  }
  const uint32_t function = object["function"].as<uint32_t>();
  const uint32_t registerCount = object["registerCount"].as<uint32_t>();
  if (function > 255U || registerCount > 255U ||
      !parseProfileDataType(object["dataType"].as<const char *>(),
                            field.dataType) ||
      !copyProfileText(object["byteOrder"], field.wireByteOrder,
                       sizeof(field.wireByteOrder), fieldName, error)) {
    if (error.length() == 0) {
      error = String(fieldName) +
        " has an unsupported function, register count, datatype, or byte order.";
    }
    return false;
  }
  field.readFunction = (uint8_t)function;
  field.registerCount = (uint8_t)registerCount;
  field.scale = object["scale"].as<double>();
  field.offset = object["offset"].as<double>();

  JsonObjectConst bitField = object["bitField"].as<JsonObjectConst>();
  if (!bitField.isNull()) {
    static const char *const allowedBitFieldKeys[] = {
      "mask", "shift", "signed"
    };
    if (!profileObjectHasOnlyKeys(
          bitField, "bitField", allowedBitFieldKeys, error)) {
      return false;
    }
    if (!parseProfileU64(bitField["mask"], field.bitMask) ||
        !bitField["shift"].is<unsigned int>() ||
        bitField["shift"].as<uint32_t>() > 63U ||
        !bitField["signed"].is<bool>()) {
      error = String(fieldName) +
        ".bitField requires mask, shift, and signed.";
      return false;
    }
    field.bitShift = (uint8_t)bitField["shift"].as<uint32_t>();
    field.bitFieldSigned = bitField["signed"].as<bool>();
    field.hasBitField = true;
  }
  if (object.containsKey("invalidRaw")) {
    if (!parseProfileU64(object["invalidRaw"], field.invalidRaw)) {
      error = String(fieldName) +
        ".invalidRaw must be an exact integer or decimal/hex string.";
      return false;
    }
    field.hasInvalidRaw = true;
  }
  JsonObjectConst validRange = object["validRange"].as<JsonObjectConst>();
  if (!validRange.isNull()) {
    static const char *const allowedRangeKeys[] = {"minimum", "maximum"};
    if (!profileObjectHasOnlyKeys(
          validRange, "validRange", allowedRangeKeys, error)) {
      return false;
    }
    if (!validRange["minimum"].is<double>() ||
        !validRange["maximum"].is<double>()) {
      error = String(fieldName) +
        ".validRange requires numeric minimum and maximum.";
      return false;
    }
    field.validMinimum = validRange["minimum"].as<double>();
    field.validMaximum = validRange["maximum"].as<double>();
    field.hasValidRange = true;
  }
  return validateProfileField(field, fieldName, error);
}

bool parseGenericVector(JsonObjectConst object, const char *name,
                        const char *expectedUnit,
                        ModbusFieldProfile fields[3], String &semantics,
                        String &error) {
  static const char *const allowedVectorKeys[] = {
    "unit", "semantics", "x", "y", "z"
  };
  if (object.isNull() || !object["unit"].is<const char *>() ||
      !profileObjectHasOnlyKeys(
        object, name, allowedVectorKeys, error) ||
      strcmp(object["unit"].as<const char *>(), expectedUnit) != 0 ||
      !object["semantics"].is<const char *>()) {
    if (error.length() > 0) return false;
    error = String("measurements.") + name +
      " requires the canonical unit and semantics.";
    return false;
  }
  semantics = object["semantics"].as<const char *>();
  const char *axes[] = {"x", "y", "z"};
  for (uint8_t axis = 0; axis < 3; axis++) {
    String fieldName = String("measurements.") + name + "." + axes[axis];
    if (!parseGenericProfileField(object[axes[axis]].as<JsonObjectConst>(),
                                  fieldName.c_str(), fields[axis], error)) {
      return false;
    }
  }
  return true;
}

void syncRuntimeProfileFields(ModbusSensorProfile &profile) {
  const auto syncVector = [](const ModbusFieldProfile fields[3],
                             ModbusVectorProfile &legacy) {
    legacy = {};
    legacy.startRegister = fields[0].address;
    legacy.yRegister = fields[1].address;
    legacy.zRegister = fields[2].address;
    legacy.usesExplicitAxisRegisters =
      fields[1].address != fields[0].address + fields[0].registerCount ||
      fields[2].address != fields[1].address + fields[1].registerCount;
    legacy.scale = (float)fabs(fields[0].scale);
    legacy.signedRaw =
      fields[0].dataType == ModbusProfileDataType::Int16;
  };
  syncVector(profile.accelerationFields, profile.acceleration);
  syncVector(profile.velocityFields, profile.velocity);
  syncVector(profile.displacementFields, profile.displacement);
  syncVector(profile.frequencyFields, profile.frequency);
  profile.temperature.registerAddress = profile.temperatureField.address;
  profile.temperature.scale = (float)fabs(profile.temperatureField.scale);
  profile.temperature.signedRaw =
    profile.temperatureField.dataType == ModbusProfileDataType::Int16;
}

bool parseSensorProfileJson(const String &json, ModbusSensorProfile &profile, String &error) {
  if (json.length() == 0 || json.length() > SENSOR_PROFILE_UPLOAD_MAX_BYTES) {
    error = "Profile JSON is empty or exceeds the upload limit.";
    return false;
  }
  profileJsonDocument.clear();
  const DeserializationError jsonError =
    deserializeJson(profileJsonDocument, json,
                    DeserializationOption::NestingLimit(12));
  if (jsonError) {
    error = String("Invalid JSON: ") + jsonError.c_str();
    return false;
  }
  JsonObjectConst root = profileJsonDocument.as<JsonObjectConst>();
  if (root.isNull() || !root["schemaVersion"].is<int>() ||
      root["schemaVersion"].as<int>() != PROFILE_SCHEMA_VERSION) {
    error = "Only sensor profile schemaVersion 3 is supported.";
    return false;
  }
  static const char *const allowedRootKeys[] = {
    "schemaVersion", "profileKey", "sensorType", "modelName",
    "registerMapVersion", "modbus", "measurements",
    "frequencyRangeHz", "acquisition", "vendorClock", "bulkRead"
  };
  if (!profileObjectHasOnlyKeys(
        root, "profile", allowedRootKeys, error)) {
    return false;
  }

  ModbusSensorProfile parsed = {};
  if (!copyProfileText(root["profileKey"], parsed.profileKey,
                       sizeof(parsed.profileKey), "profileKey", error) ||
      !copyProfileText(root["sensorType"], parsed.sensorType,
                       sizeof(parsed.sensorType), "sensorType", error) ||
      !copyProfileText(root["modelName"], parsed.modelName,
                       sizeof(parsed.modelName), "modelName", error) ||
      !copyProfileText(root["registerMapVersion"], parsed.registerMapVersion,
                       sizeof(parsed.registerMapVersion), "registerMapVersion", error)) {
    return false;
  }

  JsonObjectConst modbus = root["modbus"].as<JsonObjectConst>();
  if (modbus.isNull() ||
      !modbus["baud"].is<unsigned long>() ||
      !modbus["slaveId"].is<unsigned int>()) {
    error = "modbus requires numeric baud and slaveId.";
    return false;
  }
  const uint32_t parsedSlaveId = modbus["slaveId"].as<uint32_t>();
  if (parsedSlaveId > 255U) {
    error = "modbus.slaveId must fit in one byte.";
    return false;
  }
  parsed.defaultBaud = modbus["baud"].as<uint32_t>();
  parsed.defaultSlaveId = (uint8_t)parsedSlaveId;
  static const char *const allowedModbusKeys[] = {
    "mode", "baud", "slaveId", "serialFormat", "addressConvention",
    "defaultReadFunction", "maximumReadRegisters"
  };
  if (!profileObjectHasOnlyKeys(
        modbus, "modbus", allowedModbusKeys, error)) {
    return false;
  }
  if (!modbus["mode"].is<const char *>() ||
      strcmp(modbus["mode"].as<const char *>(), "rtu") != 0 ||
      !modbus["serialFormat"].is<const char *>() ||
      !modbus["addressConvention"].is<const char *>() ||
      strcmp(modbus["addressConvention"].as<const char *>(),
             "pdu_zero_based") != 0 ||
      !modbus["defaultReadFunction"].is<unsigned int>() ||
      !modbus["maximumReadRegisters"].is<unsigned int>()) {
    error =
      "Schema 3 modbus requires mode=rtu, serialFormat, addressConvention=pdu_zero_based, defaultReadFunction, and maximumReadRegisters.";
    return false;
  }
  const uint32_t defaultFunction =
    modbus["defaultReadFunction"].as<uint32_t>();
  const uint32_t maximumRead =
    modbus["maximumReadRegisters"].as<uint32_t>();
  if (defaultFunction > 255U || maximumRead > 255U ||
      !copyProfileText(modbus["serialFormat"], parsed.serialFormat,
                       sizeof(parsed.serialFormat),
                       "modbus.serialFormat", error)) {
    return false;
  }
  parsed.readFunction = (uint8_t)defaultFunction;
  parsed.maximumReadRegisters = (uint8_t)maximumRead;
  parsed.usesGenericFieldMap = true;

  JsonObjectConst measurements = root["measurements"].as<JsonObjectConst>();
  static const char *const allowedMeasurementKeys[] = {
    "acceleration", "velocity", "displacement", "frequency", "temperature"
  };
  String accelerationSemantics;
  String velocitySemantics;
  String ignoredSemantics;
  if (measurements.isNull() ||
      !profileObjectHasOnlyKeys(
        measurements, "measurements", allowedMeasurementKeys, error) ||
      !parseGenericVector(
        measurements["acceleration"].as<JsonObjectConst>(),
        "acceleration", "m/s^2", parsed.accelerationFields,
        accelerationSemantics, error) ||
      !parseGenericVector(
        measurements["velocity"].as<JsonObjectConst>(),
        "velocity", "mm/s", parsed.velocityFields,
        velocitySemantics, error) ||
      !parseGenericVector(
        measurements["displacement"].as<JsonObjectConst>(),
        "displacement", "um", parsed.displacementFields,
        ignoredSemantics, error) ||
      !parseGenericVector(
        measurements["frequency"].as<JsonObjectConst>(),
        "frequency", "Hz", parsed.frequencyFields,
        ignoredSemantics, error)) {
    return false;
  }
  if (accelerationSemantics == "instantaneous_waveform") {
    parsed.accelerationIsInstantaneousWaveform = true;
  } else if (accelerationSemantics == "sensor_reported_magnitude") {
    parsed.accelerationIsInstantaneousWaveform = false;
  } else {
    error =
      "measurements.acceleration.semantics must be instantaneous_waveform or sensor_reported_magnitude.";
    return false;
  }
  if (velocitySemantics == "sensor_reported_rms") {
    parsed.velocityIsRms = true;
  } else if (velocitySemantics == "sensor_reported_unspecified") {
    parsed.velocityIsRms = false;
  } else {
    error =
      "measurements.velocity.semantics must be sensor_reported_rms or sensor_reported_unspecified.";
    return false;
  }
  JsonObjectConst temperature =
    measurements["temperature"].as<JsonObjectConst>();
  static const char *const allowedTemperatureKeys[] = {"unit", "value"};
  if (temperature.isNull() ||
      !profileObjectHasOnlyKeys(
        temperature, "measurements.temperature",
        allowedTemperatureKeys, error) ||
      !temperature["unit"].is<const char *>() ||
      strcmp(temperature["unit"].as<const char *>(), "C") != 0 ||
      !parseGenericProfileField(
        temperature["value"].as<JsonObjectConst>(),
        "measurements.temperature.value",
        parsed.temperatureField, error)) {
    if (error.length() == 0) {
      error = "measurements.temperature requires unit C and a value field.";
    }
    return false;
  }

  JsonObjectConst frequencyRange =
    root["frequencyRangeHz"].as<JsonObjectConst>();
  static const char *const allowedRangeKeys[] = {"minimum", "maximum"};
  if (frequencyRange.isNull() ||
      !profileObjectHasOnlyKeys(
        frequencyRange, "frequencyRangeHz", allowedRangeKeys, error) ||
      !frequencyRange["minimum"].is<float>() ||
      !frequencyRange["maximum"].is<float>()) {
    error = "frequencyRangeHz requires numeric minimum and maximum.";
    return false;
  }
  parsed.frequencyMinimum = frequencyRange["minimum"].as<float>();
  parsed.frequencyMaximum = frequencyRange["maximum"].as<float>();

  JsonObjectConst acquisition = root["acquisition"].as<JsonObjectConst>();
  static const char *const allowedAcquisitionKeys[] = {
    "minimumPollIntervalMs"
  };
  if (acquisition.isNull() ||
      !profileObjectHasOnlyKeys(
        acquisition, "acquisition", allowedAcquisitionKeys, error) ||
      !acquisition["minimumPollIntervalMs"].is<unsigned long>()) {
    error = "Schema 3 requires acquisition.minimumPollIntervalMs.";
    return false;
  }
  parsed.minimumPollIntervalMs =
    acquisition["minimumPollIntervalMs"].as<uint32_t>();
  syncRuntimeProfileFields(parsed);

  JsonObjectConst clock = root["vendorClock"].as<JsonObjectConst>();
  static const char *const allowedClockKeys[] = {
    "enabled", "startRegister", "registerCount"
  };
  if (clock.isNull() ||
      !profileObjectHasOnlyKeys(
        clock, "vendorClock", allowedClockKeys, error)) {
    return false;
  }
  if (clock.isNull() || !clock["enabled"].is<bool>()) {
    error = "vendorClock.enabled must be boolean.";
    return false;
  }
  parsed.supportsVendorClock = clock["enabled"].as<bool>();
  if (!parseProfileU16(clock["startRegister"], parsed.clockStartRegister) ||
      !clock["registerCount"].is<unsigned int>()) {
    error = "vendorClock requires startRegister and registerCount.";
    return false;
  }
  const uint32_t clockCount = clock["registerCount"].as<uint32_t>();
  if (clockCount > 65535U) {
    error = "vendorClock.registerCount exceeds 65535.";
    return false;
  }
  parsed.clockRegisterCount = (uint16_t)clockCount;

  JsonObjectConst fastFrame = root["bulkRead"].as<JsonObjectConst>();
  static const char *const allowedBulkKeys[] = {
    "enabled", "function", "startAddress", "registerCount"
  };
  if (fastFrame.isNull() ||
      !profileObjectHasOnlyKeys(
        fastFrame, "bulkRead", allowedBulkKeys, error)) {
    return false;
  }
  if (fastFrame.isNull() || !fastFrame["enabled"].is<bool>()) {
    error = "bulkRead.enabled must be boolean.";
    return false;
  }
  parsed.supportsContiguousFastFrame = fastFrame["enabled"].as<bool>();
  if (!parseProfileU16(
        fastFrame["startAddress"], parsed.fastFrameStartRegister) ||
      !fastFrame["registerCount"].is<unsigned int>()) {
    error = "bulkRead requires a start address and registerCount.";
    return false;
  }
  const uint32_t count = fastFrame["registerCount"].as<uint32_t>();
  if (count > 65535U) {
    error = "bulkRead.registerCount exceeds 65535.";
    return false;
  }
  parsed.fastFrameRegisterCount = (uint16_t)count;
  if (!fastFrame["function"].is<unsigned int>() ||
      fastFrame["function"].as<uint32_t>() > 255U) {
    error = "bulkRead requires a valid function.";
    return false;
  }
  parsed.bulkReadFunction = (uint8_t)fastFrame["function"].as<uint32_t>();

  if (!validateSensorProfile(parsed, error)) return false;
  profile = parsed;
  return true;
}

String profileFloatJson(float value) {
  char buffer[24];
  // Nine significant decimal digits are enough to round-trip any IEEE-754
  // binary32 value without silently changing a sensor scale on download/upload.
  snprintf(buffer, sizeof(buffer), "%.9g", (double)value);
  return String(buffer);
}

String profileDoubleJson(double value) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.17g", value);
  return String(buffer);
}

String profileU64Hex(uint64_t value) {
  char buffer[21];
  snprintf(buffer, sizeof(buffer), "0x%llX",
           (unsigned long long)value);
  return String(buffer);
}

String sensorProfileJson(const ModbusSensorProfile &profile) {
  String json;
  json.reserve(7000);
  json = "{\n";
  json += "  \"schemaVersion\": 3,\n";
  json += "  \"profileKey\": \"" + jsonEscape(profile.profileKey) + "\",\n";
  json += "  \"sensorType\": \"" + jsonEscape(profile.sensorType) + "\",\n";
  json += "  \"modelName\": \"" + jsonEscape(profile.modelName) + "\",\n";
  json += "  \"registerMapVersion\": \"" + jsonEscape(profile.registerMapVersion) + "\",\n";
  json += "  \"modbus\": {\n";
  json += "    \"mode\": \"rtu\",\n";
  json += "    \"baud\": " + String(profile.defaultBaud) + ",\n";
  json += "    \"slaveId\": " + String(profile.defaultSlaveId) + ",\n";
  json += "    \"serialFormat\": \"" + jsonEscape(profile.serialFormat) + "\",\n";
  json += "    \"addressConvention\": \"pdu_zero_based\",\n";
  json += "    \"defaultReadFunction\": " + String(profile.readFunction) + ",\n";
  json += "    \"maximumReadRegisters\": " +
          String(profile.maximumReadRegisters) + "\n";
  json += "  },\n";

  const auto appendField = [&](const ModbusFieldProfile &field,
                               const String &indent) {
    json += indent + "{\"enabled\": " +
            String(field.enabled ? "true" : "false");
    if (field.enabled) {
      json += ", \"function\": " + String(field.readFunction);
      json += ", \"address\": " + String(field.address);
      json += ", \"registerCount\": " + String(field.registerCount);
      json += ", \"dataType\": \"" +
              String(profileDataTypeName(field.dataType)) + "\"";
      json += ", \"byteOrder\": \"" +
              jsonEscape(field.wireByteOrder) + "\"";
      json += ", \"scale\": " + profileDoubleJson(field.scale);
      json += ", \"offset\": " + profileDoubleJson(field.offset);
      if (field.hasBitField) {
        json += ", \"bitField\": {\"mask\": \"" +
                profileU64Hex(field.bitMask) + "\", \"shift\": " +
                String(field.bitShift) + ", \"signed\": " +
                String(field.bitFieldSigned ? "true" : "false") + "}";
      }
      if (field.hasInvalidRaw) {
        json += ", \"invalidRaw\": \"" +
                profileU64Hex(field.invalidRaw) + "\"";
      }
      if (field.hasValidRange) {
        json += ", \"validRange\": {\"minimum\": " +
                profileDoubleJson(field.validMinimum) +
                ", \"maximum\": " +
                profileDoubleJson(field.validMaximum) + "}";
      }
    }
    json += "}";
  };

  const auto appendVector = [&](const char *name, const char *unit,
                                const char *semantics,
                                const ModbusFieldProfile fields[3],
                                bool trailingComma) {
    json += "    \"" + String(name) + "\": {\n";
    json += "      \"unit\": \"" + String(unit) + "\",\n";
    json += "      \"semantics\": \"" + String(semantics) + "\",\n";
    const char *axes[] = {"x", "y", "z"};
    for (uint8_t axis = 0; axis < 3; axis++) {
      json += "      \"" + String(axes[axis]) + "\": ";
      appendField(fields[axis], "");
      json += axis < 2 ? ",\n" : "\n";
    }
    json += "    }";
    json += trailingComma ? ",\n" : "\n";
  };

  json += "  \"measurements\": {\n";
  appendVector(
    "acceleration", "m/s^2",
    profile.accelerationIsInstantaneousWaveform
      ? "instantaneous_waveform" : "sensor_reported_magnitude",
    profile.accelerationFields, true);
  appendVector(
    "velocity", "mm/s",
    profile.velocityIsRms
      ? "sensor_reported_rms" : "sensor_reported_unspecified",
    profile.velocityFields, true);
  appendVector("displacement", "um", "sensor_reported",
               profile.displacementFields, true);
  appendVector("frequency", "Hz", "sensor_reported",
               profile.frequencyFields, true);
  json += "    \"temperature\": {\n";
  json += "      \"unit\": \"C\",\n";
  json += "      \"value\": ";
  appendField(profile.temperatureField, "");
  json += "\n    }\n";
  json += "  },\n";
  json += "  \"frequencyRangeHz\": {\"minimum\": " +
          profileFloatJson(profile.frequencyMinimum) + ", \"maximum\": " +
          profileFloatJson(profile.frequencyMaximum) + "},\n";
  json += "  \"acquisition\": {\"minimumPollIntervalMs\": " +
          String(profile.minimumPollIntervalMs) + "},\n";
  json += "  \"vendorClock\": {\"enabled\": " +
          String(profile.supportsVendorClock ? "true" : "false") +
          ", \"startRegister\": " + String(profile.clockStartRegister) +
          ", \"registerCount\": " + String(profile.clockRegisterCount) + "},\n";
  json += "  \"bulkRead\": {\"enabled\": " +
          String(profile.supportsContiguousFastFrame ? "true" : "false") +
          ", \"function\": " + String(profile.bulkReadFunction) +
          ", \"startAddress\": " + String(profile.fastFrameStartRegister) +
          ", \"registerCount\": " +
          String(profile.fastFrameRegisterCount) + "}\n";
  json += "}\n";
  return json;
}

String sanitizeIdentifier(const String &value) {
  String out;
  out.reserve(value.length());
  for (size_t i = 0; i < value.length(); i++) {
    char c = value.charAt(i);
    bool ok = (c >= 'A' && c <= 'Z') ||
              (c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') ||
              c == '-' || c == '_' || c == '.';
    if (ok) {
      out += c;
    } else if (c == ' ' || c == '/' || c == '\\') {
      out += '-';
    }
  }
  out.trim();
  while (out.indexOf("--") >= 0) {
    out.replace("--", "-");
  }
  if (out.length() > 48) {
    out = out.substring(0, 48);
  }
  return out;
}

String buildSetupApSsid() {
  if (setupApSsid.length() >= 1 && setupApSsid.length() <= 32) {
    return setupApSsid;
  }
  return deviceId;
}

bool setupApClientPresent() {
  return setupApStarted && WiFi.softAPgetStationNum() > 0;
}

// [WIFI AP] [HIGH RISK] STA connection attempts pause while a setup-AP client
// is present, but only for a bounded grace window - not for as long as that
// device happens to stay associated. A technician's phone or laptop commonly
// remains joined to the setup AP long after the setup page was actually used
// (no explicit "forget network", and the captive-portal auto-popup added
// earlier makes OS WiFi managers more likely to keep the association alive in
// the background). Gating STA on WiFi.softAPgetStationNum() > 0 unconditionally
// therefore held STA/MQTT offline indefinitely in the field - this was the
// root cause of "standard WiFi never connects" reports even with fully
// correct saved credentials. The grace window still protects the one thing
// this guard exists for (not switching the shared AP/STA channel out from
// under an in-flight setup HTTP request/response) without blocking real
// provisioning forever. This does not disconnect or otherwise touch the AP
// client - it only decides when serviceWiFi() may proceed with STA.
const unsigned long SETUP_AP_STA_PAUSE_GRACE_MS = 20000;

bool setupApBlockingStaAssociation() {
  if (!setupApClientPresent()) {
    setupApClientJoinedMs = 0;
    return false;
  }
  if (setupApClientJoinedMs == 0) {
    setupApClientJoinedMs = millis();
  }
  return millis() - setupApClientJoinedMs < SETUP_AP_STA_PAUSE_GRACE_MS;
}

bool requireSetupApRequest() {
  WiFiClient client = server.client();
  if (client && client.localIP() == setupApIp) {
    return true;
  }
  server.send(403, "text/plain; charset=UTF-8",
              "This setup endpoint is available only through the IVSS access point.");
  return false;
}

// [WIFI AP] Translate the radio's numeric disconnect reason into the actual
// field diagnosis. These map 1:1 onto wifi_err_reason_t; the wording is chosen
// so a technician reading the serial log or the portal knows what to fix
// without needing the 802.11 spec.
const char *staDisconnectReasonText(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:
      return "SSID not found on air - check spelling, or the AP is 5 GHz-only "
             "or out of range (this controller supports 2.4 GHz only)";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_AUTH_EXPIRE:
      return "authentication rejected by the AP - wrong password or wrong security type";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "WPA handshake timed out - almost always an incorrect WiFi password";
    case WIFI_REASON_802_1X_AUTH_FAILED:
      return "802.1X/Enterprise account rejected - check Enterprise username and password";
    case WIFI_REASON_ASSOC_FAIL:
      return "association refused by the AP";
    case WIFI_REASON_ASSOC_TOOMANY:
      return "AP refused: too many clients already connected";
    case WIFI_REASON_MIC_FAILURE:
      return "encryption MIC failure - wrong password or a mismatched cipher";
    case WIFI_REASON_BEACON_TIMEOUT:
      return "lost the AP's beacon - weak signal or the AP went away";
    case WIFI_REASON_CONNECTION_FAIL:
      return "the AP rejected the connection";
    case WIFI_REASON_AUTH_LEAVE:
      return "the AP deauthenticated this device";
    case WIFI_REASON_ASSOC_LEAVE:
      return "this device left the AP (normally a firmware-initiated disconnect)";
    case 0:
      return "none";
    default:
      return "see wifi_err_reason_t for this code";
  }
}

// [WIFI AP] [HIGH RISK] Runs in the WiFi/event task, not loop(). It only stores
// scalars and prints; it must never call WiFi.begin()/disconnect() or touch the
// Modbus/MQTT state machines, or it would race the cooperative loop() ordering.
void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi] Associated with the AP; waiting for an IP address.");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      staEverGotIp = true;
      lastStaDisconnectReason = 0;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      const uint8_t reason = info.wifi_sta_disconnected.reason;
      lastStaDisconnectReason = reason;
      staDisconnectCount++;
      Serial.printf("[WiFi] STA disconnect reason %u: %s\n",
                    (unsigned)reason, staDisconnectReasonText(reason));
      break;
    }
    default:
      break;
  }
}

void applyWifiPowerPolicy() {
  // [WIFI AP] [HIGH RISK] Preserve the product's explicit 11 dBm radio policy.
  // AP and STA share this setting, so site commissioning must still provide
  // adequate 2.4 GHz coverage. The cap reduces RF peak demand; bounded STA/MQTT
  // retries and the recovery AP handle temporary outages without a tight loop.
  // Modem sleep applies to STA idle periods; SoftAP beaconing and connected-
  // client traffic continue normally.
  // The Arduino core returns false when setSleep() is a no-op because the
  // requested state is already active. Verify the resulting state instead of
  // treating that return value as an error.
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  const bool sleepConfigured =
    WiFi.getSleep() == WIFI_PS_MIN_MODEM;
  const bool txPowerSet =
    WiFi.setTxPower(WIFI_POWER_11dBm) ||
    WiFi.getTxPower() == WIFI_POWER_11dBm;
  if (!sleepConfigured || !txPowerSet) {
    Serial.printf("[Power] WiFi policy incomplete: sleep=%d txPower=%d.\n",
                  sleepConfigured ? 1 : 0, txPowerSet ? 1 : 0);
  }
}

void startSetupPortal() {
  // [WIFI AP] [HIGH RISK] This is the authoritative SoftAP start path. Preserve
  // AP+STA mode, address configuration, channel, visibility, and call order.
  // Do not run the STA state machine when no router has been configured.
  WiFi.mode(wifiSsid.length() > 0 ? WIFI_AP_STA : WIFI_AP);
  applyWifiPowerPolicy();
  // Every explicit boot/button/serial activation receives a fresh window. The
  // countdown is armed only after a valid sensor document reaches MQTT.
  setupApExpiryArmed = false;
  setupApHealthyPublishMs = 0;
  String networkHostname = "IVSS-" + sanitizeIdentifier(deviceId);
  networkHostname.toLowerCase();
  if (networkHostname.length() > 32) {
    networkHostname = networkHostname.substring(0, 32);
  }
  const bool staHostnameSet = WiFi.setHostname(networkHostname.c_str());
  const bool apHostnameSet = WiFi.softAPsetHostname(networkHostname.c_str());
  networkIdentityHealthy = staHostnameSet && apHostnameSet;
  if (!networkIdentityHealthy) {
    // Hostnames are a convenience only. A failed hostname must never prevent
    // the radio, provisioning AP, or a valid STA configuration from starting.
    Serial.println("[Network] One or more interface hostnames could not be set; IP networking will continue.");
  }

  if (!setupApStarted) {
    setupApSsid = buildSetupApSsid();
    if (setupApPass.length() == 0) {
      setupApPass = SETUP_PIN;
    }
    if (setupApPass.length() > 0 && setupApPass.length() < 8) {
      setupApPass = SETUP_PIN;
    }

    IPAddress apGw(192, 168, 4, 1);
    IPAddress apMask(255, 255, 255, 0);
    if (!WiFi.softAPConfig(setupApIp, apGw, apMask)) {
      Serial.println("[AP] Failed to configure the required 192.168.4.1 address.");
      return;
    }
    setupApStarted = WiFi.softAP(setupApSsid.c_str(), setupApPass.c_str(), 1, 0, 1);
    if (setupApStarted) {
      // Reapply after SoftAP start so TX-power verification cannot race the
      // asynchronous interface-start event on a cold boot.
      applyWifiPowerPolicy();
    }
    if (setupApStarted && WiFi.softAPIP() != setupApIp) {
      Serial.println("[AP] Address verification failed; refusing to expose setup on an unexpected address.");
      WiFi.softAPdisconnect(true);
      setupApStarted = false;
    }

    if (setupApStarted) {
      Serial.println("[AP] Started setup AP.");
      Serial.println("[AP] SSID: " + setupApSsid);
      Serial.println("[AP] Password is not printed; use the provisioned product credential.");
      Serial.println("[AP] IP: " + WiFi.softAPIP().toString());
      Serial.println("[AP] Open: http://192.168.4.1 or http://sensor.local");
    } else {
      Serial.println("[AP] Failed to start setup AP. STA/MQTT/acquisition will continue; recovery retries are rate-limited to once per minute.");
    }
  }

  if (!setupApStarted) return;

  if (!setupApDnsStarted) {
    // [WIFI AP] Wildcard captive-portal DNS: every hostname a connected phone
    // or laptop looks up (including the OS's own "is this a captive portal?"
    // probe domains) resolves straight back to the AP. Combined with the
    // catch-all redirect in handleNotFound(), this is what makes iOS/Android/
    // Windows pop the setup page open automatically after joining the network,
    // instead of the user having to know to type an address.
    setupApDnsStarted = dnsServer.start(53, "*", setupApIp);
    if (!setupApDnsStarted) {
      Serial.println("[AP] Captive-portal DNS failed to start; the setup page still works at the address above.");
    }
  }
  if (!setupApMdnsStarted) {
    // "sensor.local" is safe to reuse across every unit because only one
    // device's own isolated setup AP is ever active on a given phone/laptop
    // at a time - there is no shared network for the name to collide on.
    setupApMdnsStarted = MDNS.begin("sensor");
    if (!setupApMdnsStarted) {
      Serial.println("[AP] sensor.local could not start; 192.168.4.1 still works.");
    }
  }

  if (!webServerStarted) {
    if (!webServerRoutesConfigured) {
      server.on("/",       HTTP_GET,  handleConfig);
      server.on("/config", HTTP_GET,  handleConfig);
      server.on("/status", HTTP_GET,  handleStatus);
      server.on("/save",   HTTP_POST, handleSave);
      server.on("/assets/jetbrains-mono.woff2", HTTP_GET, handleJetBrainsMonoFont);
      server.on("/sensor-profile.json", HTTP_GET, handleProfileDownload);
      // The API route is used by the setup page. Legacy and trailing-slash
      // aliases keep bookmarks and older cached setup pages from reaching a 404.
      server.on("/api/sensor-profile", HTTP_POST,
                handleProfileUploadComplete, handleProfileUploadStream);
      server.on("/sensor-profile", HTTP_POST,
                handleProfileUploadComplete, handleProfileUploadStream);
      server.on("/sensor-profile/", HTTP_POST,
                handleProfileUploadComplete, handleProfileUploadStream);
      server.on("/sensor-profile", HTTP_GET, handleConfig);
      server.on("/sensor-profile/", HTTP_GET, handleConfig);
      server.onNotFound(handleNotFound);
      webServerRoutesConfigured = true;
    }
    server.begin();
    webServerStarted = true;
    Serial.println("[Web] Setup server started on http://192.168.4.1 or http://sensor.local");
  }
}

void activateRecoverySetupPortal(const char *reason) {
  // [WIFI AP] [MQTT] [HIGH RISK] Recovery AP activation is idempotent and
  // bounded. A server outage must make local diagnostics reachable, but a
  // failed softAP start must not become a tight radio/CPU retry loop.
  const unsigned long now = millis();

  if (setupApStarted) {
    // A previously armed post-success expiry must not close the AP while the
    // MQTT/Node path is unhealthy.
    setupApExpiryArmed = false;
    setupApHealthyPublishMs = 0;
    if (!recoverySetupApLatched) {
      recoverySetupApLatched = true;
      Serial.print("[Recovery] Setup AP kept open: ");
      Serial.println(reason);
    }
    return;
  }

  if (recoverySetupApAttempted &&
      now - lastRecoverySetupApAttemptMs < RECOVERY_AP_RETRY_INTERVAL_MS) {
    return;
  }

  recoverySetupApAttempted = true;
  lastRecoverySetupApAttemptMs = now;
  startSetupPortal();
  if (setupApStarted) {
    recoverySetupApLatched = true;
    Serial.print("[Recovery] Setup AP reopened: ");
    Serial.println(reason);
  } else {
    Serial.println("[Recovery] Setup AP reopen failed; retrying in 60 seconds.");
  }
}

void armSetupApLifetimeAfterHealthyPublish() {
  recoverySetupApLatched = false;
  if (!setupApStarted || setupApExpiryArmed) return;
  setupApHealthyPublishMs = millis();
  setupApExpiryArmed = true;
  Serial.println("[AP] Valid sensor telemetry published; setup AP will close in 10 minutes.");
}

void serviceSetupApLifetime() {
  if (!setupApStarted || !setupApExpiryArmed ||
      millis() - setupApHealthyPublishMs < SETUP_AP_HEALTHY_WINDOW_MS) {
    return;
  }

  if (webServerStarted) {
    server.stop();
    webServerStarted = false;
  }
  if (setupApDnsStarted) {
    dnsServer.stop();
    setupApDnsStarted = false;
  }
  if (setupApMdnsStarted) {
    MDNS.end();
    setupApMdnsStarted = false;
  }
  WiFi.softAPdisconnect(true);
  setupApStarted = false;
  setupApExpiryArmed = false;
  setupApHealthyPublishMs = 0;
  recoverySetupApAttempted = false;
  lastRecoverySetupApAttemptMs = 0;
  WiFi.mode(wifiSsid.length() > 0 ? WIFI_STA : WIFI_OFF);
  if (wifiSsid.length() > 0) applyWifiPowerPolicy();
  Serial.println("[AP] Commissioning window expired; setup AP stopped. A power cycle, setup trigger, or future network/server outage will reopen it.");
}

bool nodeServerHeartbeatFresh(unsigned long now) {
  return mqttClient.connected() && nodeServerHeartbeatSeen &&
         now - lastNodeServerHeartbeatMs < NODE_SERVER_HEARTBEAT_TIMEOUT_MS;
}

void mqttMessageReceived(char *topic, uint8_t *payload, unsigned int length) {
  (void)payload;
  (void)length;
  if (topic == nullptr || strcmp(topic, NODE_SERVER_HEARTBEAT_TOPIC) != 0) {
    return;
  }

  const bool wasUnavailable = !nodeServerHeartbeatSeen || nodeServerHeartbeatTimedOut;
  lastNodeServerHeartbeatMs = millis();
  nodeServerHeartbeatSeen = true;
  nodeServerHeartbeatTimedOut = false;
  if (wasUnavailable) {
    // Give the restored service a full telemetry-progress window. Otherwise a
    // seven-hour outage leaves the old timestamp expired and serviceMqtt()
    // immediately tears down an otherwise healthy recovered connection.
    mqttSessionProgressMs = lastNodeServerHeartbeatMs;
    // The loop re-primes waveform/timing state on the actual end-to-end-ready
    // edge. That remains correct even if this heartbeat arrives while a setup
    // AP client is still inside its bounded 20-second radio grace.
    Serial.println("[Server] Node.js heartbeat received; normal telemetry can resume.");
  }
}

void announceSetupPortal(const char *reason) {
  if (!setupApStarted) startSetupPortal();
  if (!setupApStarted) {
    Serial.println("[Setup] Setup portal is unavailable; a restart will retry initialization.");
    return;
  }
  Serial.print("[Setup] ");
  Serial.print(reason);
  Serial.println(". Setup portal is already active in AP+STA mode.");
  Serial.println("[Setup] SSID: " + setupApSsid + " | URL: http://192.168.4.1 or http://sensor.local");
}

void beginStaConnection(const char *reason) {
  // [WIFI AP] [HIGH RISK] STA reconnects share the SoftAP radio. Do not add AP
  // client-driven disconnect/reconnect loops here.
  if (wifiSsid.length() == 0) {
    staConnectInProgress = false;
    Serial.println("[WiFi] STA not configured. Setup AP remains available.");
    return;
  }
  const bool wantEnterprise = wifiEnterpriseUser.length() > 0;
  if (wantEnterprise && wifiPass.length() == 0) {
    staConnectInProgress = false;
    Serial.println("[WiFi] Enterprise username is set but the password is blank. STA held offline.");
    return;
  }
  // A WPA2-PSK passphrase is 8-63 characters by standard. A shorter non-blank
  // password can never associate, so say so once instead of retrying forever.
  if (!wantEnterprise && wifiPass.length() > 0 && wifiPass.length() < 8) {
    staConnectInProgress = false;
    Serial.println("[WiFi] Saved WiFi password is shorter than the 8-character WPA2 "
                   "minimum. STA held offline; re-enter it in the setup page.");
    return;
  }
  wl_status_t beginResult = WL_IDLE_STATUS;

  WiFi.mode(setupApStarted ? WIFI_AP_STA : WIFI_STA);
  applyWifiPowerPolicy();
  WiFi.disconnect(false, false); // Disconnect STA only; do not erase saved credentials or stop AP.

  if (wantEnterprise) {
    // WPA2-Enterprise (802.1X, PEAP/MSCHAPv2) - the method used by the large
    // majority of corporate and campus networks that ask for a username. No
    // CA certificate is configured, so the RADIUS server's certificate is not
    // validated; that matches how most ESP32 deployments join these networks,
    // but it is weaker than pinning a certificate.
    // Fully replace any previous supplicant state before installing the new
    // PEAP credentials. This matters when switching between PSK and 802.1X or
    // after a failed enterprise attempt.
    esp_wifi_sta_wpa2_ent_disable();
    esp_wifi_sta_wpa2_ent_clear_identity();
    esp_wifi_sta_wpa2_ent_clear_username();
    esp_wifi_sta_wpa2_ent_clear_password();
    esp_wifi_sta_wpa2_ent_clear_ca_cert();
    const esp_err_t identityResult = esp_wifi_sta_wpa2_ent_set_identity(
      (const unsigned char *)wifiEnterpriseUser.c_str(), wifiEnterpriseUser.length());
    const esp_err_t usernameResult = esp_wifi_sta_wpa2_ent_set_username(
      (const unsigned char *)wifiEnterpriseUser.c_str(), wifiEnterpriseUser.length());
    const esp_err_t passwordResult = esp_wifi_sta_wpa2_ent_set_password(
      (const unsigned char *)wifiPass.c_str(), wifiPass.length());
    const esp_err_t enableResult = esp_wifi_sta_wpa2_ent_enable();
    if (identityResult != ESP_OK || usernameResult != ESP_OK ||
        passwordResult != ESP_OK || enableResult != ESP_OK) {
      esp_wifi_sta_wpa2_ent_disable();
      wifiEnterpriseActive = false;
      staConnectInProgress = false;
      lastWifiAttemptMs = millis();
      Serial.printf("[WiFi] Enterprise credential setup failed: identity=%d username=%d password=%d enable=%d.\n",
                    (int)identityResult, (int)usernameResult,
                    (int)passwordResult, (int)enableResult);
      return;
    }
    WiFi.setMinSecurity(WIFI_AUTH_WPA2_ENTERPRISE);
    wifiEnterpriseActive = true;
    beginResult = WiFi.begin(wifiSsid.c_str());
  } else {
    // Clear 802.1X state unconditionally, not only when this firmware believes
    // it previously enabled it. The supplicant's enterprise flag lives outside
    // our own bookkeeping, so any desync between wifiEnterpriseActive and the
    // driver would leave a plain WPA2-PSK network being attacked with PEAP and
    // failing forever. These calls are idempotent and cost nothing when 802.1X
    // was never on, which makes the unconditional form strictly safer.
    esp_wifi_sta_wpa2_ent_disable();
    esp_wifi_sta_wpa2_ent_clear_identity();
    esp_wifi_sta_wpa2_ent_clear_username();
    esp_wifi_sta_wpa2_ent_clear_password();
    esp_wifi_sta_wpa2_ent_clear_ca_cert();
    wifiEnterpriseActive = false;
    // Plant networks are often still WPA-only or WPA/WPA2 mixed. The Arduino
    // core otherwise filters association candidates at WPA2-PSK and silently
    // refuses to associate with a legacy AP that the scan list happily shows.
    // An empty password means an intentionally open network. Do not silently
    // filter it out with a WPA-only authentication threshold.
    WiFi.setMinSecurity(wifiPass.length() == 0 ? WIFI_AUTH_OPEN
                                               : WIFI_AUTH_WPA_PSK);
    beginResult = WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  }
  lastWifiAttemptMs = millis();
  staConnectStartedMs = lastWifiAttemptMs;
  staConnectInProgress = true;
  Serial.println("[WiFi] STA connecting to '" + wifiSsid + "' (" + String(reason) +
                  (wantEnterprise ? ", WPA2-Enterprise" : ", WPA/WPA2-PSK") + ").");
  // WiFi.begin() rejects some configurations outright (SSID/passphrase length,
  // STA enable failure, driver config/connect failure). Without checking this
  // return value a hard, immediate refusal was indistinguishable from a normal
  // association in progress, and only surfaced 10 s later as a generic timeout.
  if (beginResult == WL_CONNECT_FAILED) {
    staConnectInProgress = false;
    Serial.println("[WiFi] ERROR: the WiFi driver refused this connection request "
                   "outright (check SSID length 1-32 and password length; a WPA2 "
                   "password must be 8-63 characters).");
  }
}

void serviceWiFi() {
  // [WIFI AP] [HIGH RISK] Called continuously from loop(); retries and scans
  // must remain bounded so SoftAP and sensor acquisition remain responsive.
  unsigned long now = millis();
  wl_status_t status = WiFi.status();

  if (status != lastStaStatus) {
    if (status == WL_CONNECTED) {
      staConnectInProgress = false;
      wifiRetryDelayMs = WIFI_RETRY_INITIAL_MS;
      Serial.println("[WiFi] STA connected.");
      Serial.println("[WiFi] STA IP: " + WiFi.localIP().toString());
      Serial.println("[WiFi] RSSI: " + String(WiFi.RSSI()) + " dBm");
    } else if (lastStaStatus == WL_CONNECTED) {
      Serial.println("[WiFi] STA disconnected. AP remains available; retrying without stopping acquisition.");
      if (mqttClient.connected()) {
        mqttClient.disconnect();
      }
    }
    lastStaStatus = status;
  }

  // A configuration response must finish over SoftAP before changing the
  // shared radio channel. Once that client leaves, apply the saved credentials
  // even if the old STA connection still reports WL_CONNECTED.
  if (staSettingsApplyPending) {
    if (setupApBlockingStaAssociation()) return;
    staSettingsApplyPending = false;
    wifiRetryDelayMs = WIFI_RETRY_INITIAL_MS;
    beginStaConnection("saved settings");
    return;
  }

  if (wifiSsid.length() == 0 || status == WL_CONNECTED) {
    return;
  }

  // Do not start or cancel STA association attempts while the setup operator
  // is actively using SoftAP. AP and STA share one radio; this pause avoids
  // channel churn without disconnecting either interface or changing AP mode.
  // Bounded by setupApBlockingStaAssociation() so a client that merely stays
  // associated (without actively using the page) cannot hold STA offline
  // forever - see that function's comment for why this matters.
  if (setupApBlockingStaAssociation()) {
    // Do not launch another attempt, but never abort an authentication
    // handshake that is already running. Enterprise exchanges in particular
    // can take several seconds and were previously cancelled here.
    return;
  }

  const unsigned long staAttemptTimeoutMs =
    wifiEnterpriseActive ? WIFI_ENTERPRISE_CONNECT_ATTEMPT_TIMEOUT_MS
                         : WIFI_CONNECT_ATTEMPT_TIMEOUT_MS;
  if (staConnectInProgress && now - staConnectStartedMs >= staAttemptTimeoutMs) {
    staConnectInProgress = false;
    WiFi.disconnect(false, false);
    // Start the retry cooldown at the end of the failed attempt. Previously the
    // timer remained anchored to its start, causing an immediate new attempt
    // and almost continuous active scanning/association.
    lastWifiAttemptMs = now;
    wifiRetryDelayMs = min(wifiRetryDelayMs * 2UL, WIFI_RETRY_MAX_MS);
    // Always report the radio's own reason for the failure. This is the single
    // most useful line in the log when commissioning does not come up.
    Serial.printf("[WiFi] STA connect attempt timed out after %lu ms. Last radio "
                  "reason %u: %s. Next retry in %lu ms; AP remains available.\n",
                  staAttemptTimeoutMs, (unsigned)lastStaDisconnectReason,
                  staDisconnectReasonText(lastStaDisconnectReason),
                  wifiRetryDelayMs);
    activateRecoverySetupPortal("STA WiFi connection failed");
  }

  if (!staConnectInProgress && now - lastWifiAttemptMs >= wifiRetryDelayMs) {
    beginStaConnection("periodic retry");
  }
}

void serviceMqtt() {
  // [MQTT] Broker servicing is deliberately gated by STA availability so
  // acquisition and the setup AP survive broker/network outages.
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED || mqttBroker.length() == 0) {
    if (mqttClient.connected()) {
      mqttClient.disconnect();
    }
    if (lastMqttConnected) {
      Serial.println("[MQTT] Disconnected because STA WiFi or broker config is unavailable.");
      lastMqttConnected = false;
    }
    if (mqttDisconnectedSinceMs == 0) mqttDisconnectedSinceMs = now;
    mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
    mqttConnectedSinceMs = 0;
    nodeServerHeartbeatSeen = false;
    nodeServerHeartbeatTimedOut = false;
    if (now - mqttDisconnectedSinceMs >= MQTT_OUTAGE_AP_TRIGGER_MS) {
      activateRecoverySetupPortal("STA WiFi or MQTT configuration is unavailable");
    }
    return;
  }

  // Use the same bounded setup grace as STA association. Merely leaving a phone
  // associated with the setup AP must never suppress MQTT recovery forever.
  // No WiFi disconnect or mode transition is performed here.
  if (setupApBlockingStaAssociation()) {
    if (mqttClient.connected()) mqttClient.loop();
    return;
  }

  if (mqttClient.connected()) {
    if (!lastMqttConnected) {
      Serial.println("[MQTT] Connected.");
      lastMqttConnected = true;
      mqttSessionProgressMs = now;
      if (mqttConnectedSinceMs == 0) mqttConnectedSinceMs = now;
    }
    // Service keepalive and incoming Node heartbeat before making any health
    // decision. PubSubClient returns false when its TCP/MQTT session has died.
    if (!mqttClient.loop()) {
      Serial.println("[MQTT] Session keepalive failed; rebuilding the connection.");
      mqttClient.disconnect();
      lastMqttConnected = false;
      mqttDisconnectedSinceMs = now;
      lastMqttAttemptMs = now;
      mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
      mqttConnectedSinceMs = 0;
      nodeServerHeartbeatSeen = false;
      nodeServerHeartbeatTimedOut = false;
      activateRecoverySetupPortal("MQTT broker session failed");
      return;
    }

    // The callback above records millis() when a heartbeat arrives. Refresh the
    // local clock so unsigned elapsed-time arithmetic cannot see that new stamp
    // as being one millisecond "in the future".
    now = millis();
    const bool serverReady = nodeServerHeartbeatFresh(now);
    const bool heartbeatWaitExpired = mqttConnectedSinceMs != 0 &&
      now - (nodeServerHeartbeatSeen ? lastNodeServerHeartbeatMs
                                    : mqttConnectedSinceMs) >=
        NODE_SERVER_HEARTBEAT_TIMEOUT_MS;
    if (!serverReady && heartbeatWaitExpired) {
      if (!nodeServerHeartbeatTimedOut) {
        nodeServerHeartbeatTimedOut = true;
        Serial.println("[Server] Node.js heartbeat missing for 45 s; telemetry slowed and recovery AP enabled.");
      }
      activateRecoverySetupPortal("Node.js dashboard server is unavailable");
      // A successful subscribe() only proves that the request bytes were sent;
      // an ACL can still reject it. Retry slowly while the heartbeat is absent
      // so corrected broker permissions recover without a power cycle.
      if (now - lastNodeHeartbeatSubscribeAttemptMs >=
          NODE_SERVER_HEARTBEAT_RESUBSCRIBE_MS) {
        lastNodeHeartbeatSubscribeAttemptMs = now;
        if (!mqttClient.subscribe(NODE_SERVER_HEARTBEAT_TOPIC)) {
          Serial.println("[MQTT] Heartbeat resubscribe write failed; rebuilding the session.");
          mqttClient.disconnect();
          lastMqttConnected = false;
          mqttDisconnectedSinceMs = now;
          lastMqttAttemptMs = now;
          mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
          mqttConnectedSinceMs = 0;
          nodeServerHeartbeatSeen = false;
          nodeServerHeartbeatTimedOut = false;
          return;
        }
      }
    }
    // A live MQTT session must make observable forward progress. This catches a
    // half-open connection left behind by a sleeping broker computer. Normal
    // sensor-fault documents also count as progress, so this watchdog does not
    // confuse a physical sensor fault with a transport fault.
    if (serverReady && !setupApBlockingStaAssociation() &&
        mqttSessionProgressMs != 0 &&
        now - mqttSessionProgressMs >= MQTT_SESSION_PROGRESS_TIMEOUT_MS) {
      Serial.println("[MQTT] No successful telemetry write for 90 s; rebuilding the stale session.");
      mqttClient.disconnect();
      lastMqttConnected = false;
      mqttDisconnectedSinceMs = now;
      lastMqttAttemptMs = now;
      mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
      mqttConnectedSinceMs = 0;
      nodeServerHeartbeatSeen = false;
      nodeServerHeartbeatTimedOut = false;
      activateRecoverySetupPortal("MQTT telemetry session stopped making progress");
      return;
    }
    mqttDisconnectedSinceMs = 0;
    mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
    return;
  }

  if (lastMqttConnected) {
    Serial.println("[MQTT] Disconnected. rc=" + String(mqttClient.state()));
    lastMqttConnected = false;
  }
  mqttConnectedSinceMs = 0;
  nodeServerHeartbeatSeen = false;
  nodeServerHeartbeatTimedOut = false;

  if (mqttDisconnectedSinceMs == 0) {
    mqttDisconnectedSinceMs = now;
  }

  if (now - lastMqttAttemptMs >= mqttRetryDelayMs) {
    lastMqttAttemptMs = now;
    mqttReconnect();
  }

  if (now - mqttDisconnectedSinceMs >= MQTT_DOWN_LOG_INTERVAL_MS) {
    static unsigned long lastMqttDownLogMs = 0;
    if (now - lastMqttDownLogMs >= MQTT_DOWN_LOG_INTERVAL_MS) {
      lastMqttDownLogMs = now;
      Serial.println("[MQTT] Broker still unavailable; acquisition and setup AP continue.");
    }
  }
}

// =============================================================================
// HTML helpers
// =============================================================================
static const char HTML_HEAD[] PROGMEM =
  "<!DOCTYPE html><html><head>"
  "<meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>IVSS Setup</title>"
  "<style>"
  "@font-face{font-family:'JetBrains Mono';src:url('/assets/jetbrains-mono.woff2') format('woff2');font-style:normal;font-weight:400;font-display:swap}"
  ":root{color-scheme:dark;--bg:#04080b;--panel:#0a141b;--titlebar:#0e1e29;--line:#1c313f;--line-strong:#2c4a5e;"
       "--text:#d8ecf7;--muted:#7994a8;--accent:#39c1ff;"
       "--control-bg:#0d5a82;--control-hover:#116d99;--control-active:#083f5c;"
       "--control-active-text:#fff;--ok:#3ddc84;--warn:#f0b429;--bad:#ff5c5c}"
  // Flat instrument-panel styling: solid fills only (no gradients, no drop
  // shadows), square corners throughout, and a solid-color title bar on the
  // header and on every .section so each configuration block reads as one
  // self-contained window - the same convention used on avionics/radar
  // multi-function displays. Sizing still keeps a >=32px control height and
  // 16px input font (stops iOS auto-zoom on focus) for real touch use.
  "*{box-sizing:border-box;border-radius:0}"
  "body{margin:0;min-height:100vh;background:var(--bg);color:var(--text);"
       "font-family:'JetBrains Mono','Cascadia Mono','SFMono-Regular',Consolas,monospace;"
       "letter-spacing:0;padding:20px 16px;font-size:15px;line-height:1.5}"
  ".shell{max-width:780px;margin:0 auto;border:1px solid var(--line-strong);background:var(--panel)}"
  ".top{padding:16px 20px;background:var(--titlebar);border-bottom:1px solid var(--line-strong);display:flex;justify-content:space-between;gap:12px;align-items:flex-start}"
  "h1{font-size:18px;line-height:1.3;margin:0 0 4px;color:#fff;font-weight:700;letter-spacing:.03em;text-transform:uppercase}"
  ".brand-word{font-family:'JetBrains Mono',monospace;font-weight:700}"
  ".sub{color:var(--muted);margin:0;font-size:12.5px}.lead{color:var(--muted);margin:0;padding:14px 20px 0;font-size:13px;line-height:1.6}"
  ".badge{border:1px solid var(--line-strong);background:#0a1822;color:var(--accent);padding:5px 10px;font-size:11px;letter-spacing:.05em;text-transform:uppercase;white-space:nowrap}"
  "form{padding:16px 20px}.section{border:1px solid var(--line-strong);background:var(--panel);margin-bottom:16px}"
  ".section h2{margin:0;padding:10px 16px;background:var(--titlebar);border-bottom:1px solid var(--line-strong);font-size:12.5px;font-weight:700;letter-spacing:.06em;text-transform:uppercase;color:var(--text)}"
  ".section h2::before{content:'';display:inline-block;width:7px;height:7px;background:var(--accent);margin-right:9px;vertical-align:middle}"
  ".section h2 small{display:block;margin:4px 0 0 16px;color:var(--muted);font-size:11.5px;font-weight:400;letter-spacing:0;text-transform:none;line-height:1.45}"
  ".grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:16px}.field{min-width:0}"
  "label{display:block;margin:0 0 6px;font-size:12px;font-weight:700;letter-spacing:.03em;text-transform:uppercase;color:var(--muted)}"
  "input{width:100%;padding:10px 11px;background:#050c12;color:var(--text);border:1px solid var(--line-strong);"
       "font:400 15px 'JetBrains Mono','Cascadia Mono',Consolas,monospace;outline:none}"
  "input[type=file]{padding:8px}input::file-selector-button{margin-right:10px;padding:7px 10px;border:1px solid var(--line-strong);background:#132635;color:var(--text);font-family:'JetBrains Mono',monospace;font-size:12.5px;font-weight:400;cursor:pointer}"
  "input:focus{border-color:var(--accent)}"
  "input:disabled{opacity:.45;cursor:not-allowed}"
  ".hint{margin:8px 0 0;color:var(--muted);font-size:12.5px;line-height:1.6}"
  ".check{display:flex;gap:10px;align-items:flex-start;min-height:36px;padding:6px 16px;color:var(--muted);font-size:13px;line-height:1.5}"
  ".check input{width:16px;height:16px;min-width:16px;margin-top:1px;accent-color:var(--accent)}"
  ".btn{width:100%;min-height:36px;padding:9px 14px;background:var(--control-bg);color:#fff;"
       "border:1px solid var(--line-strong);cursor:pointer;font-family:'JetBrains Mono',monospace;"
       "font-size:12.5px;font-weight:700;letter-spacing:.05em;text-transform:uppercase}"
  ".btn:hover{background:var(--control-hover)}"
  ".btn:active{background:var(--control-active);color:var(--control-active-text)}"
  ".btn:disabled{opacity:.55;cursor:wait}"
  ".err{color:var(--bad);margin:12px 0;font-weight:700}.ok{color:var(--ok);margin:12px 0;font-weight:700}"
  ".status{padding:16px;display:grid;grid-template-columns:1fr 1fr;gap:10px}.kv{border:1px solid var(--line);border-left:2px solid var(--accent);background:#081119;padding:10px 12px}"
  ".kv b{display:block;color:var(--muted);font-size:10.5px;text-transform:uppercase;letter-spacing:.07em;margin-bottom:5px}"
  ".kv span{font-size:13px;color:var(--text);word-break:break-word;display:inline-flex;align-items:center;gap:7px;line-height:1.5}"
  ".dot{width:8px;height:8px;min-width:8px;background:var(--muted)}.dot.ok{background:var(--ok)}.dot.warn{background:var(--warn)}.dot.bad{background:var(--bad)}"
  ".upload-state{display:none;margin:0 16px 16px;padding:10px 12px;border:1px solid var(--line-strong);color:var(--muted);font-size:13px;line-height:1.5}"
  ".upload-state.show{display:block}.upload-state.okay{color:var(--ok);border-color:#215c3d;background:#081b12}.upload-state.bad{color:var(--bad);border-color:#652f36;background:#1c0d10}"
  ".overlay{position:fixed;inset:0;background:rgba(2,6,9,.92);display:none;align-items:center;justify-content:center;z-index:20;padding:20px}"
  ".overlay.show{display:flex}.loader-card{width:min(360px,100%);background:var(--panel);border:1px solid var(--line-strong);padding:22px;text-align:center}"
  // A horizontal scan bar (a solid block sweeping a track) replaces the old
  // spinning ring - same "still working" signal, no circle, no gradient.
  ".spinner{width:100%;height:6px;background:#0b161f;border:1px solid var(--line-strong);position:relative;overflow:hidden;margin:0 0 16px}"
  ".spinner::after{content:'';position:absolute;top:0;left:-30%;width:30%;height:100%;background:var(--accent);animation:scan 1s linear infinite}"
  "@keyframes scan{0%{left:-30%}100%{left:100%}}"
  ".loader-title{font-weight:700;font-size:14px;margin-bottom:6px;letter-spacing:.04em;text-transform:uppercase}.loader-text{color:var(--muted);font-size:13px;line-height:1.5}.progress{height:5px;background:#0b161f;border:1px solid var(--line-strong);overflow:hidden;margin-top:16px}.progress span{display:block;width:8%;height:100%;background:var(--accent)}"
  "@media(max-width:620px){.top{display:block}.badge{display:inline-block;margin-top:10px}.grid{grid-template-columns:1fr}.status{grid-template-columns:1fr}.lead{padding:14px 0 0}}"
  "@media(max-width:400px){body{padding:14px 10px;font-size:14.5px}.top,form{padding-left:16px;padding-right:16px}.grid,.status{padding:12px}}"
  "</style></head><body>";

// =============================================================================
// Web handlers
// =============================================================================
// [WIFI AP] The handlers below expose setup/status through the protected AP.
// [SENSOR PROFILE] Profile upload handlers validate and persist field settings.
// [HIGH RISK] Saving configuration or a profile changes subsequent STA, MQTT,
// UART, register-map, and acquisition behaviour; preserve validation ordering.
void handleConfig() {
  if (!requireSetupApRequest()) return;
  bool staConnected = (WiFi.status() == WL_CONNECTED);
  String staStatus = wifiSsid.length() == 0 ? "not configured" : (staConnected ? "connected" : "disconnected/retrying");
  String staIp = staConnected ? WiFi.localIP().toString() : "-";
  String apIp = setupApStarted ? WiFi.softAPIP().toString() : "-";
  const bool mqttBrokerConnected = mqttClient.connected();
  const bool dashboardReady = nodeServerHeartbeatFresh(millis());
  String mqttStatus = mqttBroker.length() == 0
    ? "not configured"
    : !mqttBrokerConnected
      ? "broker disconnected/retrying"
      : dashboardReady
        ? "broker connected | dashboard online"
        : "broker connected | dashboard heartbeat missing";
  String rssiText = staConnected ? String(WiFi.RSSI()) + " dBm" : "-";

  String apDot = setupApStarted ? (WiFi.softAPgetStationNum() > 0 ? "ok" : "warn") : "bad";
  String staDot = staConnected ? "ok" : (wifiSsid.length() == 0 ? "warn" : "bad");
  String mqttDot = dashboardReady ? "ok" : (mqttBrokerConnected || mqttBroker.length() == 0 ? "warn" : "bad");

  String html = FPSTR(HTML_HEAD);
  html.reserve(20000);
  html += "<main class='shell'>";
  html += String("<div class='top'><div><h1><span class='brand-word'>IVSS</span> Setup</h1><p class='sub'>") + htmlEscape(FIRMWARE_NAME) + " v" + htmlEscape(FIRMWARE_VERSION) + " | Device: " + htmlEscape(deviceId) + "</p></div><span class='badge'>Setup online</span></div>";
  html += "<p class='lead'>Enter your WiFi and MQTT broker below and press Save. Everything else has a working default.</p>";
  html += "<form id='configForm' method='POST' action='/save'>";
  html += "<section class='section'><h2>Live Status</h2><div class='status'>";
  html += String("<div class='kv'><b>Sensor</b><span>") + htmlEscape(SENSOR_MODEL_NAME) + " | " + htmlEscape(DEVICE_PROTOCOL_NAME) + "</span></div>";
  html += String("<div class='kv'><b>Sensor Profile</b><span>") +
          htmlEscape(ACTIVE_SENSOR_PROFILE.profileKey) + " | " +
          htmlEscape(SENSOR_REGISTER_MAP_VERSION) + " | " +
          String(sensorProfileLoadedFromStorage ? "uploaded" : "built-in") +
          "</span></div>";
  html += String("<div class='kv'><b>Setup AP</b><span><i class='dot ") + apDot + "'></i>" + htmlEscape(setupApSsid) + " | " + apIp + " | clients " + String(WiFi.softAPgetStationNum()) + "</span></div>";
  html += String("<div class='kv'><b>STA WiFi</b><span><i class='dot ") + staDot + "'></i>" + htmlEscape(staStatus) + " | " + htmlEscape(wifiSsid) + " | " + staIp +
          (wifiEnterpriseUser.length() > 0 ? " | Enterprise (" + htmlEscape(wifiEnterpriseUser) + ")" : "") + "</span></div>";
  html += String("<div class='kv'><b>MQTT</b><span><i class='dot ") + mqttDot + "'></i>" + htmlEscape(mqttStatus) + " | " + htmlEscape(mqttBroker) + ":" + String(mqttPort) + "</span></div>";
  html += String("<div class='kv'><b>RSSI</b><span>") + htmlEscape(rssiText) + "</span></div>";
  // The radio's own reason for the last failed association is the single most
  // actionable thing a commissioning technician can see. Surfacing it here means
  // they do not need a serial cable to tell a wrong password from a wrong SSID.
  if (!staConnected && wifiSsid.length() > 0) {
    html += String("<div class='kv'><b>Last WiFi Failure</b><span><i class='dot ") +
            (lastStaDisconnectReason == 0 ? "warn" : "bad") + "'></i>" +
            (lastStaDisconnectReason == 0
               ? String("no association attempt has failed yet")
               : String("reason ") + String((unsigned)lastStaDisconnectReason) + " - " +
                 htmlEscape(staDisconnectReasonText(lastStaDisconnectReason))) +
            "</span></div>";
  }
  html += String("<div class='kv'><b>Acquisition</b><span>Target ") +
          String(sensorPollIntervalMs) + " ms | actual " +
          String(lastActualSensorPollIntervalMs) + " ms</span></div>";
  html += "</div></section>";
  html += "<section class='section'><h2>Network<small>Where the sensor connects and where it sends data</small></h2><div class='grid'>";
  html += "<div class='field'><label>WiFi SSID</label>"
          "<input id='ssid' name='ssid' value='" + htmlEscape(wifiSsid) + "' required></div>";
  html += "<div class='field'><label>WiFi Password</label>"
          "<input name='wpass' type='password' placeholder='Leave blank to keep current'>"
          "<p class='hint'>Leave blank to keep the saved password. Also used as the Enterprise account password below, if that is turned on.</p></div>";
  html += "<div class='field'><label>Enterprise Username</label>"
          "<input name='ent_user' id='entUser' maxlength='127' autocomplete='off' value='" + htmlEscape(wifiEnterpriseUser) +
          (wifiEnterpriseUser.length() == 0 ? "' disabled" : "'") +
          " placeholder='Only used if Enterprise is turned on below'>"
          "<p class='hint'>Ignored completely unless \"Use Enterprise authentication\" below is checked - a saved value here never takes effect on its own.</p></div>";
  html += "<div class='field'><label>MQTT Broker IP / Host</label>"
          "<input name='broker' value='" + htmlEscape(mqttBroker) + "' required></div>";
  html += "<div class='field'><label>MQTT Port</label>"
          "<input name='port' type='number' min='1' max='65535' value='" + String(mqttPort) + "' required></div>";
  html += "<div class='field'><label>MQTT Username</label>"
          "<input name='muser' value='" + htmlEscape(mqttUser) + "' placeholder='Optional'></div>";
  html += "<div class='field'><label>MQTT Password</label>"
          "<input name='mpass' type='password' placeholder='Leave blank to keep current'></div>";
  html += "</div><label class='check'><input id='entEnable' name='ent_enable' type='checkbox' value='1' onchange=\"document.getElementById('entUser').disabled=!this.checked\"" +
          String(wifiEnterpriseUser.length() > 0 ? " checked" : "") +
          ">Use Enterprise (802.1X / PEAP) authentication</label>"
          "<p class='hint' style='padding:0 16px 12px'>For a corporate/campus network that also asks for a username. Unchecked always means an ordinary network, regardless of what is saved above.</p>";
  html += "<label class='check'><input name='blank_wpass' type='checkbox' value='1'>Save WiFi password as blank (open network).</label>";
  html += "<label class='check'><input name='blank_mpass' type='checkbox' value='1'>Save MQTT password as blank.</label></section>";
  html += "<section class='section'><h2>Device<small>Identity, credentials, and read rate for this unit</small></h2><div class='grid'>";
  html += "<div class='field'><label>Serial / Device ID</label>"
          "<input value='" + htmlEscape(deviceId) + "' readonly></div>";
  html += "<div class='field'><label>Setup AP SSID</label>"
          "<input name='ap_ssid' minlength='1' maxlength='32' value='" +
          htmlEscape(setupApSsid) + "' required>"
          "<p class='hint'>Default: the full TR serial number. Does not change the Device ID.</p></div>";
  html += "<div class='field'><label>Machine Identification Name</label>"
          "<input name='identification_name' maxlength='64' value='" +
          htmlEscape(identificationName) +
          "' placeholder='Example: Cooling Pump 3'>"
          "<p class='hint'>Optional friendly name. The Device ID stays the same.</p></div>";
  html += "<div class='field'><label>Setup AP Password</label>"
          "<input name='ap_pass' type='password' minlength='8' maxlength='63' placeholder='Leave blank to keep current'>"
          "<p class='hint'>Default: the last 12 digits of the Device ID. 8-63 characters to change it.</p></div>";
  html += "<div class='field'><label>Acquisition Interval (ms)</label>"
          "<input name='poll_ms' type='number' min='" +
          String(ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs) +
          "' max='" + String(SENSOR_READ_MAX_INTERVAL_MS) +
          "' step='10' value='" + String(sensorPollIntervalMs) + "' required>"
          "<p class='hint'>" +
          String(1000.0f / (float)max(1UL, sensorPollIntervalMs), 2) +
          " readings/sec. Minimum for this profile: " +
          String(ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs) + " ms.</p></div>";
  html += "</div></section>";
  html += "<div id='configState' class='upload-state' role='status' aria-live='polite'></div>"
          "<button id='configButton' class='btn' type='submit'>Save &amp; Apply</button>"
          "</form>";
  html += "<form id='profileForm' method='POST' action='/sensor-profile' enctype='multipart/form-data'>"
          "<section class='section'><h2>Sensor Profile<small>Only needed for a non-default sensor model</small></h2><div class='grid'>"
          "<div class='field'><label>Profile JSON File</label>"
          "<input id='profileFile' name='profile' type='file' accept='.json,application/json' required>"
          "<p class='hint'>Schema 3 JSON, max 12 KB. Validated before it replaces the active profile.</p></div>"
          "<div class='field'><label>Current Profile</label><input id='currentProfile' value='" +
          htmlEscape(ACTIVE_SENSOR_PROFILE.profileKey) + "' readonly>"
          "<p class='hint'><a href='/sensor-profile.json' style='color:#39c1ff'>Download current JSON</a></p></div>"
          "</div><div id='profileState' class='upload-state' role='status' aria-live='polite'></div>"
          "<button id='profileButton' class='btn' type='submit'>Upload and Apply Sensor Profile</button>"
          "</section></form>";
  html += "<div id='busyOverlay' class='overlay' role='alert' aria-live='assertive'>"
          "<div class='loader-card'><div class='spinner'></div>"
          "<div id='busyTitle' class='loader-title'>Working...</div>"
          "<div id='busyText' class='loader-text'>Please keep this page open.</div>"
          "<div class='progress'><span id='busyProgress'></span></div></div></div>";
  html += "<script>"
          "async function getJson(u){const r=await fetch(u,{cache:'no-store'});let j;try{j=await r.json();}catch(_){throw new Error('The device returned an unreadable response.');}if(!r.ok)throw new Error(j.error||('Request failed with status '+r.status+'.'));return j;}"
          "const overlay=document.getElementById('busyOverlay'),busyTitle=document.getElementById('busyTitle'),busyText=document.getElementById('busyText'),busyProgress=document.getElementById('busyProgress');"
          "function showBusy(title,text,percent){busyTitle.textContent=title;busyText.textContent=text;busyProgress.style.width=(percent||8)+'%';overlay.classList.add('show');}"
          "function hideBusy(){overlay.classList.remove('show');}"
          "async function verifyUploadedProfile(expectedKey){if(!expectedKey)return false;for(let i=0;i<12;i++){try{const j=await getJson('/status');if(j.sensorProfile===expectedKey)return true;}catch(_){}await new Promise(r=>setTimeout(r,1000));}return false;}"
          "document.getElementById('configForm').addEventListener('submit',function(e){e.preventDefault();"
          "const form=this,button=document.getElementById('configButton'),state=document.getElementById('configState');"
          "button.disabled=true;state.className='upload-state';state.textContent='';showBusy('Saving settings','Applying the network configuration. Please wait.',35);"
          "const body=new URLSearchParams(new FormData(form));body.set('response','json');const xhr=new XMLHttpRequest();xhr.open('POST','/save');xhr.timeout=30000;xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded;charset=UTF-8');"
          "xhr.onload=function(){let data;try{data=JSON.parse(xhr.responseText);}catch(_){data={ok:false,message:xhr.responseText||'The device returned an unreadable response.'};}"
          "if(xhr.status>=200&&xhr.status<300&&data.ok){busyProgress.style.width='100%';busyTitle.textContent='Settings saved';busyText.textContent=data.apRestarting?'Reconnect to the saved setup network to continue.':'The new configuration is now active.';state.className='upload-state show okay';state.textContent=data.message||'Settings saved successfully.';"
          "if(data.pollIntervalMs){const poll=form.elements['poll_ms'];if(poll)poll.value=data.pollIntervalMs;}setTimeout(hideBusy,1200);}"
          "else{hideBusy();state.className='upload-state show bad';state.textContent=data.message||('Save failed with status '+xhr.status+'.');}button.disabled=false;};"
          "xhr.onerror=function(){hideBusy();button.disabled=false;state.className='upload-state show bad';state.textContent='The connection changed while settings were being applied. If the setup network name or password changed, reconnect using the saved credentials.';};"
          "xhr.ontimeout=function(){hideBusy();button.disabled=false;state.className='upload-state show bad';state.textContent='The save response timed out. Reconnect to the setup network and check the displayed settings before trying again.';};"
          "xhr.send(body.toString());});"
          "document.getElementById('profileForm').addEventListener('submit',function(e){e.preventDefault();"
          "const form=this,file=document.getElementById('profileFile'),button=document.getElementById('profileButton'),state=document.getElementById('profileState');"
          "if(!file.files.length)return;let expectedKey='';file.files[0].text().then(t=>{try{expectedKey=JSON.parse(t).profileKey||'';}catch(_){}});button.disabled=true;state.className='upload-state';state.textContent='';showBusy('Uploading sensor profile','Validating and saving the profile. Do not close this page.',10);"
          "const xhr=new XMLHttpRequest();xhr.open('POST','/api/sensor-profile');xhr.timeout=45000;"
          "xhr.upload.onprogress=function(ev){if(ev.lengthComputable){const p=Math.max(10,Math.min(80,Math.round(ev.loaded/ev.total*80)));busyProgress.style.width=p+'%';busyText.textContent='Uploading... '+Math.round(ev.loaded/ev.total*100)+'%';}};"
          "xhr.onload=function(){let data;try{data=JSON.parse(xhr.responseText);}catch(_){data={ok:false,message:'The device returned an unreadable response. Reconnect to the setup network and try again.'};}"
          "if(xhr.status>=200&&xhr.status<300&&data.ok){busyProgress.style.width='100%';busyTitle.textContent='Profile applied';busyText.textContent='The sensor interface is restarting with the new profile.';"
          "document.getElementById('currentProfile').value=data.profileKey||'updated';state.className='upload-state show okay';state.textContent=data.message;file.value='';setTimeout(hideBusy,1200);}"
          "else{hideBusy();state.className='upload-state show bad';state.textContent=data.message||('Upload failed with status '+xhr.status+'.');}button.disabled=false;};"
          "xhr.onerror=function(){hideBusy();button.disabled=false;state.className='upload-state show bad';state.textContent='Connection lost during upload. Reconnect to the setup network and try again; the previous profile remains active.';};"
          "xhr.ontimeout=async function(){busyTitle.textContent='Checking saved profile';busyText.textContent='The response was delayed. Verifying the active profile now.';busyProgress.style.width='90%';"
          "if(await verifyUploadedProfile(expectedKey)){document.getElementById('currentProfile').value=expectedKey;state.className='upload-state show okay';state.textContent='The profile is active. The upload response was interrupted, but the saved profile was verified.';file.value='';}"
          "else{state.className='upload-state show bad';state.textContent='The upload did not complete. The previous profile remains active. Please try again.';}hideBusy();button.disabled=false;};"
          "xhr.send(new FormData(form));});"
          "</script></main></body></html>";
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSave() {
  if (!requireSetupApRequest()) return;
  const bool jsonResponse = server.arg("response") == "json";
  String newIdentificationName = server.arg("identification_name");
  newIdentificationName.trim();
  String newSsid   = server.arg("ssid");
  String newWpass  = server.arg("wpass");
  bool blankWpass  = server.hasArg("blank_wpass");
  String newBroker = server.arg("broker");
  String newPort   = server.arg("port");
  String newMuser  = server.arg("muser");
  String newMpass  = server.arg("mpass");
  String newPollMs = server.arg("poll_ms");
  String newApSsid = server.arg("ap_ssid");
  String newApPass = server.arg("ap_pass");
  String newEntUser = server.arg("ent_user");
  newEntUser.trim();
  bool entEnabled = server.hasArg("ent_enable");
  bool blankMpass  = server.hasArg("blank_mpass");
  if (newIdentificationName.length() > 64 ||
      newSsid.length() > 32 || newWpass.length() > 64 ||
      newBroker.length() > 128 || newMuser.length() > 64 ||
      newMpass.length() > 64 || newApSsid.length() > 32 ||
      newApPass.length() > 63 || newEntUser.length() > 127) {
    server.send(400, "text/plain", "A configuration field exceeds its maximum length.");
    return;
  }
  // The "Use Enterprise authentication" checkbox is the single source of
  // truth for which auth mode is saved, precisely so a value left over in the
  // Enterprise Username text field can never silently re-enable 802.1X on a
  // later, unrelated save. Unchecked always means an ordinary network, full
  // stop, regardless of what text is still sitting in that field.
  if (!entEnabled) {
    newEntUser = "";
  } else if (newEntUser.length() == 0) {
    server.send(
      400, "text/plain; charset=UTF-8",
      "Enterprise authentication is checked, but no Enterprise Username was entered.");
    return;
  } else if (newWpass.length() == 0 && blankWpass == false && wifiPass.length() == 0) {
    server.send(
      400, "text/plain; charset=UTF-8",
      "Enterprise authentication needs a password (the WiFi Password field above doubles as the account password).");
    return;
  }
  // Reject an impossible WPA2 passphrase at save time rather than letting the
  // sensor retry an association that the 802.11 standard guarantees will fail.
  // Enterprise is exempt: there the password is a RADIUS account password with
  // no 8-character floor. Blank stays allowed - that means an open network.
  if (!entEnabled && !blankWpass && newWpass.length() > 0 && newWpass.length() < 8) {
    server.send(
      400, "text/plain; charset=UTF-8",
      "A WPA2 WiFi password must be 8-63 characters. Leave the field blank to keep "
      "the current one, or tick \"Save WiFi password as blank\" for an open network.");
    return;
  }
  newApSsid.trim();
  if (newApSsid.length() == 0) {
    server.send(400, "text/plain; charset=UTF-8", "Setup AP SSID must contain 1-32 characters.");
    return;
  }
  if (newApPass.length() > 0 && newApPass.length() < 8) {
    server.send(400, "text/plain; charset=UTF-8", "Setup AP password must contain 8-63 characters.");
    return;
  }

  int parsedPort = mqttPort;
  if (newPort.length() > 0) {
    parsedPort = newPort.toInt();
    if (parsedPort < 1 || parsedPort > 65535) {
      Serial.printf("[Config] Invalid MQTT port %d. Keeping %d.\n", parsedPort, mqttPort);
      parsedPort = mqttPort;
    }
  }
  const unsigned long profilePollMinimum =
    max((unsigned long)SENSOR_READ_MIN_INTERVAL_MS,
        (unsigned long)ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs);
  unsigned long parsedPollInterval = sensorPollIntervalMs;
  bool pollIntervalClampedToProfile = false;
  if (newPollMs.length() > 0) {
    char *end = nullptr;
    errno = 0;
    const unsigned long requested = strtoul(newPollMs.c_str(), &end, 10);
    if (errno != 0 || end == newPollMs.c_str() || *end != '\0' ||
        requested > SENSOR_READ_MAX_INTERVAL_MS) {
      server.send(
        400, "text/plain; charset=UTF-8",
        "Acquisition interval must be a positive number no greater than " +
        String(SENSOR_READ_MAX_INTERVAL_MS) + " milliseconds.");
      return;
    }
    // A user may reasonably request a round value such as 300 ms while the
    // active sensor profile requires 350 ms. Apply the profile's audited safe
    // minimum instead of rejecting the whole settings form and silently
    // leaving an older, much slower interval active.
    parsedPollInterval = max(requested, profilePollMinimum);
    pollIntervalClampedToProfile = requested < profilePollMinimum;
  }

  const String oldWifiSsid = wifiSsid;
  const String oldIdentificationName = identificationName;
  const String oldWifiPass = wifiPass;
  const String oldWifiEnterpriseUser = wifiEnterpriseUser;
  const String oldMqttBroker = mqttBroker;
  const String oldMqttUser = mqttUser;
  const String oldMqttPass = mqttPass;
  const String oldSetupApSsid = setupApSsid;
  const String oldSetupApPass = setupApPass;
  const int oldMqttPort = mqttPort;
  const unsigned long oldSensorPollIntervalMs = sensorPollIntervalMs;

  identificationName = newIdentificationName;
  if (newSsid.length() > 0) wifiSsid = newSsid;
  if (blankWpass) {
    wifiPass = "";
  } else if (newWpass.length() > 0) {
    wifiPass = newWpass;
  }
  // Unlike the password fields, the Enterprise Username is shown in the form
  // (not a secret) and is always taken verbatim, including blank - that is
  // how a unit is switched back from an Enterprise network to a plain one.
  wifiEnterpriseUser = newEntUser;
  if (newBroker.length() > 0) mqttBroker = newBroker;
  mqttPort = parsedPort;
  mqttUser = newMuser;
  sensorPollIntervalMs = parsedPollInterval;
  setupApSsid = newApSsid;
  if (newApPass.length() > 0) setupApPass = newApPass;
  if (blankMpass) {
    mqttPass = "";
  } else if (newMpass.length() > 0) {
    mqttPass = newMpass;
  }
  if (!persistCurrentConfig()) {
    identificationName = oldIdentificationName;
    wifiSsid = oldWifiSsid;
    wifiPass = oldWifiPass;
    wifiEnterpriseUser = oldWifiEnterpriseUser;
    mqttBroker = oldMqttBroker;
    mqttUser = oldMqttUser;
    mqttPass = oldMqttPass;
    setupApSsid = oldSetupApSsid;
    setupApPass = oldSetupApPass;
    mqttPort = oldMqttPort;
    sensorPollIntervalMs = oldSensorPollIntervalMs;
    server.send(
      500, "text/plain; charset=UTF-8",
      "Configuration was not saved or verified. Previous settings remain active. Reason: " +
      lastConfigStorageError);
    return;
  }

  const bool setupApCredentialsChanged =
    setupApSsid != oldSetupApSsid || setupApPass != oldSetupApPass;
  String successMessage =
    "Settings saved successfully. The new configuration is being applied without stopping acquisition.";
  if (pollIntervalClampedToProfile) {
    successMessage +=
      " The requested acquisition interval was below this sensor profile's safe minimum; IVSS applied " +
      String(parsedPollInterval) + " ms.";
  }
  if (setupApCredentialsChanged) {
    successMessage +=
      " The setup network will restart; reconnect using the saved AP name and password.";
  }
  if (jsonResponse) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(
      200, "application/json; charset=UTF-8",
      String("{\"ok\":true,\"message\":\"") + jsonEscape(successMessage) +
      "\",\"pollIntervalMs\":" + String(parsedPollInterval) +
      ",\"apRestarting\":" + String(setupApCredentialsChanged ? "true" : "false") + "}");
  } else {
    // No-JavaScript fallback: browsers that submit the form normally still get
    // a complete confirmation page instead of an ambiguous empty response.
    String html = FPSTR(HTML_HEAD);
    html += "<main class='shell'><div class='top'><div><h1>Saved</h1>"
            "<p class='sub'>Controller configuration updated</p></div><span class='badge'>Applying</span></div>"
            "<form><p class='ok'>" + htmlEscape(successMessage) + "</p>"
            "<p class='hint'>This setup network stays open for 10 minutes after valid sensor data reaches MQTT. To reopen it later, disconnect and reconnect the sensor's power.</p>"
            "</form></main></body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
  }
  // Restart the bounded STA-pause grace window from this exact moment so the
  // save response just sent above gets its full protection window, no matter
  // how long the client had already been associated before this request.
  if (setupApClientPresent()) {
    setupApClientJoinedMs = millis();
  }

  if (setupApCredentialsChanged) {
    setupApRestartPending = true;
    setupApRestartAfterMs = millis() + 1500UL;
  }

  mqttClient.disconnect();
  mqttClient.setServer(mqttBroker.c_str(), mqttPort);
  mqttClient.setCallback(mqttMessageReceived);
  mqttClient.setKeepAlive(15);
  mqttBufferReady = mqttClient.setBufferSize(MQTT_PACKET_BUFFER_SIZE);
  mqttClient.setSocketTimeout(1);
  lastMqttAttemptMs = 0;
  mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
  mqttDisconnectedSinceMs = 0;
  mqttConnectedSinceMs = 0;
  lastNodeServerHeartbeatMs = 0;
  nodeServerHeartbeatSeen = false;
  nodeServerHeartbeatTimedOut = false;
  lastMqttConnected = false;
  currentPublishIntervalMs =
    max((unsigned long)PUBLISH_FAST_MS, sensorPollIntervalMs);

  if (setupApClientPresent()) {
    // The response is still being consumed through SoftAP. Let serviceWiFi()
    // start STA association after the setup client disconnects so a router on
    // another channel cannot interrupt the saved-page response.
    staConnectInProgress = false;
    staSettingsApplyPending = true;
    wifiRetryDelayMs = WIFI_RETRY_INITIAL_MS;
    lastWifiAttemptMs = millis() - wifiRetryDelayMs;
    Serial.println("[WiFi] Saved STA settings; association deferred until the setup client disconnects.");
  } else {
    staSettingsApplyPending = false;
    beginStaConnection("settings saved");
  }
}

void handleProfileUploadStream() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    profileUploadJson = "";
    profileUploadError = "";
    profileUploadReady = false;
    profileUploadJson.reserve(SENSOR_PROFILE_UPLOAD_MAX_BYTES);
    WiFiClient client = server.client();
    if (!client || client.localIP() != setupApIp) {
      profileUploadError =
        "Profile uploads are accepted only through the IVSS access point.";
      return;
    }
    String filename = upload.filename;
    filename.toLowerCase();
    if (!filename.endsWith(".json")) {
      profileUploadError = "The uploaded file must have a .json extension.";
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (profileUploadError.length() == 0) {
      if (profileUploadJson.length() + upload.currentSize >
          SENSOR_PROFILE_UPLOAD_MAX_BYTES) {
        profileUploadError = "Profile JSON exceeds the 12288-byte limit.";
        profileUploadJson = "";
      } else if (!profileUploadJson.concat(
                   reinterpret_cast<const char *>(upload.buf),
                   upload.currentSize)) {
        profileUploadError = "Insufficient memory while receiving profile.";
        profileUploadJson = "";
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    profileUploadReady =
      profileUploadError.length() == 0 && profileUploadJson.length() > 0;
    if (!profileUploadReady && profileUploadError.length() == 0) {
      profileUploadError = "The uploaded profile is empty.";
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    profileUploadError = "Profile upload was interrupted.";
    profileUploadJson = "";
    profileUploadReady = false;
  }
}

bool profileUploadIsApiRequest() {
  return server.uri().startsWith("/api/");
}

void sendProfileUploadFailure(int statusCode, const String &message) {
  server.sendHeader("Cache-Control", "no-store");
  if (profileUploadIsApiRequest()) {
    server.send(statusCode, "application/json; charset=UTF-8",
                "{\"ok\":false,\"message\":\"" +
                jsonEscape(message) + "\"}");
  } else {
    String html = FPSTR(HTML_HEAD);
    html.reserve(6000);
    html += "<main class='shell'><div class='top'><div><h1>Profile Not Applied</h1>"
            "<p class='sub'>The previous sensor profile remains active</p></div>"
            "<span class='badge'>Action required</span></div><form>"
            "<p class='err'>" + htmlEscape(message) + "</p>"
            "<p><a href='/' style='color:#39c1ff'>Return to setup</a></p>"
            "</form></main></body></html>";
    server.send(statusCode, "text/html; charset=UTF-8", html);
  }
}

void handleProfileUploadComplete() {
  if (!requireSetupApRequest()) {
    profileUploadJson = "";
    profileUploadReady = false;
    return;
  }
  if (!profileUploadReady) {
    const String message = profileUploadError.length() > 0
      ? profileUploadError : String("No profile file was received.");
    sendProfileUploadFailure(400, message);
    profileUploadJson = "";
    return;
  }

  ModbusSensorProfile candidate = {};
  String error;
  if (!parseSensorProfileJson(profileUploadJson, candidate, error)) {
    sendProfileUploadFailure(400, "Profile rejected: " + error);
    profileUploadJson = "";
    profileUploadReady = false;
    return;
  }
  if (!persistSensorProfile(candidate)) {
    sendProfileUploadFailure(
      500, "Profile was valid but could not be written and verified. "
           "The previous profile remains active. Storage detail: " +
           lastProfileStorageError);
    profileUploadJson = "";
    profileUploadReady = false;
    return;
  }

  ACTIVE_SENSOR_PROFILE = candidate;
  sensorProfileLoadedFromStorage = true;
  sensorBaud = ACTIVE_SENSOR_PROFILE.defaultBaud;
  sensorModbusId = ACTIVE_SENSOR_PROFILE.defaultSlaveId;
  sensorAccelRegBase = ACTIVE_SENSOR_PROFILE.acceleration.startRegister;
  sensorReadFunction = ACTIVE_SENSOR_PROFILE.readFunction;
  useSplitModbusReads = !ACTIVE_SENSOR_PROFILE.supportsContiguousFastFrame;
  // Applying a profile is an explicit sensor reconfiguration. Start it at the
  // profile's audited acquisition interval rather than carrying over a slower
  // minimum from a temporary/previous profile (for example, a 1000 ms
  // communication-test profile). The operator can choose a slower interval
  // afterwards if required.
  sensorPollIntervalMs = max((unsigned long)SENSOR_READ_MIN_INTERVAL_MS,
                             (unsigned long)ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs);
  currentPublishIntervalMs = sensorPollIntervalMs;
  persistCurrentConfig();
  sensorProfileReconfigurePending = true;
  sensorProfileReconfigureAfterMs = millis() + 750;

  const String successMessage =
    String("Profile '") + ACTIVE_SENSOR_PROFILE.profileKey +
    "' was validated, saved, and applied. " +
    (profileStorageHealthy
      ? "Both protected storage copies are healthy."
      : "One protected copy is active; upload the profile again before shipment.");
  if (profileUploadIsApiRequest()) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(
      200, "application/json; charset=UTF-8",
      "{\"ok\":true,\"message\":\"" + jsonEscape(successMessage) +
      "\",\"profileKey\":\"" +
      jsonEscape(ACTIVE_SENSOR_PROFILE.profileKey) +
      "\",\"storageHealthy\":" +
      String(profileStorageHealthy ? "true" : "false") + "}");
  } else {
    String html = FPSTR(HTML_HEAD);
    html.reserve(6500);
    html += "<main class='shell'><div class='top'><div><h1>Sensor Profile Saved</h1>"
            "<p class='sub'>Validated and applied</p></div>"
            "<span class='badge'>Complete</span></div><form>"
            "<p class='ok'>" + htmlEscape(successMessage) + "</p>"
            "<p class='hint'>The sensor interface will restart and recalibrate automatically.</p>"
            "<p><a href='/' style='color:#39c1ff'>Return to setup</a></p>"
            "</form></main></body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
  }
  profileUploadJson = "";
  profileUploadReady = false;
}

void handleProfileDownload() {
  if (!requireSetupApRequest()) return;
  server.sendHeader("Content-Disposition",
                    "attachment; filename=\"IVSS-sensor-profile.json\"");
  server.send(200, "application/json; charset=UTF-8",
              sensorProfileJson(ACTIVE_SENSOR_PROFILE));
}

void handleJetBrainsMonoFont() {
  if (!requireSetupApRequest()) return;
  const size_t fontSize =
    static_cast<size_t>(
      reinterpret_cast<uintptr_t>(jetbrainsMonoRegularEnd) -
      reinterpret_cast<uintptr_t>(jetbrainsMonoRegularStart));
  server.sendHeader("Cache-Control", "public, max-age=31536000, immutable");
  server.send_P(200, PSTR("font/woff2"),
                reinterpret_cast<PGM_P>(jetbrainsMonoRegularStart), fontSize);
}

void handleStatus() {
  if (!requireSetupApRequest()) return;
  bool staConfigured = wifiSsid.length() > 0;
  bool staConnected = (WiFi.status() == WL_CONNECTED);
  const unsigned long statusNow = millis();
  const bool serverHeartbeatFresh = nodeServerHeartbeatFresh(statusNow);
  String json = "{";
  json += "\"deviceId\":\"" + jsonEscape(deviceId) + "\",";
  json += "\"identificationName\":\"" + jsonEscape(identificationName) + "\",";
  json += "\"firmwareName\":\"" + jsonEscape(FIRMWARE_NAME) + "\",";
  json += "\"firmwareVersion\":\"" + jsonEscape(FIRMWARE_VERSION) + "\",";
  json += "\"sensorModel\":\"" + jsonEscape(SENSOR_MODEL_NAME) + "\",";
  json += "\"sensorProfile\":\"" + jsonEscape(ACTIVE_SENSOR_PROFILE.profileKey) + "\",";
  json += "\"sensorProfileSource\":\"" +
          String(sensorProfileLoadedFromStorage ? "uploaded" : "built_in") + "\",";
  json += "\"registerMapVersion\":\"" + jsonEscape(SENSOR_REGISTER_MAP_VERSION) + "\",";
  json += "\"protocol\":\"" + jsonEscape(DEVICE_PROTOCOL_NAME) + "\",";
  json += "\"wifiMode\":\"AP_STA\",";
  json += "\"ap\":{";
  json += "\"enabled\":";
  json += (setupApStarted ? "true" : "false");
  json += ",\"ssid\":\"" + jsonEscape(setupApSsid) + "\",";
  json += "\"ip\":\"" + jsonEscape(setupApStarted ? WiFi.softAPIP().toString() : String("")) + "\",";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum());
  json += "},";
  json += "\"sta\":{";
  json += "\"configured\":";
  json += (staConfigured ? "true" : "false");
  json += ",\"connected\":";
  json += (staConnected ? "true" : "false");
  json += ",\"ssid\":\"" + jsonEscape(wifiSsid) + "\",";
  json += "\"ip\":\"" + jsonEscape(staConnected ? WiFi.localIP().toString() : String("")) + "\",";
  json += "\"rssi\":";
  json += (staConnected ? String(WiFi.RSSI()) : String("null"));
  json += "},";
  json += "\"mqtt\":{";
  json += "\"connected\":";
  json += (mqttClient.connected() ? "true" : "false");
  json += ",\"broker\":\"" + jsonEscape(mqttBroker) + "\",";
  json += "\"port\":" + String(mqttPort) + ",";
  json += "\"serverHeartbeatSeen\":" +
          String(nodeServerHeartbeatSeen ? "true" : "false") + ",";
  json += "\"serverHeartbeatFresh\":" +
          String(serverHeartbeatFresh ? "true" : "false") + ",";
  json += "\"serverHeartbeatAgeMs\":";
  json += nodeServerHeartbeatSeen
    ? String(statusNow - lastNodeServerHeartbeatMs)
    : String("null");
  json += "},";
  json += "\"sensorBus\":{";
  json += "\"uart\":2,";
  json += "\"txGpio\":" + String(UART2_TX_PIN) + ",";
  json += "\"rxGpio\":" + String(UART2_RX_PIN) + ",";
  json += "\"baud\":" + String(sensorBaud) + ",";
  json += "\"modbusId\":" + String(sensorModbusId) + ",";
  json += "\"autoDirection\":" + String(RS485_DE_RE_PIN < 0 ? "true" : "false") + ",";
  json += "\"successCount\":" + String(modbusSuccessCount) + ",";
  json += "\"failureCount\":" + String(modbusFailCount) + ",";
  json += "\"timeoutCount\":" + String(modbusTimeoutCount) + ",";
  json += "\"crcFailureCount\":" + String(modbusCrcFailCount) + ",";
  json += "\"echoCount\":" + String(modbusEchoCount) + ",";
  json += "\"discardedNoiseBytes\":" + String(modbusNoiseByteCount) + ",";
  json += "\"lastError\":\"" + jsonEscape(String(lastModbusErrorReason)) + "\"";
  json += "}";
  json += ",\"reliability\":{";
  json += "\"configStorageHealthy\":" + String(configStorageHealthy ? "true" : "false") + ",";
  json += "\"configStorageError\":\"" +
          jsonEscape(lastConfigStorageError) + "\",";
  json += "\"profileStorageHealthy\":" + String(profileStorageHealthy ? "true" : "false") + ",";
  json += "\"profileStorageError\":\"" +
          jsonEscape(lastProfileStorageError) + "\",";
  json += "\"networkIdentityHealthy\":" + String(networkIdentityHealthy ? "true" : "false") + ",";
  json += "\"watchdogActive\":" + String(watchdogReady ? "true" : "false") + ",";
  json += "\"mqttBufferReady\":" + String(mqttBufferReady ? "true" : "false") + ",";
  json += "\"telemetryMemoryHealthy\":" + String(telemetryMemoryHealthy ? "true" : "false") + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"largestFreeBlock\":" + String(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  json += "}";
  json += "}";
  server.send(200, "application/json", json);
}

void handleNotFound() {
  if (!requireSetupApRequest()) return;
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

// =============================================================================
// [MQTT] Broker connection [HIGH RISK]
// =============================================================================
// Reconnection must stay bounded and must never stop SoftAP or acquisition.
void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED || mqttBroker.length() == 0) {
    return;
  }

  Serial.print("[MQTT] Connecting to " + mqttBroker + ":" + String(mqttPort) + " ...");
  bool connected = false;
  if (mqttUser.length() > 0) {
    connected = mqttClient.connect(deviceId.c_str(), mqttUser.c_str(), mqttPass.c_str());
  } else {
    connected = mqttClient.connect(deviceId.c_str());
  }

  if (connected) {
    const bool heartbeatSubscribed =
      mqttClient.subscribe(NODE_SERVER_HEARTBEAT_TOPIC);
    if (!heartbeatSubscribed) {
      Serial.println(" connected, but Node heartbeat subscription failed.");
      mqttClient.disconnect();
      mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
      mqttConnectedSinceMs = 0;
      nodeServerHeartbeatSeen = false;
      nodeServerHeartbeatTimedOut = false;
      activateRecoverySetupPortal("MQTT heartbeat subscription failed");
      return;
    }
    mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
    mqttConnectedSinceMs = millis();
    mqttSessionProgressMs = mqttConnectedSinceMs;
    lastNodeHeartbeatSubscribeAttemptMs = mqttConnectedSinceMs;
    lastNodeServerHeartbeatMs = 0;
    nodeServerHeartbeatSeen = false;
    nodeServerHeartbeatTimedOut = false;
    Serial.println(" connected.");
  } else {
    mqttRetryDelayMs = min(mqttRetryDelayMs * 2UL, MQTT_RETRY_MAX_MS);
    Serial.println(" failed. rc=" + String(mqttClient.state()));
    activateRecoverySetupPortal("MQTT broker connection failed");
  }
}

// =============================================================================
// [SENSOR PROFILE] Profile-driven Modbus RTU (Serial2) and publishing
// =============================================================================
// [HIGH RISK] The code below controls the physical RS485 bus, response timing,
// CRC/frame acceptance, register decoding, recovery, and telemetry validity.
// Small changes can create sensor_fault reports or corrupt engineering values.
void saturatingAdd(uint32_t &counter, uint32_t amount = 1U) {
  counter = (UINT32_MAX - counter < amount) ? UINT32_MAX : counter + amount;
}

uint16_t modbusCrc16(const uint8_t *data, size_t len) {
  // [SENSOR PROFILE] [HIGH RISK] Protocol CRC; must remain Modbus CRC-16.
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void rs485SetTransmit(bool tx) {
  if (RS485_DE_RE_PIN >= 0) {
    bool level = tx ? RS485_DIR_ACTIVE_HIGH : !RS485_DIR_ACTIVE_HIGH;
    digitalWrite(RS485_DE_RE_PIN, level ? HIGH : LOW);
  }
}

void flushModbusRx(uint16_t settleMs) {
  while (Serial2.available() > 0) Serial2.read();
  if (settleMs > 0) delay(settleMs);
  while (Serial2.available() > 0) Serial2.read();
}

bool modbusRecoveryActive() {
  if (!modbusRecoveryPending) return false;
  if ((uint32_t)(millis() - modbusRecoveryStartedMs) >= MODBUS_RECOVERY_PAUSE_MS) {
    modbusRecoveryPending = false;
    return false;
  }
  return true;
}

unsigned long modbusRecoveryRemainingMs() {
  if (!modbusRecoveryActive()) return 0;
  const uint32_t elapsed = (uint32_t)(millis() - modbusRecoveryStartedMs);
  return MODBUS_RECOVERY_PAUSE_MS - elapsed;
}

uint32_t activeSensorSerialConfig() {
  if (strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8N2") == 0) return SERIAL_8N2;
  if (strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8E1") == 0) return SERIAL_8E1;
  if (strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8E2") == 0) return SERIAL_8E2;
  if (strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8O1") == 0) return SERIAL_8O1;
  if (strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8O2") == 0) return SERIAL_8O2;
  return SERIAL_8N1;
}

unsigned long activeModbusInterFrameGapMs() {
  uint32_t bitsPerCharacter = 11U;
  if (strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8N1") == 0) {
    bitsPerCharacter = 10U;
  } else if (strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8E2") == 0 ||
             strcmp(ACTIVE_SENSOR_PROFILE.serialFormat, "8O2") == 0) {
    bitsPerCharacter = 12U;
  }
  // RTU requires at least 3.5 silent character times. Keep the existing 5 ms
  // floor at normal rates and expand it automatically for slow serial links.
  const uint32_t calculated =
    (bitsPerCharacter * 3500UL + sensorBaud - 1UL) / sensorBaud;
  return max((unsigned long)MODBUS_MIN_INTER_REQUEST_GAP_MS,
             (unsigned long)calculated);
}

void restartModbusUart(const char *reason) {
  // [SENSOR PROFILE] [HIGH RISK] Ends and recreates the live sensor UART.
  Serial.printf("[Modbus] Recovery: restarting UART2 after %u consecutive failures. reason=%s\n",
                consecutiveModbusFailures, reason ? reason : "unknown");
  rs485SetTransmit(false);
  Serial2.end();
  delay(50);
  Serial2.begin(sensorBaud, activeSensorSerialConfig(),
                UART2_RX_PIN, UART2_TX_PIN);
  flushModbusRx(10);
  useSplitModbusReads = !ACTIVE_SENSOR_PROFILE.supportsContiguousFastFrame;
  consecutiveFastFrameFailures = 0;
  lastModbusRequestEndMs = millis();
}

void setSensorPollInterval(unsigned long intervalMs, bool persist) {
  unsigned long clamped = intervalMs;
  const unsigned long profileMinimum =
    max((unsigned long)SENSOR_READ_MIN_INTERVAL_MS,
        (unsigned long)ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs);
  if (clamped < profileMinimum) clamped = profileMinimum;
  if (clamped > SENSOR_READ_MAX_INTERVAL_MS) clamped = SENSOR_READ_MAX_INTERVAL_MS;
  sensorPollIntervalMs = clamped;
  currentPublishIntervalMs = clamped;
  const bool saved = !persist || persistCurrentConfig();
  Serial.printf("[Sensor] Poll interval set to %lu ms (%.2f Hz)%s\n",
                sensorPollIntervalMs, 1000.0f / (float)sensorPollIntervalMs,
                persist ? (saved ? " and saved" : " but persistence FAILED") : "");
}

void noteModbusSuccess() {
  saturatingAdd(modbusSuccessCount);
  consecutiveModbusFailures = 0;
  sensorPowerCycleAttempted = false;
  lastModbusFailureMs = 0;
  lastModbusSuccessMs = millis();
  snprintf(lastModbusErrorReason, sizeof(lastModbusErrorReason), "none");
}

void logModbusError(const char *label, const char *reason, uint16_t detail) {
  static unsigned long lastLogMs = 0;
  unsigned long now = millis();
  lastModbusFailureMs = now;
  snprintf(lastModbusErrorReason, sizeof(lastModbusErrorReason), "%s:%s:0x%04X",
           label ? label : "modbus", reason ? reason : "unknown", detail);
  consecutiveModbusFailures++;
  if (SENSOR_POWER_PIN >= 0 && !sensorPowerCycleAttempted &&
      (uint32_t)(now - lastModbusSuccessMs) >= SENSOR_POWER_CYCLE_AFTER_MS) {
    powerCycleSensor();
  }
  if (consecutiveModbusFailures >= MODBUS_FAILURES_BEFORE_RECOVERY && !modbusRecoveryActive()) {
    modbusRecoveryStartedMs = now;
    modbusRecoveryPending = true;
    restartModbusUart(lastModbusErrorReason);
    consecutiveModbusFailures = 0;
  }
  if (now - lastLogMs < SENSOR_LOG_INTERVAL_MS) {
    return;
  }
  lastLogMs = now;
  Serial.printf("[Modbus] %s failed: %s", label, reason);
  if (detail != 0) {
    Serial.printf(" (0x%04X)", detail);
  }
  Serial.printf(" | id=0x%02X baud=%lu fcode=0x%02X reg=0x%04X fail=%lu timeout=%lu crc=%lu exc=%lu bad=%lu\n",
                sensorModbusId, (unsigned long)sensorBaud, sensorReadFunction,
                sensorAccelRegBase, (unsigned long)modbusFailCount,
                (unsigned long)modbusTimeoutCount, (unsigned long)modbusCrcFailCount,
                (unsigned long)modbusExceptionCount, (unsigned long)modbusBadFrameCount);
}

void powerCycleSensor() {
  if (SENSOR_POWER_PIN < 0) return;
  sensorPowerCycleAttempted = true;
  rs485SetTransmit(false);
  Serial2.end();
  digitalWrite(SENSOR_POWER_PIN, LOW);
  delay(SENSOR_POWER_CYCLE_OFF_MS);
  feedWatchdog();
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(SENSOR_BOOT_SETTLE_MS);
  feedWatchdog();
  Serial2.begin(sensorBaud, activeSensorSerialConfig(),
                UART2_RX_PIN, UART2_TX_PIN);
  flushModbusRx(20);
  lastModbusRequestEndMs = millis();
  Serial.println("[Sensor] External sensor power was cycled after prolonged communication failure.");
}

bool readModbusRegisters(uint16_t startReg, uint16_t count, uint8_t *data,
                         size_t dataLen, const char *label) {
  return readModbusRegistersFunction(
    sensorReadFunction, startReg, count, data, dataLen, label);
}

bool readModbusRegistersFunction(uint8_t readFunction, uint16_t startReg,
                                 uint16_t count, uint8_t *data,
                                 size_t dataLen, const char *label) {
  // [SENSOR PROFILE] [HIGH RISK] Timing-sensitive transaction/parser path.
  // Timeout, frame scanning, echo removal, and CRC checks protect live reads.
  const unsigned long now = millis();
  if (modbusRecoveryActive()) {
    return false;
  }
  if (lastModbusFailureMs > 0 && now - lastModbusFailureMs < MODBUS_FAILURE_BACKOFF_MS) {
    return false;
  }

  if ((readFunction != 0x03 && readFunction != 0x04) ||
      count == 0 || count > 125 || dataLen < (size_t)count * 2) {
    saturatingAdd(modbusFailCount);
    saturatingAdd(modbusBadFrameCount);
    logModbusError(label, "invalid local request", count);
    return false;
  }

  if (lastModbusRequestEndMs > 0) {
    const unsigned long elapsed = millis() - lastModbusRequestEndMs;
    const unsigned long requiredGapMs = activeModbusInterFrameGapMs();
    if (elapsed < requiredGapMs) {
      delay(requiredGapMs - elapsed);
    }
  }

  uint8_t req[8];
  req[0] = sensorModbusId;
  req[1] = readFunction;
  req[2] = (uint8_t)(startReg >> 8);
  req[3] = (uint8_t)(startReg & 0xFF);
  req[4] = (uint8_t)(count >> 8);
  req[5] = (uint8_t)(count & 0xFF);
  uint16_t reqCrc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(reqCrc & 0xFF);
  req[7] = (uint8_t)((reqCrc >> 8) & 0xFF);

  flushModbusRx(2);

  rs485SetTransmit(true);
  delayMicroseconds(200);
  Serial2.write(req, sizeof(req));
  Serial2.flush();
  delayMicroseconds(200);
  rs485SetTransmit(false);

  const uint8_t expectedByteCount = (uint8_t)(count * 2);
  const uint32_t expectedFrameBytes = 3UL + expectedByteCount + 2UL;
  uint32_t respTimeoutMs =
    (uint32_t)((expectedFrameBytes * 12UL * 1000UL) / sensorBaud) + MODBUS_RESPONSE_TIMEOUT_MS;
  // Large responses can take seconds at low baud. Keep a finite ceiling, but
  // do not make valid profile-declared reads impossible.
  if (respTimeoutMs > 15000UL) respTimeoutMs = 15000UL;

  // Auto-direction converters may echo our request, and a late/noisy byte can
  // precede a valid response. Scan for a complete CRC-valid frame instead of
  // declaring the sensor faulty from the first three bytes in the FIFO.
  uint8_t rx[272];
  size_t rxLen = 0;
  bool sawAnyBytes = false;
  bool sawCandidate = false;
  bool sawCrcMismatch = false;
  const unsigned long responseStartMs = millis();

  while (millis() - responseStartMs <= respTimeoutMs) {
    feedWatchdog();
    while (Serial2.available() > 0) {
      sawAnyBytes = true;
      if (rxLen == sizeof(rx)) {
        memmove(rx, rx + 1, --rxLen);
        saturatingAdd(modbusNoiseByteCount);
      }
      rx[rxLen++] = (uint8_t)Serial2.read();
    }

    while (rxLen > 0) {
      if (rxLen >= sizeof(req) && memcmp(rx, req, sizeof(req)) == 0) {
        memmove(rx, rx + sizeof(req), rxLen - sizeof(req));
        rxLen -= sizeof(req);
        saturatingAdd(modbusEchoCount);
        continue;
      }

      size_t slavePos = 0;
      while (slavePos < rxLen && rx[slavePos] != sensorModbusId) slavePos++;
      if (slavePos > 0) {
        memmove(rx, rx + slavePos, rxLen - slavePos);
        rxLen -= slavePos;
        saturatingAdd(modbusNoiseByteCount, (uint32_t)slavePos);
      }
      if (rxLen < 2) break;

      const bool isNormal = rx[1] == readFunction;
      const bool isException = rx[1] == (uint8_t)(readFunction | 0x80);
      if (!isNormal && !isException) {
        memmove(rx, rx + 1, --rxLen);
        saturatingAdd(modbusNoiseByteCount);
        continue;
      }
      if (rxLen < 3) break;

      size_t frameLen = 5;
      if (isNormal) {
        if (rx[2] != expectedByteCount) {
          memmove(rx, rx + 1, --rxLen);
          saturatingAdd(modbusNoiseByteCount);
          continue;
        }
        frameLen = 3U + expectedByteCount + 2U;
      }
      sawCandidate = true;
      if (rxLen < frameLen) break;

      const uint16_t crcCalc = modbusCrc16(rx, frameLen - 2);
      const uint16_t crcRecv = (uint16_t)rx[frameLen - 2] | ((uint16_t)rx[frameLen - 1] << 8);
      if (crcCalc != crcRecv) {
        sawCrcMismatch = true;
        memmove(rx, rx + 1, --rxLen);
        saturatingAdd(modbusNoiseByteCount);
        continue;
      }

      if (isException) {
        saturatingAdd(modbusFailCount);
        saturatingAdd(modbusExceptionCount);
        logModbusError(label, "Modbus exception response", rx[2]);
        flushModbusRx(MODBUS_POST_FAIL_DRAIN_MS);
        lastModbusRequestEndMs = millis();
        return false;
      }

      memcpy(data, rx + 3, expectedByteCount);
      noteModbusSuccess();
      lastModbusRequestEndMs = millis();
      return true;
    }
    delay(1);
  }

  saturatingAdd(modbusFailCount);
  if (!sawAnyBytes) {
    saturatingAdd(modbusTimeoutCount);
    logModbusError(label, "timeout waiting for response", startReg);
  } else if (sawCrcMismatch) {
    saturatingAdd(modbusCrcFailCount);
    logModbusError(label, "no CRC-valid response", startReg);
  } else if (sawCandidate) {
    saturatingAdd(modbusTimeoutCount);
    logModbusError(label, "incomplete response", startReg);
  } else {
    saturatingAdd(modbusBadFrameCount);
    logModbusError(label, "no matching response in RX stream", startReg);
  }
  flushModbusRx(MODBUS_POST_FAIL_DRAIN_MS);
  lastModbusRequestEndMs = millis();
  return false;
}

struct DecodedProfileField {
  bool valid;
  uint64_t raw;
  double value;
};

bool decodeBcdValue(uint64_t raw, uint8_t byteCount, uint64_t &decoded) {
  decoded = 0;
  uint64_t multiplier = 1;
  for (uint8_t nibble = 0; nibble < byteCount * 2U; nibble++) {
    const uint8_t digit = (uint8_t)((raw >> (nibble * 4U)) & 0x0FU);
    if (digit > 9U) return false;
    decoded += (uint64_t)digit * multiplier;
    multiplier *= 10U;
  }
  return true;
}

bool decodeGenericField(const ModbusFieldProfile &field,
                        const uint8_t *wireBytes, size_t availableBytes,
                        DecodedProfileField &decoded) {
  decoded = {};
  if (!field.enabled) return false;
  const uint8_t byteCount = profileDataTypeBytes(field.dataType);
  if (byteCount == 0 || availableBytes < byteCount) return false;
  uint8_t canonical[8] = {};
  for (uint8_t wireIndex = 0; wireIndex < byteCount; wireIndex++) {
    const uint8_t canonicalIndex =
      (uint8_t)(field.wireByteOrder[wireIndex] - 'A');
    if (canonicalIndex >= byteCount) return false;
    canonical[canonicalIndex] = wireBytes[wireIndex];
  }
  uint64_t raw = 0;
  for (uint8_t index = 0; index < byteCount; index++) {
    raw = (raw << 8) | canonical[index];
  }
  decoded.raw = raw;
  if (field.hasInvalidRaw && raw == field.invalidRaw) return false;
  uint64_t transformedRaw = raw;
  if (field.hasBitField) {
    transformedRaw = (raw & field.bitMask) >> field.bitShift;
  }

  double numeric = 0.0;
  if (field.hasBitField && field.bitFieldSigned) {
    const uint64_t shiftedMask = field.bitMask >> field.bitShift;
    uint8_t width = 0;
    for (uint64_t bits = shiftedMask; bits != 0; bits >>= 1) width++;
    if (width == 0) return false;
    uint64_t signExtended = transformedRaw;
    if (width < 64U &&
        (transformedRaw & (1ULL << (width - 1U))) != 0) {
      signExtended |= ~((1ULL << width) - 1ULL);
    }
    int64_t signedValue = 0;
    memcpy(&signedValue, &signExtended, sizeof(signedValue));
    numeric = (double)signedValue;
  } else {
  switch (field.dataType) {
    case ModbusProfileDataType::UInt16:
      numeric = (double)(uint16_t)transformedRaw;
      break;
    case ModbusProfileDataType::Int16:
      numeric = (double)(int16_t)transformedRaw;
      break;
    case ModbusProfileDataType::UInt32:
      numeric = (double)(uint32_t)transformedRaw;
      break;
    case ModbusProfileDataType::Int32:
      numeric = (double)(int32_t)transformedRaw;
      break;
    case ModbusProfileDataType::UInt64:
      numeric = (double)transformedRaw;
      break;
    case ModbusProfileDataType::Int64: {
      int64_t signedValue = 0;
      memcpy(&signedValue, &transformedRaw, sizeof(signedValue));
      numeric = (double)signedValue;
      break;
    }
    case ModbusProfileDataType::Float32: {
      const uint32_t bits = (uint32_t)raw;
      float value = 0.0f;
      memcpy(&value, &bits, sizeof(value));
      numeric = value;
      break;
    }
    case ModbusProfileDataType::Float64: {
      double value = 0.0;
      memcpy(&value, &raw, sizeof(value));
      numeric = value;
      break;
    }
    case ModbusProfileDataType::Bcd16:
    case ModbusProfileDataType::Bcd32: {
      uint64_t bcd = 0;
      if (!decodeBcdValue(raw, byteCount, bcd)) return false;
      numeric = (double)bcd;
      break;
    }
    case ModbusProfileDataType::UInt32Mod10000: {
      const uint16_t high = (uint16_t)(raw >> 16);
      const uint16_t low = (uint16_t)raw;
      if (low > 9999U) return false;
      numeric = (double)high * 10000.0 + low;
      break;
    }
    case ModbusProfileDataType::Int32Mod10000: {
      const int16_t high = (int16_t)(raw >> 16);
      const int16_t low = (int16_t)raw;
      if (low < -9999 || low > 9999) return false;
      numeric = (double)high * 10000.0 + low;
      break;
    }
    case ModbusProfileDataType::Boolean:
      numeric = transformedRaw != 0 ? 1.0 : 0.0;
      break;
  }
  }
  const double engineering = numeric * field.scale + field.offset;
  if (!isfinite(engineering) ||
      (field.hasValidRange &&
       (engineering < field.validMinimum ||
        engineering > field.validMaximum))) {
    return false;
  }
  decoded.value = engineering;
  decoded.valid = true;
  return true;
}

bool decodeFieldFromBlock(const ModbusFieldProfile &field,
                          uint16_t blockStart, uint16_t blockCount,
                          const uint8_t *data,
                          DecodedProfileField &decoded) {
  if (!field.enabled || field.address < blockStart ||
      (uint32_t)field.address + field.registerCount >
        (uint32_t)blockStart + blockCount) {
    return false;
  }
  const size_t byteOffset = (size_t)(field.address - blockStart) * 2U;
  return decodeGenericField(field, data + byteOffset,
                            (size_t)field.registerCount * 2U, decoded);
}

bool readGenericVector(const ModbusFieldProfile fields[3],
                       DecodedProfileField decoded[3],
                       const char *label) {
  uint32_t spanStart = fields[0].address;
  uint32_t spanEnd =
    (uint32_t)fields[0].address + fields[0].registerCount;
  const uint8_t function = fields[0].readFunction;
  bool oneSpan = fields[0].enabled;
  for (uint8_t axis = 0; axis < 3; axis++) {
    oneSpan = oneSpan && fields[axis].enabled &&
              fields[axis].readFunction == function;
    spanStart = min<uint32_t>(spanStart, fields[axis].address);
    spanEnd = max<uint32_t>(
      spanEnd,
      (uint32_t)fields[axis].address + fields[axis].registerCount);
  }
  const uint32_t spanCount = spanEnd - spanStart;
  if (oneSpan && spanCount <= ACTIVE_SENSOR_PROFILE.maximumReadRegisters &&
      spanCount <= 125U) {
    uint8_t data[250];
    if (!readModbusRegistersFunction(
          function, (uint16_t)spanStart, (uint16_t)spanCount,
          data, sizeof(data), label)) {
      return false;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
      if (!decodeFieldFromBlock(
            fields[axis], (uint16_t)spanStart, (uint16_t)spanCount,
            data, decoded[axis])) {
        return false;
      }
    }
    return true;
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    uint8_t data[8];
    if (!fields[axis].enabled ||
        !readModbusRegistersFunction(
          fields[axis].readFunction, fields[axis].address,
          fields[axis].registerCount, data, sizeof(data), label) ||
        !decodeGenericField(fields[axis], data,
                            fields[axis].registerCount * 2U,
                            decoded[axis])) {
      return false;
    }
  }
  return true;
}

bool readGenericScalar(const ModbusFieldProfile &field,
                       DecodedProfileField &decoded, const char *label) {
  uint8_t data[8];
  return field.enabled &&
         readModbusRegistersFunction(
           field.readFunction, field.address, field.registerCount,
           data, sizeof(data), label) &&
         decodeGenericField(
           field, data, field.registerCount * 2U, decoded);
}

uint8_t genericVectorTransactionCount(const ModbusFieldProfile fields[3]) {
  // Mirror readGenericVector() exactly: a compatible contiguous span is one
  // transaction; otherwise the three axes are read independently.
  if (!fields[0].enabled || !fields[1].enabled || !fields[2].enabled) {
    return 0;
  }
  uint32_t spanStart = fields[0].address;
  uint32_t spanEnd =
    (uint32_t)fields[0].address + fields[0].registerCount;
  const uint8_t function = fields[0].readFunction;
  bool oneSpan = true;
  for (uint8_t axis = 0; axis < 3; axis++) {
    oneSpan = oneSpan && fields[axis].readFunction == function;
    spanStart = min<uint32_t>(spanStart, fields[axis].address);
    spanEnd = max<uint32_t>(
      spanEnd,
      (uint32_t)fields[axis].address + fields[axis].registerCount);
  }
  const uint32_t spanCount = spanEnd - spanStart;
  return oneSpan && spanCount <= ACTIVE_SENSOR_PROFILE.maximumReadRegisters &&
         spanCount <= 125U ? 1U : 3U;
}

uint8_t splitProfileTransactionsPerSample() {
  uint8_t transactions = 0;
  transactions += genericVectorTransactionCount(
    ACTIVE_SENSOR_PROFILE.accelerationFields);
  transactions += genericVectorTransactionCount(
    ACTIVE_SENSOR_PROFILE.velocityFields);
  transactions += genericVectorTransactionCount(
    ACTIVE_SENSOR_PROFILE.displacementFields);
  transactions += genericVectorTransactionCount(
    ACTIVE_SENSOR_PROFILE.frequencyFields);
  if (ACTIVE_SENSOR_PROFILE.temperatureField.enabled) transactions++;
  return transactions;
}

unsigned long effectiveSensorReadIntervalMs() {
  unsigned long intervalMs = max(
    sensorPollIntervalMs,
    (unsigned long)ACTIVE_SENSOR_PROFILE.minimumPollIntervalMs);
  if (!useSplitModbusReads) return intervalMs;

  const uint8_t transactions = splitProfileTransactionsPerSample();
  if (transactions == 0) return intervalMs;
  const unsigned long budgetedIntervalMs =
    ((unsigned long)transactions * 1000UL +
     MODBUS_SPLIT_MAX_REQUESTS_PER_SECOND - 1UL) /
    MODBUS_SPLIT_MAX_REQUESTS_PER_SECOND;
  return max(intervalMs, budgetedIntervalMs);
}

bool readSensorAccelerationRaw(float &ax, float &ay, float &az) {
  DecodedProfileField decoded[3] = {};
  if (!readGenericVector(
        ACTIVE_SENSOR_PROFILE.accelerationFields, decoded, "accel")) {
    return false;
  }
  ax = (float)decoded[0].value;
  ay = (float)decoded[1].value;
  az = (float)decoded[2].value;
  return true;
}

SensorEngineeringMetrics readSensorEngineeringMetrics() {
  SensorEngineeringMetrics m = {};

  DecodedProfileField velocity[3] = {};
  if (readGenericVector(
        ACTIVE_SENSOR_PROFILE.velocityFields, velocity, "velocity")) {
    m.velocityValid = true;
    m.rawVelocityX = velocity[0].raw;
    m.rawVelocityY = velocity[1].raw;
    m.rawVelocityZ = velocity[2].raw;
    m.velocityRmsX = (float)velocity[0].value;
    m.velocityRmsY = (float)velocity[1].value;
    m.velocityRmsZ = (float)velocity[2].value;
  }

  DecodedProfileField temperature = {};
  if (readGenericScalar(
        ACTIVE_SENSOR_PROFILE.temperatureField,
        temperature, "temperature")) {
    m.temperatureValid = true;
    m.temperatureC = (float)temperature.value;
  }

  DecodedProfileField displacement[3] = {};
  if (readGenericVector(
        ACTIVE_SENSOR_PROFILE.displacementFields,
        displacement, "displacement")) {
    m.displacementValid = true;
    m.rawDisplacementX = displacement[0].raw;
    m.rawDisplacementY = displacement[1].raw;
    m.rawDisplacementZ = displacement[2].raw;
    m.displacementX = (float)displacement[0].value;
    m.displacementY = (float)displacement[1].value;
    m.displacementZ = (float)displacement[2].value;
  }

  DecodedProfileField frequency[3] = {};
  if (readGenericVector(
        ACTIVE_SENSOR_PROFILE.frequencyFields,
        frequency, "frequency")) {
    m.rawFreqX = frequency[0].raw;
    m.rawFreqY = frequency[1].raw;
    m.rawFreqZ = frequency[2].raw;
    m.freqX = (float)frequency[0].value;
    m.freqY = (float)frequency[1].value;
    m.freqZ = (float)frequency[2].value;
    m.frequencyValid = isValidSensorFrequency(m.freqX) ||
                       isValidSensorFrequency(m.freqY) ||
                       isValidSensorFrequency(m.freqZ);
    m.frequencySuspicious =
      (m.freqX != 0.0f && !isValidSensorFrequency(m.freqX)) ||
      (m.freqY != 0.0f && !isValidSensorFrequency(m.freqY)) ||
      (m.freqZ != 0.0f && !isValidSensorFrequency(m.freqZ));
    m.dominantFreqX = m.freqX;
    m.dominantFreqY = m.freqY;
    m.dominantFreqZ = m.freqZ;
    // This diagnostic used to print on every split-profile sample. At 10 Hz it
    // could fill the UART buffer and make the acquisition path block even when
    // no serial monitor was attached.
    if (VERBOSE_SERIAL) {
      static unsigned long lastFrequencyAuditMs = 0;
      const unsigned long now = millis();
      if (now - lastFrequencyAuditMs >= SENSOR_LOG_INTERVAL_MS) {
        lastFrequencyAuditMs = now;
        Serial.printf(
          "[Sensor Frequency Audit] profile=%s registers=(0x%04X,0x%04X,0x%04X) raw=(%llu,%llu,%llu) interpreted=(%.1f,%.1f,%.1f)Hz source=sensor_reported suspicious=%d\n",
          ACTIVE_SENSOR_PROFILE.profileKey,
          ACTIVE_SENSOR_PROFILE.frequencyFields[0].address,
          ACTIVE_SENSOR_PROFILE.frequencyFields[1].address,
          ACTIVE_SENSOR_PROFILE.frequencyFields[2].address,
          (unsigned long long)m.rawFreqX,
          (unsigned long long)m.rawFreqY,
          (unsigned long long)m.rawFreqZ,
          m.freqX, m.freqY, m.freqZ,
          m.frequencySuspicious ? 1 : 0);
      }
    }
  }

  return m;
}

bool readSensorFastFrame(
  float &rawX, float &rawY, float &rawZ,
  SensorEngineeringMetrics &m
) {
  if (!ACTIVE_SENSOR_PROFILE.supportsContiguousFastFrame ||
      ACTIVE_SENSOR_PROFILE.fastFrameRegisterCount == 0 ||
      ACTIVE_SENSOR_PROFILE.fastFrameRegisterCount >
        ACTIVE_SENSOR_PROFILE.maximumReadRegisters ||
      ACTIVE_SENSOR_PROFILE.fastFrameRegisterCount > 125) {
    return false;
  }

  const uint16_t frameBase = ACTIVE_SENSOR_PROFILE.fastFrameStartRegister;
  const uint16_t frameCount = ACTIVE_SENSOR_PROFILE.fastFrameRegisterCount;
  uint8_t data[250];
  if (!readModbusRegistersFunction(
        ACTIVE_SENSOR_PROFILE.bulkReadFunction,
        frameBase, frameCount, data, sizeof(data), "bulkRead")) {
    return false;
  }

  DecodedProfileField acceleration[3] = {};
  DecodedProfileField velocity[3] = {};
  DecodedProfileField displacement[3] = {};
  DecodedProfileField frequency[3] = {};
  DecodedProfileField temperature = {};
  bool velocityDecoded = true;
  bool displacementDecoded = true;
  for (uint8_t axis = 0; axis < 3; axis++) {
    if (!decodeFieldFromBlock(
          ACTIVE_SENSOR_PROFILE.accelerationFields[axis],
          frameBase, frameCount, data, acceleration[axis])) {
      return false;
    }
    velocityDecoded =
      decodeFieldFromBlock(
        ACTIVE_SENSOR_PROFILE.velocityFields[axis],
        frameBase, frameCount, data, velocity[axis]) &&
      velocityDecoded;
    displacementDecoded =
      decodeFieldFromBlock(
        ACTIVE_SENSOR_PROFILE.displacementFields[axis],
        frameBase, frameCount, data, displacement[axis]) &&
      displacementDecoded;
    decodeFieldFromBlock(
      ACTIVE_SENSOR_PROFILE.frequencyFields[axis],
      frameBase, frameCount, data, frequency[axis]);
  }
  const bool temperatureDecoded =
    decodeFieldFromBlock(
      ACTIVE_SENSOR_PROFILE.temperatureField,
      frameBase, frameCount, data, temperature);
  rawX = (float)acceleration[0].value;
  rawY = (float)acceleration[1].value;
  rawZ = (float)acceleration[2].value;
  m = {};
  m.velocityValid = velocityDecoded;
  m.rawVelocityX = velocity[0].raw;
  m.rawVelocityY = velocity[1].raw;
  m.rawVelocityZ = velocity[2].raw;
  m.velocityRmsX = (float)velocity[0].value;
  m.velocityRmsY = (float)velocity[1].value;
  m.velocityRmsZ = (float)velocity[2].value;
  m.temperatureValid = temperatureDecoded;
  m.displacementValid = displacementDecoded;
  m.temperatureC = (float)temperature.value;
  m.rawDisplacementX = displacement[0].raw;
  m.rawDisplacementY = displacement[1].raw;
  m.rawDisplacementZ = displacement[2].raw;
  m.displacementX = (float)displacement[0].value;
  m.displacementY = (float)displacement[1].value;
  m.displacementZ = (float)displacement[2].value;
  m.rawFreqX = frequency[0].raw;
  m.rawFreqY = frequency[1].raw;
  m.rawFreqZ = frequency[2].raw;
  m.freqX = (float)frequency[0].value;
  m.freqY = (float)frequency[1].value;
  m.freqZ = (float)frequency[2].value;
  m.frequencyValid =
    (frequency[0].valid && isValidSensorFrequency(m.freqX)) ||
    (frequency[1].valid && isValidSensorFrequency(m.freqY)) ||
    (frequency[2].valid && isValidSensorFrequency(m.freqZ));
  m.frequencySuspicious =
    (m.freqX != 0.0f && !isValidSensorFrequency(m.freqX)) ||
    (m.freqY != 0.0f && !isValidSensorFrequency(m.freqY)) ||
    (m.freqZ != 0.0f && !isValidSensorFrequency(m.freqZ));
  m.dominantFreqX = m.freqX;
  m.dominantFreqY = m.freqY;
  m.dominantFreqZ = m.freqZ;
  return true;
}

bool readSensorClock(SensorClockTime &t) {
  if (!ACTIVE_SENSOR_PROFILE.supportsVendorClock ||
      ACTIVE_SENSOR_PROFILE.clockRegisterCount != 4) {
    t.valid = false;
    return false;
  }
  uint8_t timeData[8];
  if (!readModbusRegisters(ACTIVE_SENSOR_PROFILE.clockStartRegister, 4,
                           timeData, sizeof(timeData), "chipTime")) {
    t.valid = false;
    return false;
  }

  uint16_t yymm = ((uint16_t)timeData[0] << 8) | timeData[1];
  uint16_t ddhh = ((uint16_t)timeData[2] << 8) | timeData[3];
  uint16_t mmss = ((uint16_t)timeData[4] << 8) | timeData[5];
  t.month  = (uint8_t)(yymm >> 8);
  t.year   = (uint8_t)(yymm & 0xFF);
  t.hour   = (uint8_t)(ddhh >> 8);
  t.day    = (uint8_t)(ddhh & 0xFF);
  t.sec    = (uint8_t)(mmss >> 8);
  t.minute = (uint8_t)(mmss & 0xFF);
  t.ms     = ((uint16_t)timeData[6] << 8) | timeData[7];
  t.valid =
    t.month >= 1 && t.month <= 12 &&
    t.day >= 1 && t.day <= 31 &&
    t.hour <= 23 && t.minute <= 59 && t.sec <= 59 && t.ms <= 999;
  return t.valid;
}

bool calibrateSensorBaseline() {
  // [CALCULATIONS] [SENSOR PROFILE] [HIGH RISK]
  // Baseline and noise-floor values feed every dynamic acceleration result.
  // This routine performs blocking sensor reads; changing its duration or call
  // site can also affect web/MQTT responsiveness during initialization.
  if (!ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform) {
    baselineX = baselineY = baselineZ = 0.0f;
    noiseFloorX = noiseFloorY = noiseFloorZ = 0.0f;
    baselineCalibrated = true;
    Serial.println(
      "[Sensor] Baseline calibration bypassed: profile supplies a sensor-reported acceleration magnitude.");
    return true;
  }
  if (setupApBlockingStaAssociation()) {
    // Configuration access has priority. Calibration remains pending and will
    // resume after the technician disconnects from SoftAP.
    return false;
  }
  Serial.printf("[Sensor] Baseline calibration: collecting %u static samples. Keep sensor still.\n",
                SENSOR_BASELINE_SAMPLES);

  // ---- Pass 1: compute mean baseline ----
  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  uint16_t collected = 0;
  const uint32_t maxAttempts = SENSOR_BASELINE_SAMPLES * 5;
  uint32_t attempts = 0;

  while (collected < SENSOR_BASELINE_SAMPLES && attempts < maxAttempts) {
    feedWatchdog();
    if (setupApBlockingStaAssociation()) return false;
    attempts++;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (readSensorAccelerationRaw(ax, ay, az)) {
      sumX += ax; sumY += ay; sumZ += az;
      collected++;
    }
    delay(5);
  }

  if (collected < (SENSOR_BASELINE_SAMPLES / 2)) {
    // Calibration failed — use last raw reading if available, else hard fallback
    if (hasLastRaw) {
      baselineX = lastRawX;
      baselineY = lastRawY;
      baselineZ = lastRawZ;
    } else {
      baselineX = BASELINE_FALLBACK_X;
      baselineY = BASELINE_FALLBACK_Y;
      baselineZ = BASELINE_FALLBACK_Z;
    }
    // Conservative noise floor for fallback
    noiseFloorX = 0.15f;
    noiseFloorY = 0.15f;
    noiseFloorZ = 0.15f;
    baselineCalibrated = false;
    Serial.printf("[Sensor] WARNING: Calibration failed (%u/%u samples). Fallback baseline: X=%.4f Y=%.4f Z=%.4f  noiseFloor=%.4f m/s^2\n",
                  collected, SENSOR_BASELINE_SAMPLES, baselineX, baselineY, baselineZ, noiseFloorX);
    return false;
  }

  baselineX = (float)(sumX / collected);
  baselineY = (float)(sumY / collected);
  baselineZ = (float)(sumZ / collected);

  // ---- Pass 2: compute static RMS noise floor after baseline removal ----
  double sqSumX = 0.0, sqSumY = 0.0, sqSumZ = 0.0;
  uint16_t nf_collected = 0;
  attempts = 0;
  const uint16_t NF_SAMPLES = SENSOR_BASELINE_SAMPLES;

  while (nf_collected < NF_SAMPLES && attempts < (uint32_t)(NF_SAMPLES * 5)) {
    feedWatchdog();
    if (setupApBlockingStaAssociation()) return false;
    attempts++;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (readSensorAccelerationRaw(ax, ay, az)) {
      float dx = ax - baselineX;
      float dy = ay - baselineY;
      float dz = az - baselineZ;
      sqSumX += (double)dx * dx;
      sqSumY += (double)dy * dy;
      sqSumZ += (double)dz * dz;
      nf_collected++;
    }
    delay(5);
  }

  if (nf_collected > 4) {
    noiseFloorX = sqrtf((float)(sqSumX / nf_collected));
    noiseFloorY = sqrtf((float)(sqSumY / nf_collected));
    noiseFloorZ = sqrtf((float)(sqSumZ / nf_collected));
  } else {
    noiseFloorX = noiseFloorY = noiseFloorZ = 0.10f;
  }

  baselineCalibrated = true;
  Serial.printf("[Sensor] Baseline:   X=%.4f  Y=%.4f  Z=%.4f  m/s^2  (N=%u)\n",
                baselineX, baselineY, baselineZ, collected);
  Serial.printf("[Sensor] NoiseFloor: X=%.4f  Y=%.4f  Z=%.4f  m/s^2  (N=%u)\n",
                noiseFloorX, noiseFloorY, noiseFloorZ, nf_collected);
  return true;
}

bool computeVibrationRMS(
  float &rawX,  float &rawY,  float &rawZ,
  float &dynX,  float &dynY,  float &dynZ,
  float &rmsX,  float &rmsY,  float &rmsZ,
  float &displayRmsX, float &displayRmsY, float &displayRmsZ,
  float &peakX, float &peakY, float &peakZ,
  float &p2pX,  float &p2pY,  float &p2pZ,
  float &stdX,  float &stdY,  float &stdZ,
  uint16_t &sampleCount,
  unsigned long &windowMs
) {
  // [CALCULATIONS] LEGACY/UNUSED: main.cpp currently has no call site for this
  // helper, so editing it cannot affect the present runtime unless it is wired
  // back into acquisition. If re-enabled, its RMS/peak/P2P/std-dev semantics
  // would be [HIGH RISK] for historical comparability and alarm interpretation.
  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  double sumSqX = 0.0, sumSqY = 0.0, sumSqZ = 0.0;
  peakX = 0.0f; peakY = 0.0f; peakZ = 0.0f;
  p2pX = p2pY = p2pZ = 0.0f;
  stdX = stdY = stdZ = 0.0f;
  sampleCount = 0;
  rawX = 0.0f; rawY = 0.0f; rawZ = 0.0f;
  dynX = 0.0f; dynY = 0.0f; dynZ = 0.0f;
  float minX = 0.0f, minY = 0.0f, minZ = 0.0f;
  float maxX = 0.0f, maxY = 0.0f, maxZ = 0.0f;

  // Keep MQTT alive before the blocking window
  if (mqttClient.connected()) mqttClient.loop();

  const unsigned long targetWindowMs = rmsWindowMsForBaud(sensorBaud);
  unsigned long startMs = millis();
  uint32_t nextSampleAtUs = micros();

  while ((millis() - startMs) < targetWindowMs) {
    const int32_t waitUs = (int32_t)(nextSampleAtUs - micros());
    if (waitUs > 0) {
      delayMicroseconds((uint32_t)waitUs);
    }
    nextSampleAtUs += TARGET_SAMPLE_PERIOD_US;

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (readSensorAccelerationRaw(ax, ay, az)) {
      // Save last raw reading (latest snapshot for the payload)
      rawX = ax; rawY = ay; rawZ = az;
      lastRawX = ax; lastRawY = ay; lastRawZ = az;
      hasLastRaw = true;

      dynX = ax - baselineX;
      dynY = ay - baselineY;
      dynZ = az - baselineZ;
      appendWaveformSample(ax, ay, az, dynX, dynY, dynZ);

      sumX += dynX;
      sumY += dynY;
      sumZ += dynZ;
      sumSqX += (double)dynX * dynX;
      sumSqY += (double)dynY * dynY;
      sumSqZ += (double)dynZ * dynZ;

      float absX = fabsf(dynX);
      float absY = fabsf(dynY);
      float absZ = fabsf(dynZ);
      if (absX > peakX) peakX = absX;
      if (absY > peakY) peakY = absY;
      if (absZ > peakZ) peakZ = absZ;

      if (sampleCount == 0) {
        minX = maxX = dynX;
        minY = maxY = dynY;
        minZ = maxZ = dynZ;
      } else {
        if (dynX < minX) minX = dynX;
        if (dynY < minY) minY = dynY;
        if (dynZ < minZ) minZ = dynZ;
        if (dynX > maxX) maxX = dynX;
        if (dynY > maxY) maxY = dynY;
        if (dynZ > maxZ) maxZ = dynZ;
      }

      sampleCount++;
    }
    // Pace acquisition at TARGET_SAMPLE_RATE_HZ. If Modbus reads take longer
    // than the requested period, actualSampleRateHz will show the achieved rate.
  }

  windowMs = millis() - startMs;

  // Keep MQTT alive after the blocking window
  if (mqttClient.connected()) mqttClient.loop();

  if (sampleCount == 0) {
    rmsX = rmsY = rmsZ = 0.0f;
    displayRmsX = displayRmsY = displayRmsZ = 0.0f;
    dynX = dynY = dynZ = 0.0f;
    p2pX = p2pY = p2pZ = 0.0f;
    stdX = stdY = stdZ = 0.0f;
    static unsigned long lastLowSampleWarnMs = 0;
    unsigned long now = millis();
    if (now - lastLowSampleWarnMs >= SENSOR_LOG_INTERVAL_MS) {
      lastLowSampleWarnMs = now;
      Serial.printf("[Sensor] WARNING: no valid samples in %lu ms window — Modbus read failed.\n",
                    windowMs);
    }
    return false;
  }

  // True dynamic RMS (gravity + offset removed)
  rmsX = sqrtf((float)(sumSqX / sampleCount));
  rmsY = sqrtf((float)(sumSqY / sampleCount));
  rmsZ = sqrtf((float)(sumSqZ / sampleCount));
  p2pX = maxX - minX;
  p2pY = maxY - minY;
  p2pZ = maxZ - minZ;

  double meanX = sumX / sampleCount;
  double meanY = sumY / sampleCount;
  double meanZ = sumZ / sampleCount;
  stdX = sqrtf(fmaxf(0.0f, (float)(sumSqX / sampleCount - meanX * meanX)));
  stdY = sqrtf(fmaxf(0.0f, (float)(sumSqY / sampleCount - meanY * meanY)));
  stdZ = sqrtf(fmaxf(0.0f, (float)(sumSqZ / sampleCount - meanZ * meanZ)));

  if (sampleCount < 4) {
    static unsigned long lastSparseSampleWarnMs = 0;
    unsigned long now = millis();
    if (now - lastSparseSampleWarnMs >= SENSOR_LOG_INTERVAL_MS) {
      lastSparseSampleWarnMs = now;
      Serial.printf("[Sensor] WARNING: only %u samples in %lu ms window at %lu baud — RMS is sparse but Modbus is OK.\n",
                    sampleCount, windowMs, (unsigned long)sensorBaud);
    }
  }

  if (SENSOR_DIAGNOSTIC_MODE) {
    displayRmsX = rmsX;
    displayRmsY = rmsY;
    displayRmsZ = rmsZ;
  } else {
    float nfX = noiseFloorX * NOISE_FLOOR_MULTIPLIER;
    float nfY = noiseFloorY * NOISE_FLOOR_MULTIPLIER;
    float nfZ = noiseFloorZ * NOISE_FLOOR_MULTIPLIER;
    displayRmsX = sqrtf(fmaxf(0.0f, rmsX*rmsX - nfX*nfX));
    displayRmsY = sqrtf(fmaxf(0.0f, rmsY*rmsY - nfY*nfY));
    displayRmsZ = sqrtf(fmaxf(0.0f, rmsZ*rmsZ - nfZ*nfZ));

    if (displayRmsX < RMS_DEADBAND_MS2) displayRmsX = 0.0f;
    if (displayRmsY < RMS_DEADBAND_MS2) displayRmsY = 0.0f;
    if (displayRmsZ < RMS_DEADBAND_MS2) displayRmsZ = 0.0f;

    if ((rmsX > 0.0f && displayRmsX == 0.0f) ||
        (rmsY > 0.0f && displayRmsY == 0.0f) ||
        (rmsZ > 0.0f && displayRmsZ == 0.0f)) {
      static unsigned long lastDeadbandWarnMs = 0;
      unsigned long now = millis();
      if (now - lastDeadbandWarnMs >= SENSOR_LOG_INTERVAL_MS) {
        lastDeadbandWarnMs = now;
        Serial.printf("[Sensor] WARNING: deadband/noise floor suppressed measured RMS. rms=(%.5f,%.5f,%.5f) display=(%.5f,%.5f,%.5f)\n",
                      rmsX, rmsY, rmsZ, displayRmsX, displayRmsY, displayRmsZ);
      }
    }
  }

  return true;
}

void publishData() {
  // [SENSOR PROFILE] Reads profile-defined fields from the live Modbus bus.
  // [CALCULATIONS] Produces quality, RMS/overall, confidence, and alarm values.
  // [MQTT] Serializes and publishes the resulting telemetry document.
  // [HIGH RISK] This combines all three domains; changing fallback semantics,
  // validity gates, units, or payload keys can cause false sensor_fault states
  // or silently break downstream dashboards and stored data.
  float rawX       = 0.0f, rawY       = 0.0f, rawZ       = 0.0f;
  float dynX       = 0.0f, dynY       = 0.0f, dynZ       = 0.0f;
  float rmsX       = 0.0f, rmsY       = 0.0f, rmsZ       = 0.0f;
  float displayRmsX = 0.0f, displayRmsY = 0.0f, displayRmsZ = 0.0f;
  float peakX      = 0.0f, peakY      = 0.0f, peakZ      = 0.0f;
  float p2pX       = 0.0f, p2pY       = 0.0f, p2pZ       = 0.0f;
  float stdX       = 0.0f, stdY       = 0.0f, stdZ       = 0.0f;
  uint16_t sampleCount              = 0;
  unsigned long windowMs            = 0;
  SensorEngineeringMetrics eng      = {};

  const unsigned long acquisitionStartMs = millis();
  if (lastSensorPollStartMs > 0) {
    lastActualSensorPollIntervalMs = acquisitionStartMs - lastSensorPollStartMs;
  }
  lastSensorPollStartMs = acquisitionStartMs;
  const bool retryFastFrame = ACTIVE_SENSOR_PROFILE.supportsContiguousFastFrame &&
    (!useSplitModbusReads ||
     acquisitionStartMs - lastFastFrameAttemptMs >= FAST_FRAME_RETRY_INTERVAL_MS);
  bool ok = false;
  bool skipSplitThisCycle = false;

  if (!modbusRecoveryActive() && retryFastFrame) {
    lastFastFrameAttemptMs = acquisitionStartMs;
    ok = readSensorFastFrame(rawX, rawY, rawZ, eng);
    if (ok) {
      consecutiveFastFrameFailures = 0;
      useSplitModbusReads = false;
    } else {
      consecutiveFastFrameFailures++;
      if (consecutiveFastFrameFailures >= FAST_FRAME_FAILURES_BEFORE_FALLBACK) {
        useSplitModbusReads = true;
      }
      skipSplitThisCycle = true;
    }
  }

  // Some sensors answer small register requests but reject a long contiguous
  // frame. Fall back to profile-defined register groups.
  if (!ok && !skipSplitThisCycle && !modbusRecoveryActive()) {
    ok = readSensorAccelerationRaw(rawX, rawY, rawZ);
    if (ok) {
      eng = readSensorEngineeringMetrics();
    }
  }

  if (!ok) {
    if (sensorReadFailCount < UINT8_MAX) sensorReadFailCount++;
    if (sensorReadFailCount >= 10) {
      sensorReadFailCount = 0;
      static unsigned long lastLiveReadFailLogMs = 0;
      unsigned long now = millis();
      if (now - lastLiveReadFailLogMs >= SENSOR_LOG_INTERVAL_MS) {
        lastLiveReadFailLogMs = now;
        Serial.println("[Sensor] Bulk and split Modbus reads are failing; publishing an invalid sensor window.");
      }
    }
    // Fall back to last known raw values if available
    if (hasLastRaw) {
      rawX = lastRawX; rawY = lastRawY; rawZ = lastRawZ;
      dynX = ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform
        ? rawX - baselineX : rawX;
      dynY = ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform
        ? rawY - baselineY : rawY;
      dynZ = ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform
        ? rawZ - baselineZ : rawZ;
      rmsX = fabsf(dynX);
      rmsY = fabsf(dynY);
      rmsZ = fabsf(dynZ);
      displayRmsX = rmsX;
      displayRmsY = rmsY;
      displayRmsZ = rmsZ;
    }
  } else {
    sensorReadFailCount = 0;
    lastSensorReadOkMs = millis();
    hasLastRaw = true;
    lastRawX = rawX;
    lastRawY = rawY;
    lastRawZ = rawZ;
    dynX = ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform
      ? rawX - baselineX : rawX;
    dynY = ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform
      ? rawY - baselineY : rawY;
    dynZ = ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform
      ? rawZ - baselineZ : rawZ;
    rmsX = fabsf(dynX);
    rmsY = fabsf(dynY);
    rmsZ = fabsf(dynZ);
    displayRmsX = rmsX;
    displayRmsY = rmsY;
    displayRmsZ = rmsZ;
    peakX = rmsX;
    peakY = rmsY;
    peakZ = rmsZ;
    if (!ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform) {
      // These registers are already sensor-computed magnitudes. Do not
      // baseline-subtract, re-window, or claim waveform-derived peak/P2P data.
      sampleCount = 1;
      waveWriteIndex = 0;
      waveSamplesStored = 0;
    } else {
      appendWaveformSample(rawX, rawY, rawZ, dynX, dynY, dynZ);
    }
    if (ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform &&
        waveSamplesStored > 0) {
      double sumSqX = 0.0, sumSqY = 0.0, sumSqZ = 0.0;
      double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
      float minX = dynWaveX[0], maxX = dynWaveX[0];
      float minY = dynWaveY[0], maxY = dynWaveY[0];
      float minZ = dynWaveZ[0], maxZ = dynWaveZ[0];
      peakX = 0.0f; peakY = 0.0f; peakZ = 0.0f;
      for (uint16_t i = 0; i < waveSamplesStored; i++) {
        const float wx = dynWaveX[i];
        const float wy = dynWaveY[i];
        const float wz = dynWaveZ[i];
        sumX += wx; sumY += wy; sumZ += wz;
        sumSqX += (double)wx * wx;
        sumSqY += (double)wy * wy;
        sumSqZ += (double)wz * wz;
        if (wx < minX) minX = wx;
        if (wx > maxX) maxX = wx;
        if (wy < minY) minY = wy;
        if (wy > maxY) maxY = wy;
        if (wz < minZ) minZ = wz;
        if (wz > maxZ) maxZ = wz;
        const float absX = fabsf(wx);
        const float absY = fabsf(wy);
        const float absZ = fabsf(wz);
        if (absX > peakX) peakX = absX;
        if (absY > peakY) peakY = absY;
        if (absZ > peakZ) peakZ = absZ;
      }
      const float n = (float)waveSamplesStored;
      rmsX = sqrtf((float)(sumSqX / n));
      rmsY = sqrtf((float)(sumSqY / n));
      rmsZ = sqrtf((float)(sumSqZ / n));
      p2pX = maxX - minX;
      p2pY = maxY - minY;
      p2pZ = maxZ - minZ;
      const double meanX = sumX / n;
      const double meanY = sumY / n;
      const double meanZ = sumZ / n;
      stdX = sqrtf(fmaxf(0.0f, (float)(sumSqX / n - meanX * meanX)));
      stdY = sqrtf(fmaxf(0.0f, (float)(sumSqY / n - meanY * meanY)));
      stdZ = sqrtf(fmaxf(0.0f, (float)(sumSqZ / n - meanZ * meanZ)));
      displayRmsX = rmsX;
      displayRmsY = rmsY;
      displayRmsZ = rmsZ;
    }
    // The published RMS above is computed from the rolling waveform, so report
    // the number of samples that actually contributed instead of hard-coding 1.
    if (ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform) {
      sampleCount = waveSamplesStored;
    }
  }

  bool sensorOk = sampleCount > 0;
  windowMs = lastAcquisitionStartMs > 0
    ? acquisitionStartMs - lastAcquisitionStartMs
    : sensorPollIntervalMs;
  if (windowMs == 0) windowMs = 1;
  lastAcquisitionStartMs = acquisitionStartMs;
  bool rmsReliable = sensorOk;
  float actualSampleRateHz = 1000.0f / (float)windowMs;
  const float targetSampleRateHz =
    1000.0f / (float)max(1UL, sensorPollIntervalMs);
  unsigned long sensorReadStartedMs = millis();

  // Read on-chip RTC timestamp at most once every 5 s — reading every publish cycle
  // risks a delayed sensor response arriving mid-accel-read and corrupting the UART buffer.
  if (sensorReadStartedMs - lastChipTimeReadMs >= CHIP_TIME_READ_INTERVAL_MS) {
    lastChipTimeReadMs = sensorReadStartedMs;
    readSensorClock(latestChipTime); // flushes + settles bus internally
  }

  if (mqttClient.connected()) mqttClient.loop();

  // Use one acquisition timestamp for every metric in this published sample.
  unsigned long ts = millis();
  const uint64_t ts64 = monotonicMillis();
  char timestampBuffer[24];
  snprintf(timestampBuffer, sizeof(timestampBuffer), "%llu",
           (unsigned long long)ts64);
  bool shouldLogSensor = (ts - lastSensorLogMs >= SENSOR_LOG_INTERVAL_MS);
  if (shouldLogSensor) {
    lastSensorLogMs = ts;
  }
  float accelOverall = sqrtf(rmsX * rmsX + rmsY * rmsY + rmsZ * rmsZ);
  bool accelerationValid = ok && sensorOk &&
    isFiniteSensorFloat(rmsX) &&
    isFiniteSensorFloat(rmsY) &&
    isFiniteSensorFloat(rmsZ) &&
    isFiniteSensorFloat(accelOverall);
  float velocityOverall = eng.velocityValid
    ? sqrtf(eng.velocityRmsX * eng.velocityRmsX +
            eng.velocityRmsY * eng.velocityRmsY +
            eng.velocityRmsZ * eng.velocityRmsZ)
    : 0.0f;
  float displacementOverall = eng.displacementValid
    ? sqrtf(eng.displacementX * eng.displacementX +
            eng.displacementY * eng.displacementY +
            eng.displacementZ * eng.displacementZ)
    : 0.0f;
  float freqOverall = 0.0f;
  // Profile-defined frequency fields are sensor-reported values. Never
  // suppress them based on a firmware velocity heuristic; consumers may
  // apply sensor-specific policy.
  if (eng.frequencyValid) {
    float fx = isValidSensorFrequency(eng.freqX) ? eng.freqX : 0.0f;
    float fy = isValidSensorFrequency(eng.freqY) ? eng.freqY : 0.0f;
    float fz = isValidSensorFrequency(eng.freqZ) ? eng.freqZ : 0.0f;
    if (eng.velocityValid) {
      const float vx = fabsf(eng.velocityRmsX);
      const float vy = fabsf(eng.velocityRmsY);
      const float vz = fabsf(eng.velocityRmsZ);
      float strongestValidVelocity = -1.0f;
      if (fx > 0.0f && vx > strongestValidVelocity) { strongestValidVelocity = vx; freqOverall = fx; }
      if (fy > 0.0f && vy > strongestValidVelocity) { strongestValidVelocity = vy; freqOverall = fy; }
      if (fz > 0.0f && vz > strongestValidVelocity) { freqOverall = fz; }
    } else {
      freqOverall = fmaxf(fx, fmaxf(fy, fz));
    }
  }
  uint8_t confidence = computeDiagnosticConfidence(sampleCount, windowMs, sensorOk, rmsReliable);
  String healthStatus = healthStatusFromVelocity(velocityOverall, eng.velocityValid, confidence);
  String alarmReason;
  String triggeredMetric = "null";
  String alarmThreshold = "null";
  String alarmCurrentValue = "null";
  int alarmSeverity = 0;
  if (confidence < 45) {
    alarmSeverity = 2;
    alarmReason = "Low confidence - check sensor communication, mounting, baud rate, or sample rate";
    triggeredMetric = "\"quality.confidence\"";
    alarmThreshold = "45";
    alarmCurrentValue = String(confidence);
  } else if (ACTIVE_SENSOR_PROFILE.velocityIsRms &&
             eng.velocityValid && velocityOverall >= VELOCITY_CRITICAL_MM_S) {
    alarmSeverity = 3;
    alarmReason = "Velocity RMS is above critical threshold";
    triggeredMetric = "\"vibration.overall.velocityRms\"";
    alarmThreshold = String(VELOCITY_CRITICAL_MM_S, 2);
    alarmCurrentValue = String(velocityOverall, 2);
  } else if (ACTIVE_SENSOR_PROFILE.velocityIsRms &&
             eng.velocityValid && velocityOverall >= VELOCITY_ALARM_MM_S) {
    alarmSeverity = 2;
    alarmReason = "Velocity RMS is above alarm threshold";
    triggeredMetric = "\"vibration.overall.velocityRms\"";
    alarmThreshold = String(VELOCITY_ALARM_MM_S, 2);
    alarmCurrentValue = String(velocityOverall, 2);
  } else if (ACTIVE_SENSOR_PROFILE.velocityIsRms &&
             eng.velocityValid && velocityOverall >= VELOCITY_ALERT_MM_S) {
    alarmSeverity = 1;
    alarmReason = "Velocity RMS is above alert threshold";
    triggeredMetric = "\"vibration.overall.velocityRms\"";
    alarmThreshold = String(VELOCITY_ALERT_MM_S, 2);
    alarmCurrentValue = String(velocityOverall, 2);
  } else if (eng.velocityValid && ACTIVE_SENSOR_PROFILE.velocityIsRms) {
    alarmReason = "Velocity RMS below alert threshold";
  } else if (eng.velocityValid) {
    alarmReason =
      "Sensor velocity statistic is unspecified; RMS alarm thresholds are disabled";
  } else {
    alarmReason = "Sensor velocity registers unavailable; using acceleration diagnostics only";
  }

  if (VERBOSE_SERIAL && shouldLogSensor) {
    Serial.printf("[Sensor] Config:     baud=%lu id=0x%02X fcode=0x%02X accelReg=0x%04X diag=%d deadband=%.5f nfMult=%.2f\n",
                  (unsigned long)sensorBaud, sensorModbusId, sensorReadFunction,
                  sensorAccelRegBase, SENSOR_DIAGNOSTIC_MODE ? 1 : 0,
                  RMS_DEADBAND_MS2, NOISE_FLOOR_MULTIPLIER);
    Serial.printf("[Sensor] Baseline:   X=%.4f  Y=%.4f  Z=%.4f  m/s^2 | noiseFloor X=%.5f Y=%.5f Z=%.5f\n",
                  baselineX, baselineY, baselineZ, noiseFloorX, noiseFloorY, noiseFloorZ);
    Serial.printf("[Sensor] Dynamic:    X=%.5f  Y=%.5f  Z=%.5f  m/s^2\n", dynX, dynY, dynZ);
    Serial.printf("[Sensor] Peak/P2P:   peak=(%.5f,%.5f,%.5f) p2p=(%.5f,%.5f,%.5f) std=(%.5f,%.5f,%.5f)\n",
                  peakX, peakY, peakZ, p2pX, p2pY, p2pZ, stdX, stdY, stdZ);
    Serial.printf("[Sensor] Velocity:   rms=(%.2f,%.2f,%.2f) overall=%.2f mm/s valid=%d status=%s confidence=%u\n",
                  eng.velocityRmsX, eng.velocityRmsY, eng.velocityRmsZ, velocityOverall,
                  eng.velocityValid ? 1 : 0, healthStatus.c_str(), confidence);
    Serial.printf("[Sensor] Frequency:  raw XYZ=(%llu,%llu,%llu) -> (%.1f,%.1f,%.1f) Hz | velOverall=%.2f mm/s\n",
                  (unsigned long long)eng.rawFreqX,
                  (unsigned long long)eng.rawFreqY,
                  (unsigned long long)eng.rawFreqZ,
                  eng.freqX, eng.freqY, eng.freqZ, velocityOverall);
    Serial.printf("[Sensor] Temp/Disp:  temp=%.2f C disp=(%.1f,%.1f,%.1f) um\n",
                  eng.temperatureC, eng.displacementX, eng.displacementY, eng.displacementZ);
    Serial.printf("[Sensor] FastFrame:  N=%u interval=%lu ms actualRate=%.2f Hz reliable=%d modbusFail=%lu\n",
                  sampleCount, windowMs, actualSampleRateHz, rmsReliable ? 1 : 0,
                  (unsigned long)modbusFailCount);
    Serial.printf("[Sensor] Raw:        ax=%.4f  ay=%.4f  az=%.4f  m/s^2\n", rawX, rawY, rawZ);
    Serial.printf("[Sensor] TrueRMS:    X=%.5f  Y=%.5f  Z=%.5f  m/s^2 | peak X=%.5f Y=%.5f Z=%.5f\n",
                  rmsX, rmsY, rmsZ, peakX, peakY, peakZ);
    Serial.printf("[Sensor] DisplayRMS: X=%.5f  Y=%.5f  Z=%.5f  m/s^2 | N=%u  interval=%lu ms\n",
                  displayRmsX, displayRmsY, displayRmsZ, sampleCount, windowMs);
    if (actualSampleRateHz < (targetSampleRateHz * 0.8f)) {
      Serial.printf("[Sensor] WARNING: achieved %.2f Hz below %.2f Hz target. Check baud, MQTT latency, and RS485 wiring.\n",
                    actualSampleRateHz, targetSampleRateHz);
    }
  }

  // Sensor acquisition/calculation runs independently of MQTT. Do not build a
  // multi-kilobyte JSON document when the network cannot carry it, and do not
  // serialize/transmit it faster than the explicit telemetry limit.
  // A client may remain associated with the recovery AP for hours. Only the
  // bounded 20-second active-setup grace may pause telemetry; raw station count
  // must never create an AP-vs-MQTT deadlock. A broker connection alone is not
  // useful: skip the multi-kilobyte JSON while the Node application heartbeat
  // is absent, but keep the MQTT subscription/keepalive and 2 s sensor health
  // reads alive so recovery is detected without wasting radio/CPU energy.
  const bool linkReady =
    (WiFi.status() == WL_CONNECTED) && mqttClient.connected() &&
    nodeServerHeartbeatFresh(ts) && !setupApBlockingStaAssociation();
  if (!linkReady) {
    if (shouldLogSensor) {
      Serial.println("[MQTT] Skipping payload construction; STA/MQTT not ready.");
    }
    return;
  }
  if (ts - lastTelemetryAttemptMs < MQTT_PUBLISH_PERIOD_MS) {
    return;
  }
  lastTelemetryAttemptMs = ts;

  // Clean per-device topic (server also accepts legacy factory/vibration/<id>).
  String topic = "sensors/" + deviceId + "/measurements";
  const char *wifiState   = (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
  const char *mqttState   = mqttClient.connected() ? "connected" : "disconnected";
  const bool recoveringModbus = modbusRecoveryActive();
  const char *sensorState = ok ? "online" : (recoveringModbus ? "offline" : (hasLastRaw ? "stale" : "timeout"));
  const char *dataQuality = ok ? "valid"  : (recoveringModbus ? "invalid" : (hasLastRaw ? "stale" : "invalid"));
  char modbusIdBuf[7];
  snprintf(modbusIdBuf, sizeof(modbusIdBuf), "0x%02X", sensorModbusId);
  {
    static String payload;   // reused across publishes: avoids per-cycle heap realloc
    if (!payload.reserve(3600)) {
      saturatingAdd(publishFailCount);
      saturatingAdd(recentPublishFails);
      telemetryMemoryHealthy = false;
      return;
    }
    payload = "{";
    payload += "\"deviceId\":\"" + deviceId + "\",";
    payload += "\"identificationName\":\"" + jsonEscape(identificationName) + "\",";
    payload += "\"sensorType\":\"" + jsonEscape(ACTIVE_SENSOR_PROFILE.sensorType) + "\",";
    payload += "\"source\":\"edge_firmware\",";
    payload += "\"metadata\":{";
    payload += "\"name\":\"" + jsonEscape(identificationName) + "\",";
    payload += "\"identificationName\":\"" + jsonEscape(identificationName) + "\",";
    payload += "\"firmwareName\":\"" + jsonEscape(FIRMWARE_NAME) + "\",";
    payload += "\"firmwareVersion\":\"" + jsonEscape(FIRMWARE_VERSION) + "\",";
    payload += "\"sensorModel\":\"" + jsonEscape(SENSOR_MODEL_NAME) + "\",";
    payload += "\"sensorProfile\":\"" + jsonEscape(ACTIVE_SENSOR_PROFILE.profileKey) + "\",";
    payload += "\"sensorProfileSource\":\"" +
               String(sensorProfileLoadedFromStorage ? "uploaded" : "built_in") + "\",";
    payload += "\"registerMapVersion\":\"" + jsonEscape(SENSOR_REGISTER_MAP_VERSION) + "\",";
    payload += "\"protocol\":\"" + jsonEscape(DEVICE_PROTOCOL_NAME) + "\",";
    payload += "\"ipAddress\":\"" + jsonEscape(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\",";
    payload += "\"source\":\"edge_firmware\"";
    payload += "},";
    payload += "\"status\":{\"wifi\":\"" + String(wifiState) + "\",\"mqtt\":\"" + String(mqttState) +
               "\",\"sensor\":\"" + String(sensorState) + "\",\"dataQuality\":\"" + String(dataQuality) + "\"},";
    payload += "\"timestamp\":";
    payload += timestampBuffer;
    payload += ",\"timestampSource\":\"monotonic_uptime_ms64\",";
    payload += "\"sensorOk\":" + String(sensorOk ? "true" : "false") + ",";
    payload += "\"healthStatus\":\"" + healthStatus + "\",";
    payload += "\"confidence\":" + String(confidence) + ",";
    payload += "\"sampleCount\":" + String(sampleCount) + ",";
    payload += "\"rmsWindowSamples\":" + String(waveSamplesStored) + ",";
    payload += "\"windowMs\":" + String(windowMs) + ",";
    payload += "\"actualSampleRateHz\":" + String(actualSampleRateHz, 2) + ",";
    payload += "\"targetSampleRateHz\":" + String(targetSampleRateHz, 2) + ",";
    payload += "\"sampleRateHz\":" + String(actualSampleRateHz, 2) + ",";
    payload += "\"sensorBaud\":" + String(sensorBaud) + ",";
    payload += "\"sensorTxGpio\":" + String(UART2_TX_PIN) + ",";
    payload += "\"sensorRxGpio\":" + String(UART2_RX_PIN) + ",";
    payload += "\"modbusFailCount\":" + String(modbusFailCount) + ",";
    payload += "\"crcFailCount\":" + String(modbusCrcFailCount) + ",";
    payload += "\"timeoutCount\":" + String(modbusTimeoutCount) + ",";
    payload += "\"quality\":{";
    payload += "\"sampleCount\":" + String(sampleCount) + ",";
    payload += "\"rmsWindowSamples\":" + String(waveSamplesStored) + ",";
    payload += "\"windowMs\":" + String(windowMs) + ",";
    payload += "\"actualSampleRateHz\":" + String(actualSampleRateHz, 2) + ",";
    payload += "\"targetSampleRateHz\":" + String(targetSampleRateHz, 2) + ",";
    payload += "\"modbusFailCount\":" + String(modbusFailCount) + ",";
    payload += "\"crcFailCount\":" + String(modbusCrcFailCount) + ",";
    payload += "\"timeoutCount\":" + String(modbusTimeoutCount) + ",";
    payload += "\"modbusSuccessCount\":" + String(modbusSuccessCount) + ",";
    payload += "\"modbusEchoCount\":" + String(modbusEchoCount) + ",";
    payload += "\"modbusDiscardedNoiseBytes\":" + String(modbusNoiseByteCount) + ",";
    payload += "\"consecutiveModbusFailures\":" + String(consecutiveModbusFailures) + ",";
    payload += "\"lastModbusError\":\"" + jsonEscape(String(lastModbusErrorReason)) + "\",";
    payload += "\"modbusRecoveryActive\":" + String(modbusRecoveryActive() ? "true" : "false") + ",";
    payload += "\"modbusRecoveryRemainingMs\":" + String(modbusRecoveryRemainingMs()) + ",";
    payload += "\"sensorPollIntervalMs\":" + String(sensorPollIntervalMs) + ",";
    payload += "\"actualPollIntervalMs\":" + String(lastActualSensorPollIntervalMs) + ",";
    payload += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    payload += "\"confidence\":" + String(confidence) + ",";
    payload += "\"rmsReliable\":" + String(rmsReliable ? "true" : "false");
    payload += "},";
    payload += "\"measurementQuality\":{";
    payload += "\"velocity\":{";
    payload += "\"unit\":\"mm/s\",";
    payload += "\"resolution\":" + String(SENSOR_VELOCITY_LSB_MM_S, 3) + ",";
    payload += "\"zeroMeaning\":\"below_sensor_resolution_or_no_motion\",";
    payload += "\"x\":\"" + String(sensorAxisQualityFlag(eng.velocityValid, eng.velocityRmsX, SENSOR_VELOCITY_LSB_MM_S)) + "\",";
    payload += "\"y\":\"" + String(sensorAxisQualityFlag(eng.velocityValid, eng.velocityRmsY, SENSOR_VELOCITY_LSB_MM_S)) + "\",";
    payload += "\"z\":\"" + String(sensorAxisQualityFlag(eng.velocityValid, eng.velocityRmsZ, SENSOR_VELOCITY_LSB_MM_S)) + "\",";
    payload += "\"magnitude\":\"" + String(sensorAxisQualityFlag(eng.velocityValid, velocityOverall, SENSOR_VELOCITY_LSB_MM_S)) + "\"";
    payload += "},";
    payload += "\"displacement\":{";
    payload += "\"unit\":\"um\",";
    payload += "\"resolution\":" + String(SENSOR_DISPLACEMENT_LSB_UM, 3) + ",";
    payload += "\"zeroMeaning\":\"below_sensor_resolution_or_no_motion\",";
    payload += "\"x\":\"" + String(sensorAxisQualityFlag(eng.displacementValid, eng.displacementX, SENSOR_DISPLACEMENT_LSB_UM)) + "\",";
    payload += "\"y\":\"" + String(sensorAxisQualityFlag(eng.displacementValid, eng.displacementY, SENSOR_DISPLACEMENT_LSB_UM)) + "\",";
    payload += "\"z\":\"" + String(sensorAxisQualityFlag(eng.displacementValid, eng.displacementZ, SENSOR_DISPLACEMENT_LSB_UM)) + "\",";
    payload += "\"magnitude\":\"" + String(sensorAxisQualityFlag(eng.displacementValid, displacementOverall, SENSOR_DISPLACEMENT_LSB_UM)) + "\"";
    payload += "},";
    payload += "\"frequency\":{";
    payload += "\"unit\":\"Hz\",";
    payload += "\"resolution\":" + String(SENSOR_FREQUENCY_SCALE, 3) + ",";
    payload += "\"x\":\"" + String(sensorAxisQualityFlag(eng.frequencyValid && isValidSensorFrequency(eng.freqX), eng.freqX, SENSOR_FREQUENCY_SCALE)) + "\",";
    payload += "\"y\":\"" + String(sensorAxisQualityFlag(eng.frequencyValid && isValidSensorFrequency(eng.freqY), eng.freqY, SENSOR_FREQUENCY_SCALE)) + "\",";
    payload += "\"z\":\"" + String(sensorAxisQualityFlag(eng.frequencyValid && isValidSensorFrequency(eng.freqZ), eng.freqZ, SENSOR_FREQUENCY_SCALE)) + "\",";
    payload += "\"magnitude\":\"" + String(sensorAxisQualityFlag(eng.frequencyValid, freqOverall, SENSOR_FREQUENCY_SCALE)) + "\"";
    payload += "}";
    payload += "},";
    payload += "\"rawRegisters\":{";
    payload += "\"velocity\":{\"x\":" + uint64Json(eng.rawVelocityX) + ",\"y\":" + uint64Json(eng.rawVelocityY) + ",\"z\":" + uint64Json(eng.rawVelocityZ) + "},";
    payload += "\"displacement\":{\"x\":" + uint64Json(eng.rawDisplacementX) + ",\"y\":" + uint64Json(eng.rawDisplacementY) + ",\"z\":" + uint64Json(eng.rawDisplacementZ) + "},";
    payload += "\"frequency\":{\"x\":" + uint64Json(eng.rawFreqX) + ",\"y\":" + uint64Json(eng.rawFreqY) + ",\"z\":" + uint64Json(eng.rawFreqZ) + "}";
    payload += "},";
    payload += "\"measurements\":{";
    payload += "\"acceleration\":{\"unit\":\"m/s^2\",";
    payload += "\"source\":\"" +
      String(ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform
        ? "firmware_calculated_rms" : "sensor_reported_magnitude") + "\",";
    payload += "\"x\":" + jsonNumberOrNull(accelerationValid, rmsX, 5) + ",";
    payload += "\"y\":" + jsonNumberOrNull(accelerationValid, rmsY, 5) + ",";
    payload += "\"z\":" + jsonNumberOrNull(accelerationValid, rmsZ, 5) + ",";
    payload += "\"magnitude\":" + jsonNumberOrNull(accelerationValid, accelOverall, 5) + "},";
    payload += "\"velocity\":{\"unit\":\"mm/s\",\"source\":\"" +
      String(ACTIVE_SENSOR_PROFILE.velocityIsRms
        ? "sensor_reported_rms" : "sensor_reported_unspecified") + "\",";
    payload += "\"x\":" + sensorAxisJsonValue(eng.velocityValid, eng.velocityRmsX, SENSOR_VELOCITY_LSB_MM_S, 3) + ",";
    payload += "\"y\":" + sensorAxisJsonValue(eng.velocityValid, eng.velocityRmsY, SENSOR_VELOCITY_LSB_MM_S, 3) + ",";
    payload += "\"z\":" + sensorAxisJsonValue(eng.velocityValid, eng.velocityRmsZ, SENSOR_VELOCITY_LSB_MM_S, 3) + ",";
    payload += "\"magnitude\":" + sensorAxisJsonValue(eng.velocityValid, velocityOverall, SENSOR_VELOCITY_LSB_MM_S, 3) + "},";
    payload += "\"displacement\":{\"unit\":\"um\",";
    payload += "\"x\":" + sensorAxisJsonValue(eng.displacementValid, eng.displacementX, SENSOR_DISPLACEMENT_LSB_UM, 1) + ",";
    payload += "\"y\":" + sensorAxisJsonValue(eng.displacementValid, eng.displacementY, SENSOR_DISPLACEMENT_LSB_UM, 1) + ",";
    payload += "\"z\":" + sensorAxisJsonValue(eng.displacementValid, eng.displacementZ, SENSOR_DISPLACEMENT_LSB_UM, 1) + ",";
    payload += "\"magnitude\":" + sensorAxisJsonValue(eng.displacementValid, displacementOverall, SENSOR_DISPLACEMENT_LSB_UM, 1) + "},";
    payload += "\"sensorCalculatedFrequency\":{\"unit\":\"Hz\",";
    payload += "\"source\":\"sensor_reported\",";
    payload += "\"confidence\":\"low\",";
    payload += "\"suspicious\":" + String(eng.frequencySuspicious ? "true" : "false") + ",";
    payload += "\"x\":" + sensorAxisJsonValue(eng.frequencyValid && isValidSensorFrequency(eng.freqX), eng.freqX, SENSOR_FREQUENCY_SCALE, 1) + ",";
    payload += "\"y\":" + sensorAxisJsonValue(eng.frequencyValid && isValidSensorFrequency(eng.freqY), eng.freqY, SENSOR_FREQUENCY_SCALE, 1) + ",";
    payload += "\"z\":" + sensorAxisJsonValue(eng.frequencyValid && isValidSensorFrequency(eng.freqZ), eng.freqZ, SENSOR_FREQUENCY_SCALE, 1) + ",";
    payload += "\"magnitude\":" + sensorAxisJsonValue(eng.frequencyValid, freqOverall, 1.0f, 1) + "},";
    payload += "\"temperature\":{\"value\":" + (eng.temperatureValid ? String(eng.temperatureC, 2) : String("null")) + ",\"unit\":\"C\"}";
    payload += "},";
    payload += "\"alarm\":{";
    payload += "\"status\":\"" + healthStatus + "\",";
    payload += "\"severity\":" + String(alarmSeverity) + ",";
    payload += "\"reason\":\"" + alarmReason + "\",";
    payload += "\"triggeredMetric\":" + triggeredMetric + ",";
    payload += "\"threshold\":" + alarmThreshold + ",";
    payload += "\"currentValue\":" + alarmCurrentValue + ",";
    payload += "\"durationMs\":0,";
    payload += "\"confidence\":" + String(confidence);
    payload += "},";
    payload += "\"sensor\":{\"baud\":" + String(sensorBaud) +
               ",\"modbusId\":\"" + String(modbusIdBuf) +
               "\",\"txGpio\":" + String(UART2_TX_PIN) +
               ",\"rxGpio\":" + String(UART2_RX_PIN) +
               ",\"autoDirection\":" + String(RS485_DE_RE_PIN < 0 ? "true" : "false") +
               ",\"status\":\"" + String(sensorOk ? "ok" : "fault") + "\"},";
    payload += "\"chipTime\":";
    if (latestChipTime.valid) {
      unsigned fullYear = (latestChipTime.year < 100) ? 2000u + latestChipTime.year : (unsigned)latestChipTime.year;
      char chipTimeBuf[40];
      snprintf(chipTimeBuf, sizeof(chipTimeBuf), "%04u-%02u-%02uT%02u:%02u:%02u.%03u",
               fullYear, (unsigned)latestChipTime.month, (unsigned)latestChipTime.day,
               (unsigned)latestChipTime.hour, (unsigned)latestChipTime.minute,
               (unsigned)latestChipTime.sec, (unsigned)latestChipTime.ms);
      payload += "\"";
      payload += String(chipTimeBuf);
      payload += "\"";
    } else {
      payload += "null";
    }
    payload += "}";

    // [MQTT] [HIGH RISK] Enforce transport readiness and packet capacity before
    // publication. Bypassing this gate can produce truncated/failed messages.
    const size_t requiredPacketBytes = payload.length() + topic.length() + 16U;
    if (linkReady && mqttBufferReady && requiredPacketBytes < MQTT_PACKET_BUFFER_SIZE) {
      bool published = mqttClient.publish(topic.c_str(), payload.c_str());
      saturatingAdd(recentPublishAttempts);
      if (published) {
        saturatingAdd(publishSuccessCount);
        lastPublishOkMs = millis();
        mqttSessionProgressMs = lastPublishOkMs;
        // Broker acceptance alone is not end-to-end recovery: Mosquitto may be
        // alive while the Node dashboard is stopped. Only a valid sensor frame
        // plus a fresh application heartbeat may begin the AP close window.
        if (sensorOk && nodeServerHeartbeatFresh(lastPublishOkMs)) {
          armSetupApLifetimeAfterHealthyPublish();
        }
      } else {
        saturatingAdd(publishFailCount);
        saturatingAdd(recentPublishFails);
        Serial.printf("[MQTT] PUBLISH %s FAIL | %d bytes\n", topic.c_str(), payload.length());
        // A false return means the packet was not accepted by the transport.
        // Drop the suspect socket now so serviceMqtt() starts a clean bounded
        // reconnect instead of repeatedly writing into a dead session.
        mqttClient.disconnect();
        lastMqttConnected = false;
        mqttDisconnectedSinceMs = millis();
        lastMqttAttemptMs = mqttDisconnectedSinceMs;
        mqttRetryDelayMs = MQTT_RETRY_INITIAL_MS;
      }
    } else if (linkReady) {
      saturatingAdd(publishFailCount);
      saturatingAdd(recentPublishFails);
      Serial.printf("[MQTT] PUBLISH BLOCKED | payload=%u required=%u buffer=%u ready=%d\n",
                    (unsigned)payload.length(), (unsigned)requiredPacketBytes,
                    (unsigned)MQTT_PACKET_BUFFER_SIZE, mqttBufferReady ? 1 : 0);
    } else if (shouldLogSensor) {
      Serial.println("[MQTT] Skipping publish; STA/MQTT not ready.");
    }
    return;
  }
}
// =============================================================================
// LED
// =============================================================================
void updateLed() {
  unsigned long now = millis();
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  bool mqttConnected = mqttClient.connected();

  // AP active + STA disconnected: slow blink.
  if (!wifiConnected) {
    if (now - lastLedToggleMs >= LED_SLOW_BLINK_MS) {
      lastLedToggleMs = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
    }
    return;
  }

  // STA connected + MQTT disconnected: double blink.
  if (!mqttConnected) {
    unsigned long phase = now % LED_DOUBLE_BLINK_PERIOD_MS;
    bool on = (phase < LED_MQTT_BLINK_MS) ||
              (phase >= (LED_MQTT_BLINK_MS * 2) && phase < (LED_MQTT_BLINK_MS * 3));
    digitalWrite(LED_PIN, on ? LOW : HIGH);
    return;
  }

  // STA connected + MQTT connected: heartbeat pulse.
  if (ledPulseEndMs != 0 && (int32_t)(now - ledPulseEndMs) >= 0) {
    digitalWrite(LED_PIN, HIGH); // off
    ledPulseEndMs = 0;
  }
  if ((int32_t)(now - nextConnectedPulse) >= 0) {
    digitalWrite(LED_PIN, LOW);  // on
    ledPulseEndMs       = now + LED_CONNECTED_PULSE_MS;
    nextConnectedPulse  = now + LED_CONNECTED_INTERVAL_MS;
  }
}

uint64_t monotonicMillis() {
  static uint32_t previousLow = 0;
  static uint64_t high = 0;
  const uint32_t low = millis();
  if (low < previousLow) high += (1ULL << 32);
  previousLow = low;
  return high | low;
}

void feedWatchdog() {
  if (watchdogReady) {
    esp_task_wdt_reset();
  }
}

// =============================================================================
// Arduino lifecycle [HIGH RISK]
// =============================================================================
// setup() and loop() encode the working initialization and service order.
// Reordering AP start, Preferences/profile load, STA/MQTT setup, UART start,
// calibration, web service, sensor reads, or broker servicing can reproduce
// missing-AP, sensor_fault, watchdog, or responsiveness failures.
void setup() {
#if !defined(SENSOR_ON_PROGRAM_UART_PINS)
  Serial.begin(115200);
#endif
  const bool cpuFrequencySet = setCpuFrequencyMhz(RUNTIME_CPU_FREQUENCY_MHZ);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);  // BOOT button for on-demand WiFi setup
  digitalWrite(LED_PIN, HIGH); // off (active-low)

  delay(200);
  Serial.println("\n[Boot] IVSS Controller starting...");

  Serial.printf("[Boot] Reset reason code: %d  (1=POWERON 3=SW 4=PANIC 5=INT_WDT 6=TASK_WDT 7=RTC_WDT 12=BROWNOUT)\n", (int)esp_reset_reason());
  Serial.printf("[Power] CPU target=%lu MHz applied=%d actual=%lu MHz.\n",
                (unsigned long)RUNTIME_CPU_FREQUENCY_MHZ,
                cpuFrequencySet ? 1 : 0,
                (unsigned long)getCpuFrequencyMhz());

  // Hardware watchdog: if loop() ever stalls longer than WDT_TIMEOUT_S the chip
  // resets and recovers automatically instead of freezing silently.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtConfig = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  const esp_err_t wdtInitResult = esp_task_wdt_init(&wdtConfig);
#else
  const esp_err_t wdtInitResult = esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  const esp_err_t wdtAddResult = esp_task_wdt_add(NULL);
  watchdogReady = (wdtAddResult == ESP_OK) || (esp_task_wdt_status(NULL) == ESP_OK);
  Serial.printf("[WDT] init=%d add=%d active=%d\n",
                (int)wdtInitResult, (int)wdtAddResult, watchdogReady ? 1 : 0);
  feedWatchdog();

  Serial.println("[Setup] WiFi/MQTT page starts at http://192.168.4.1 (or http://sensor.local) and closes 10 minutes after healthy telemetry begins.");

  randomSeed(esp_random());

  // [WIFI AP] Load configuration first so the portal can use AP-only mode when
  // no STA network exists, avoiding an unnecessary second radio interface.
  loadPreferences();
  // IVSS owns credentials in Preferences. Keep the Arduino WiFi driver
  // from separately persisting stale PSK/802.1X state across retries.
  WiFi.persistent(false);
  // Register before any interface is started so the very first association
  // attempt at boot is already instrumented.
  WiFi.onEvent(onWifiEvent);
  startSetupPortal();

  mqttClient.setServer(mqttBroker.c_str(), mqttPort);
  mqttClient.setCallback(mqttMessageReceived);
  mqttClient.setKeepAlive(15);
  // [MQTT] Packet capacity and socket timeout are part of the working runtime
  // contract; changes affect blocking time and whether full telemetry fits.
  mqttBufferReady = mqttClient.setBufferSize(MQTT_PACKET_BUFFER_SIZE);
  mqttClient.setSocketTimeout(1);
  Serial.printf("[MQTT] packet buffer=%u ready=%d\n",
                (unsigned)MQTT_PACKET_BUFFER_SIZE, mqttBufferReady ? 1 : 0);

  if (wifiSsid.length() > 0) {
    beginStaConnection("boot");
  } else {
    Serial.println("[Boot] No WiFi credentials saved. Setup AP is active at http://192.168.4.1 (or http://sensor.local)");
  }

  if (mqttBroker.length() == 0) {
    Serial.println("[Boot] MQTT broker is empty. Configure it from the setup AP.");
  }

  if (SENSOR_POWER_PIN >= 0) {
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, HIGH);
  }
  // [SENSOR PROFILE] Start UART only after the active profile/config is loaded.
  Serial2.begin(sensorBaud, activeSensorSerialConfig(),
                UART2_RX_PIN, UART2_TX_PIN);
  if (RS485_DE_RE_PIN >= 0) {
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    rs485SetTransmit(false); // receive mode idle
  }
  Serial.printf("[Sensor] UART2 started TX=%d RX=%d baud=%lu\n",
                UART2_TX_PIN, UART2_RX_PIN, (unsigned long)sensorBaud);

  delay(SENSOR_BOOT_SETTLE_MS);
  flushModbusRx(20);

  // Do not enter a long calibration loop when an uploaded profile points to a
  // disconnected or not-yet-installed sensor. Calibration is retried safely
  // after communication returns.
  bool sensorReadyForCalibration = false;
  {
    float x = 0, y = 0, z = 0;
    sensorReadyForCalibration = readSensorAccelerationRaw(x, y, z);
    // Schema-3 profiles are authoritative. Do not sweep baud rates, IDs, and
    // registers automatically after one failed read: that produces a burst of
    // bus activity at boot and can worsen current draw or pin contention. A
    // mismatched/disconnected sensor remains on the normal bounded backoff path.
  }
  baselineCalibrationPending =
    !sensorReadyForCalibration || !calibrateSensorBaseline();
  if (baselineCalibrationPending) {
    Serial.println("[Sensor] Baseline calibration deferred until communication is available.");
  }
  mqttDisconnectedSinceMs = millis();
  Serial.println("[Boot] Runtime started. Setup AP remains open until 10 minutes after the first valid MQTT publish.");
}

// Reopen the commissioning AP after its automatic shutdown.
//   1) hold the BOOT button (GPIO0) for SETUP_BUTTON_HOLD_MS
//   2) type "setup" (or "ap") + Enter in the serial monitor
void checkSetupTriggers() {
  // BOOT button held low long enough
  if (digitalRead(SETUP_BUTTON_PIN) == LOW) {
    if (setupBtnDownMs == 0) setupBtnDownMs = millis();
    else if (millis() - setupBtnDownMs >= SETUP_BUTTON_HOLD_MS) {
      Serial.println("[Setup] BOOT button held.");
      setupBtnDownMs = 0;
      startSetupPortal();
      announceSetupPortal("BOOT button held");
      return;
    }
  } else {
    setupBtnDownMs = 0;
  }

  // Serial commands are unavailable when GPIO1/GPIO3 are assigned to the
  // sensor: reading UART0 there would mix sensor traffic with console input.
#if !defined(SENSOR_ON_PROGRAM_UART_PINS)
  // Serial command
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialCmd.trim();
      if (serialCmd.equalsIgnoreCase("setup") || serialCmd.equalsIgnoreCase("ap")) {
        Serial.println("[Setup] Serial command received.");
        serialCmd = "";
        startSetupPortal();
        announceSetupPortal("serial command");
        return;
      }
      if (serialCmd.startsWith("poll ")) {
        unsigned long requested = (unsigned long)serialCmd.substring(5).toInt();
        setSensorPollInterval(requested, true);
        serialCmd = "";
        return;
      }
      serialCmd = "";
    } else if (serialCmd.length() < 32) {
      serialCmd += c;
    }
  }
#endif
}

// Adapt publish rate to link quality: strong/normal WiFi -> 10 Hz;
// weak signal or publish losses -> back off to 5 Hz so a bad link is not
// flooded (flooding a bad link is what causes drops and stutter).
void updatePublishCadence() {
  // [MQTT] [CALCULATIONS] Adapts acquisition/publication cadence using link
  // quality. Threshold changes affect both network load and sensor sample rate.
  if (millis() - lastCadenceEvalMs < CADENCE_EVAL_INTERVAL_MS) return;
  lastCadenceEvalMs = millis();
  if (WiFi.status() != WL_CONNECTED || !mqttClient.connected()) {
    currentPublishIntervalMs = PUBLISH_SLOW_MS;   // link down: do not flood
    recentPublishAttempts = 0; recentPublishFails = 0;
    return;
  }
  int rssi = WiFi.RSSI();  // dBm (negative; closer to 0 = stronger)
  float failRatio = recentPublishAttempts > 0
    ? (float)recentPublishFails / (float)recentPublishAttempts : 0.0f;
  recentPublishAttempts = 0; recentPublishFails = 0;
  if (failRatio > 0.20f || rssi <= -80) {
    currentPublishIntervalMs = PUBLISH_SLOW_MS;   // weak / lossy -> 2 Hz
  } else if (failRatio > 0.05f || rssi <= -70) {
    currentPublishIntervalMs = PUBLISH_MED_MS;    // marginal -> 5 Hz
  } else {
    currentPublishIntervalMs = PUBLISH_FAST_MS;   // strong -> 10 Hz
  }
}

void loop() {
  // [HIGH RISK] Preserve this cooperative service order. There is no RTOS task
  // isolation here; blocking or retry loops in one subsystem delay the others.
  feedWatchdog();  // feed the hardware watchdog every iteration
  if (webServerStarted) {
    server.handleClient();
  }
  if (setupApDnsStarted) {
    dnsServer.processNextRequest();
  }
  if (setupApRestartPending &&
      (int32_t)(millis() - setupApRestartAfterMs) >= 0) {
    setupApRestartPending = false;
    if (webServerStarted) {
      server.stop();
      webServerStarted = false;
    }
    if (setupApDnsStarted) {
      dnsServer.stop();
      setupApDnsStarted = false;
    }
    if (setupApMdnsStarted) {
      MDNS.end();
      setupApMdnsStarted = false;
    }
    WiFi.softAPdisconnect(true);
    setupApStarted = false;
    startSetupPortal();
  }
  serviceSetupApLifetime();
  if (sensorProfileReconfigurePending &&
      (int32_t)(millis() - sensorProfileReconfigureAfterMs) >= 0) {
    // [SENSOR PROFILE] [HIGH RISK] Applies a newly validated profile to the live
    // UART and resets all profile-dependent acquisition/calibration state.
    sensorProfileReconfigurePending = false;
    hasLastRaw = false;
    baselineCalibrated = false;
    baselineCalibrationPending = true;
    baselineX = BASELINE_FALLBACK_X;
    baselineY = BASELINE_FALLBACK_Y;
    baselineZ = BASELINE_FALLBACK_Z;
    noiseFloorX = 0.0f;
    noiseFloorY = 0.0f;
    noiseFloorZ = 0.0f;
    latestChipTime = {};
    lastChipTimeReadMs = millis() - CHIP_TIME_READ_INTERVAL_MS;
    waveWriteIndex = 0;
    waveSamplesStored = 0;
    consecutiveModbusFailures = 0;
    lastModbusFailureMs = 0;
    modbusRecoveryPending = false;
    sensorPowerCycleAttempted = false;
    lastModbusSuccessMs = millis();
    lastSensorReadOkMs = 0;
    snprintf(lastModbusErrorReason, sizeof(lastModbusErrorReason), "none");
    restartModbusUart("sensor profile changed");
    delay(SENSOR_BOOT_SETTLE_MS);
    float testX = 0.0f, testY = 0.0f, testZ = 0.0f;
    if (readSensorAccelerationRaw(testX, testY, testZ)) {
      baselineCalibrationPending = setupApBlockingStaAssociation() ||
                                   !calibrateSensorBaseline();
    } else {
      Serial.println("[Profile] Sensor did not answer the new profile; calibration will retry after communication returns.");
    }
    lastBaselineCalibrationAttemptMs = millis();
  }
  if (baselineCalibrationPending && !sensorProfileReconfigurePending &&
      millis() - lastBaselineCalibrationAttemptMs >= BASELINE_RETRY_INTERVAL_MS &&
      !modbusRecoveryActive() && !setupApBlockingStaAssociation()) {
    lastBaselineCalibrationAttemptMs = millis();
    float testX = 0.0f, testY = 0.0f, testZ = 0.0f;
    if (readSensorAccelerationRaw(testX, testY, testZ)) {
      baselineCalibrationPending = !calibrateSensorBaseline();
    }
    // An attempt can itself take many seconds. Measure the cooldown from its
    // completion so an intermittent profile/sensor cannot immediately start
    // another blocking calibration pass.
    lastBaselineCalibrationAttemptMs = millis();
  }
  checkSetupTriggers();  // BOOT button / serial "setup" -> WiFi setup AP
  updatePublishCadence(); // adapt publish rate to WiFi + publish-loss

  // --- Heartbeat: periodic visibility into uptime, heap, links, counters ---
  {
    unsigned long hbNow = millis();
    if (hbNow - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
      lastHeartbeatMs = hbNow;
      // Heap traversal is diagnostic work, not a loop service. Running these
      // calls on every iteration kept one CPU core unnecessarily active.
      uint32_t freeHeap = ESP.getFreeHeap();
      uint32_t largestFreeBlock =
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
      if (freeHeap < minFreeHeap) minFreeHeap = freeHeap;
      if (largestFreeBlock < minLargestFreeBlock) {
        minLargestFreeBlock = largestFreeBlock;
      }
      if (!heap_caps_check_integrity_all(false)) {
        Serial.println("[HEAP] Integrity failure detected; restarting before corrupted state is used.");
        delay(20);
        esp_restart();
      }
      if (freeHeap < MIN_SAFE_FREE_HEAP_BYTES ||
          largestFreeBlock < MIN_SAFE_LARGEST_BLOCK_BYTES) {
        if (lowHeapHeartbeatCount < 255) lowHeapHeartbeatCount++;
      } else {
        lowHeapHeartbeatCount = 0;
      }
      if (lowHeapHeartbeatCount >= 3) {
        Serial.printf("[HEAP] Sustained low memory heap=%u largest=%u; controlled restart.\n",
                      freeHeap, largestFreeBlock);
        delay(20);
        esp_restart();
      }
      Serial.printf("[HB] up=%lus heap=%u min=%u wifi=%s mqtt=%s modbus(fail=%lu to=%lu crc=%lu) pub(ok=%lu fail=%lu) lastReadOk=%ldms lastPubOk=%ldms\n",
        hbNow / 1000, freeHeap, minFreeHeap,
        (WiFi.status() == WL_CONNECTED ? "up" : "down"),
        (mqttClient.connected() ? "up" : "down"),
        (unsigned long)modbusFailCount, (unsigned long)modbusTimeoutCount, (unsigned long)modbusCrcFailCount,
        (unsigned long)publishSuccessCount, (unsigned long)publishFailCount,
        lastSensorReadOkMs ? (long)(hbNow - lastSensorReadOkMs) : -1L,
        lastPublishOkMs ? (long)(hbNow - lastPublishOkMs) : -1L);
      Serial.printf("[HB] acquisition interval=%lu ms (%.1f Hz) mqttMax=%.1f Hz rssi=%d dBm\n",
        currentPublishIntervalMs, 1000.0f / (float)currentPublishIntervalMs,
        MQTT_PUBLISH_RATE_HZ,
        (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0);
      long recoveryLeftMs = (long)modbusRecoveryRemainingMs();
      Serial.printf("[HB] modbus ok=%lu fail=%lu consecutive=%u last='%s' configuredPoll=%lu ms effectivePoll=%lu ms actual=%lu ms splitTx=%u recoveryLeft=%ld ms heap=%u\n",
        (unsigned long)modbusSuccessCount,
        (unsigned long)modbusFailCount,
        consecutiveModbusFailures,
        lastModbusErrorReason,
        sensorPollIntervalMs,
        effectiveSensorReadIntervalMs(),
        lastActualSensorPollIntervalMs,
        useSplitModbusReads ? splitProfileTransactionsPerSample() : 1U,
        recoveryLeftMs,
        freeHeap);
      Serial.printf("[HB] heap largest=%u minLargest=%u config=%d profile=%d networkIdentity=%d wdt=%d mqttBuffer=%d telemetryMemory=%d\n",
                    largestFreeBlock, minLargestFreeBlock,
                    configStorageHealthy ? 1 : 0, profileStorageHealthy ? 1 : 0,
                    networkIdentityHealthy ? 1 : 0,
                     watchdogReady ? 1 : 0,
                     mqttBufferReady ? 1 : 0, telemetryMemoryHealthy ? 1 : 0);
      Serial.printf("[HB] power cpu=%luMHz loopIdle=%lums wifiRetry=%lums mqttRetry=%lums\n",
                    (unsigned long)getCpuFrequencyMhz(), LOOP_IDLE_DELAY_MS,
                    wifiRetryDelayMs, mqttRetryDelayMs);
      Serial.printf("[HB] sta mode=%s inProgress=%d gotIpEver=%d disconnects=%lu "
                    "txPower=%d(0.25dBm) lastReason=%u: %s\n",
                    wifiEnterpriseActive ? "enterprise" : "psk",
                    staConnectInProgress ? 1 : 0, staEverGotIp ? 1 : 0,
                    (unsigned long)staDisconnectCount, (int)WiFi.getTxPower(),
                    (unsigned)lastStaDisconnectReason,
                    staDisconnectReasonText(lastStaDisconnectReason));
    }
  }

  serviceWiFi();

  unsigned long now = millis();
  unsigned long profileSafeIntervalMs = effectiveSensorReadIntervalMs();
  if (setupApBlockingStaAssociation()) {
    profileSafeIntervalMs =
      max(profileSafeIntervalMs, AP_CLIENT_SENSOR_INTERVAL_MS);
  }
  const bool telemetryPathReady =
    WiFi.status() == WL_CONNECTED && mqttClient.connected() &&
    nodeServerHeartbeatFresh(now) && !setupApBlockingStaAssociation();
  if (telemetryPathReady && !telemetryPathWasReady) {
    // [CALCULATIONS] [HIGH RISK] A long outage is sampled at 2 s to save power.
    // Never mix those aliased/stale points into the restored normal-rate RMS,
    // including when the heartbeat returned during the 20 s AP-client grace.
    if (ACTIVE_SENSOR_PROFILE.accelerationIsInstantaneousWaveform) {
      waveWriteIndex = 0;
      waveSamplesStored = 0;
    }
    lastAcquisitionStartMs = 0;
    // Make the first post-recovery sensor read eligible immediately instead of
    // waiting for the previous 2 s outage interval to finish.
    lastPublishMs = now - max(1UL, effectiveSensorReadIntervalMs());
  }
  telemetryPathWasReady = telemetryPathReady;
  if (!telemetryPathReady) {
    // There is no consumer for a 10 Hz stream. Keep a slow health sample so
    // communication state is known, then restore the profile/configured cadence
    // immediately after the end-to-end server path becomes usable.
    profileSafeIntervalMs =
      max(profileSafeIntervalMs, OFFLINE_SENSOR_INTERVAL_MS);
  }
  if (consecutiveModbusFailures > 0 || modbusRecoveryActive()) {
    // A healthy network must not cause a disconnected/faulty sensor to produce
    // 10 Hz UART attempts and multi-kilobyte invalid MQTT payloads. One valid
    // Modbus response clears the failure counter and restores fast cadence.
    profileSafeIntervalMs =
      max(profileSafeIntervalMs, SENSOR_FAULT_INTERVAL_MS);
  }
  const unsigned long sensorReadIntervalMs =
    currentPublishIntervalMs < profileSafeIntervalMs
      ? profileSafeIntervalMs
      : currentPublishIntervalMs;
  if (now - lastPublishMs >= sensorReadIntervalMs) {
    lastPublishMs = now;
    publishData();
  }

  serviceMqtt();
  updateLed();
  delay(LOOP_IDLE_DELAY_MS);
}
