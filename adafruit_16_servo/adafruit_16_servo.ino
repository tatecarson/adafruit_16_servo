/***************************************************
  Adafruit 16-channel PWM & Servo driver - Calibration & Control

  Based on Adafruit example code.
  ------> http://www.adafruit.com/products/815

  Serial Commands:
    Testing / Calibration:
      S<n> <pos>     - Move servo n to position (degrees, range per servo)
      P<n> <pulse>   - Move servo n to raw pulse (150-600)
      CAL <n> <min> <max> - Set servo n calibration (pulse values)
      SWEEP <n>      - Test sweep servo n
      OFF <n>        - Turn off servo n
      RELEASE <n>    - Force-release servo n
      STATUS         - Show all servo calibrations
    Performance / Installation:
      MOVE <n> <deg> <ms> - Animated move with easing
      UP <n> <pct>   - Move servo n to absolute percent up
      DOWN <n> <pct> - Move servo n to absolute percent down
      UMOVE <n> <pct> <ms> - Animated move to absolute percent up
      DMOVE <n> <pct> <ms> - Animated move to absolute percent down
      ALLUP <pct> [ms]   - Move all protected winch servos up together
      ALLDOWN <pct> [ms] - Move all protected winch servos down together
      RIG <UP|DOWN> <pct> <spd> [ms] - Manual winch + DC motor test
      MOTION <id>       - Play baked browser Motion by id
      PLAY <n> [LOOP]    - Play keyframe sequence
      SPLAY <n> [LOOP]   - Play speed sequence (DC motor)
      RUN <n> [LOOP]     - Run a chained program of sequences
      STOP [n]           - Stop all motion or hold one servo
      ROTATE <spd>       - Set DC motor rotation speed
      TIMESCALE <n>      - Scale sequence timing n times slower
      HELP               - Show commands
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFiS3.h>
#include <ArduinoOTA.h>

#include "Secrets.h"
#include "Sync.h"
#include "Web.h"

#include "servo_runtime.h"
#include "dc_motor.h"
#include "storage.h"
#include "servo_calibration.h"
#include "servo_setup.h"
#include "sequence_setup.h"

#include "servo_control.h"
#include "motion_engine.h"
#include "sequence_engine.h"
#include "animation_engine.h"
#include "servo_maintenance.h"
#include "command_interface.h"

// Wi-Fi / OTA state. otaReady gates syncPoll() so we don't UDP before the
// stack is up. otaInProgress pauses the animation loop during an upload so
// the servo loop doesn't fight the OTA flash.
static char wifiSsid[] = WIFI_SSID;
static char wifiPass[] = WIFI_PASS;
static_assert(sizeof(OTA_PASSWORD) > 1, "OTA_PASSWORD in Secrets.h must not be empty");
bool otaReady = false;
bool otaInProgress = false;
bool otaError = false;

static bool ipIsUnset(IPAddress ip) {
  return ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0;
}

static void onOtaStart() {
  otaInProgress = true;
  Serial.println(F("OTA upload started."));
}

static void onOtaApply() {
  Serial.println(F("OTA upload complete. Applying update..."));
}

static void onOtaError(int code, const char* message) {
  otaError = true;
  otaInProgress = false;
  Serial.print(F("OTA error ")); Serial.print(code);
  Serial.print(F(": ")); Serial.println(message);
}

static bool waitForWifiAndIp(unsigned long timeoutMs) {
  unsigned long startedAt = millis();
  while (millis() - startedAt < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED && !ipIsUnset(WiFi.localIP())) return true;
    delay(500);
    Serial.print(".");
  }
  return false;
}

static void printWifiStatus() {
  Serial.print(F("WiFi status: ")); Serial.println(WiFi.status());
  Serial.print(F("SSID: ")); Serial.println(WiFi.SSID());
  Serial.print(F("RSSI: ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));
  Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
}

static bool wifiHasUsableIp() {
  return WiFi.status() == WL_CONNECTED && !ipIsUnset(WiFi.localIP());
}

static void networkServicesBegin() {
  ArduinoOTA.onStart(onOtaStart);
  ArduinoOTA.beforeApply(onOtaApply);
  ArduinoOTA.onError(onOtaError);
  ArduinoOTA.begin(WiFi.localIP(), OTA_HOSTNAME, OTA_PASSWORD, InternalStorage);
  webBegin();
  syncBegin();
  otaReady = true;
}

static void wifiAndOtaBegin() {
  Serial.print(F("WiFi firmware: "));
  Serial.println(WiFi.firmwareVersion());
  WiFi.begin(wifiSsid, wifiPass);
  if (!waitForWifiAndIp(30000)) {
    Serial.println();
    Serial.println(F("WiFi did not get a usable IP. Retrying once..."));
    printWifiStatus();
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(wifiSsid, wifiPass);
  }
  Serial.println();
  if (!waitForWifiAndIp(30000)) {
    Serial.println(F("WiFi unavailable; continuing USB-only."));
    printWifiStatus();
    return;
  }
  networkServicesBegin();
  Serial.println(F("OTA + sync ready."));
  Serial.print(F("Node id: ")); Serial.println(syncNodeId());
  Serial.print(F("Sync UDP port: ")); Serial.println(SYNC_PORT);
  Serial.print(F("Control: http://")); Serial.print(WiFi.localIP()); Serial.println(F("/"));
  printWifiStatus();
}

// Streamed JSON snapshot for /status.json. Keep keys short to keep the
// payload under the TCP MTU on a single send.
void writeStatusJson(WiFiClient& client) {
  // Build the whole payload in RAM, then push it to the WiFi coprocessor in
  // one shot. Each WiFiClient::print is an SPI round-trip; the previous
  // ~100-call version took ~1s per status poll and starved /cmd handling.
  String s;
  s.reserve(1200);

  IPAddress ip = WiFi.localIP();
  char ipBuf[16];
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

  s += F("{\"node\":");          s += syncNodeId();
  s += F(",\"ip\":\"");          s += ipBuf; s += '"';
  s += F(",\"uptimeMs\":");      s += millis();
  s += F(",\"otaInProgress\":"); s += (otaInProgress ? F("true") : F("false"));
  s += F(",\"sequence\":{\"active\":"); s += (sequenceActive ? F("true") : F("false"));
  s += F(",\"loop\":");          s += (sequenceLoop ? F("true") : F("false"));
  s += F(",\"len\":");           s += currentSequenceLength;
  s += F(",\"startedMs\":");     s += (sequenceActive ? (millis() - sequenceStartTime) : 0UL);
  s += '}';
  s += F(",\"speedSeq\":{\"active\":"); s += (speedSeqActive ? F("true") : F("false"));
  s += F(",\"loop\":");          s += (speedSeqLoop ? F("true") : F("false"));
  s += F(",\"len\":");           s += currentSpeedSeqLength;
  s += F(",\"startedMs\":");     s += (speedSeqActive ? (millis() - speedSeqStartTime) : 0UL);
  s += '}';
  s += F(",\"motion\":{\"active\":"); s += (motionRuntime.active ? F("true") : F("false"));
  // motionRuntime.id is emitted unescaped: motionCopyString() (motion_engine.h)
  // rejects '"' and '\\' at parse time, and the schema id regex
  // ^[a-z][a-z0-9-]{0,31}$ permits no other JSON-significant characters.
  s += F(",\"id\":\"");          s += motionRuntime.id;
  s += F("\",\"tracks\":");      s += motionRuntime.trackCount;
  s += F(",\"durationMs\":");    s += motionRuntime.durationMs;
  s += F(",\"startedMs\":");     s += (motionRuntime.active ? (millis() - motionRuntime.startMs) : 0UL);
  s += '}';
  // Sequence runner (servo-3a9) — schema v1 RUN <id> [LOOP]. Same id-emit
  // safety argument as motion: parser-rejected charset + schema regex.
  s += F(",\"runseq\":{\"active\":"); s += (sequenceRunner.active ? F("true") : F("false"));
  s += F(",\"id\":\"");          s += sequenceRunner.id;
  s += F("\",\"step\":");        s += sequenceRunner.currentStep;
  s += F(",\"steps\":");         s += sequenceRunner.stepCount;
  s += F(",\"loop\":");          s += (sequenceRunner.loop ? F("true") : F("false"));
  s += F(",\"stepMs\":");        s += (sequenceRunner.active ? (millis() - sequenceRunner.stepStartMs) : 0UL);
  s += '}';
  // WAVE removed (servo-dz7); browser clients should treat absence of the
  // "wave" key as wave-not-supported on this firmware build.
  s += F(",\"timescale\":");     s += timeMultiplier;
  s += F(",\"servos\":[");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (i) s += ',';
    s += F("{\"i\":");           s += i;
    s += F(",\"pulse\":");       s += servoState[i].posPulse;
    s += F(",\"target\":");      s += servoState[i].targetPulse;
    s += F(",\"moving\":");      s += (servoState[i].moving ? F("true") : F("false"));
    s += '}';
  }
  s += F("],\"motor\":{\"speed\":"); s += motorState.currentSpeed;
  s += F(",\"ramping\":");       s += (motorState.ramping ? F("true") : F("false"));
  s += F("}}");

  client.write((const uint8_t*)s.c_str(), s.length());
}

// Returns the largest sketch (in bytes) that InternalStorage can stage for
// OTA on this board. The library's ceiling is (FLASH_LENGTH - SKETCH_START)/2
// (page-aligned) — on UNO R4 WiFi that's 122880 bytes (120 KB), well under
// the 262144-byte total flash that arduino-cli reports. Web.cpp uses this to
// reject oversized uploads before any flash is touched.
int otaMaxSize() {
  return (int)InternalStorage.maxSize();
}

bool otaReceive(WiFiClient& client, int contentLength) {
  Serial.print(F("Browser OTA: receiving "));
  Serial.print(contentLength);
  Serial.println(F(" bytes (limit "));
  Serial.print(otaMaxSize());
  Serial.println(F(")"));

  // CRITICAL: InternalStorage.open() returns 0 if contentLength exceeds the
  // OTA partition (or if the flash driver fails to init). If we ignore that
  // failure and let apply() run anyway, the library erases the first page of
  // the running sketch and reboots into garbage — bricking the board and
  // requiring USB recovery. Refuse the upload here instead.
  otaInProgress = true;
  if (InternalStorage.open(contentLength) == 0) {
    Serial.println(F("Browser OTA: InternalStorage.open() refused (too large or flash busy)"));
    otaInProgress = false;
    return false;
  }

  int received = 0;
  unsigned long deadline = millis() + 60000;
  uint8_t buf[256];
  while (received < contentLength && millis() < deadline) {
    int avail = client.available();
    if (avail <= 0) { delay(1); continue; }
    int toRead = min(avail, min((int)sizeof(buf), contentLength - received));
    int n = client.read(buf, toRead);
    for (int i = 0; i < n; i++) InternalStorage.write(buf[i]);
    received += n;
  }
  InternalStorage.close();

  if (received == contentLength) {
    Serial.println(F("Browser OTA: upload complete."));
    return true;
  }
  Serial.print(F("Browser OTA: incomplete ("));
  Serial.print(received);
  Serial.println(F(" bytes)"));
  otaInProgress = false;
  return false;
}

void otaApply() {
  Serial.println(F("Browser OTA: applying update..."));
  InternalStorage.apply();
}

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ServoConfig servoConfig[NUM_SERVOS];
ServoState servoState[NUM_SERVOS];

MotorState motorState;

// WAVE pattern state removed (servo-dz7). See animation_engine.h.

// Time multiplier for scaling sequence durations (1 = no scaling, 60 = 60x slower)
// Set via serial command: TIMESCALE <n>
uint16_t timeMultiplier = 1;

// Sequence playback state (pointers to PROGMEM arrays)
bool sequenceActive = false;
bool sequenceLoop = false;
const Keyframe* currentSequence = nullptr;
uint8_t currentSequenceLength = 0;
unsigned long sequenceStartTime = 0;
uint8_t lastTriggeredKeyframe = 0;

// Speed sequence playback state (pointers to PROGMEM arrays)
bool speedSeqActive = false;
bool speedSeqLoop = false;
const SpeedFrame* currentSpeedSeq = nullptr;
uint8_t currentSpeedSeqLength = 0;
unsigned long speedSeqStartTime = 0;
uint8_t lastTriggeredSpeedFrame = 0;

// Chained program playback state
bool programActive = false;
bool programLoop = false;
const SequenceProgramDefinition* currentProgram = nullptr;
bool programPositionDone = true;
bool programSpeedDone = true;
uint8_t currentProgramPositionStepIndex = 0;
uint16_t currentProgramPositionIteration = 0;
uint8_t currentProgramSpeedStepIndex = 0;
uint16_t currentProgramSpeedIteration = 0;

// Browser-baked Motion playback state
MotionRuntime motionRuntime;

// Browser-baked Sequence runner state (servo-3a9)
SequenceRuntime sequenceRunner;

// Default calibration values
#define DEFAULT_MIN 150
#define DEFAULT_MAX 600

/**
 * @brief Initialize per-servo calibration and runtime state to sane defaults.
 */
void initServoDefaults() {
  uint16_t defaultCenter = (DEFAULT_MIN + DEFAULT_MAX) / 2;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servoConfig[i].minPulse = DEFAULT_MIN;
    servoConfig[i].maxPulse = DEFAULT_MAX;
    servoConfig[i].stopPulse = defaultCenter;
    servoConfig[i].totalDegrees = 180;
    servoConfig[i].allowRelease = true;
    servoConfig[i].upDegrees = 0;
    servoConfig[i].downDegrees = 0;
    servoConfig[i].reverseDir = false;
    servoConfig[i].offsetDeg = 0;

    servoState[i].posPulse = defaultCenter;
    servoState[i].targetPulse = defaultCenter;
    servoState[i].startPulse = defaultCenter;
    servoState[i].moveStartMs = 0;
    servoState[i].moveDurationMs = 0;
    servoState[i].moving = false;
    servoState[i].linearMove = false;
  }
}

// Serial input buffer (fixed-size to avoid String heap fragmentation)
#define INPUT_BUFFER_SIZE 50
char inputBuffer[INPUT_BUFFER_SIZE];
uint8_t inputIndex = 0;
bool inputComplete = false;

/**
 * @brief Initialize hardware, driver, and servo state before entering the main loop.
 *
 * Initializes the Serial console, the PWM/servo driver, applies default and custom servo
 * configurations, and reserves the serial input buffer.
 */
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("Servo Calibration & Control"));
  Serial.println(F("Type HELP for commands"));
  Serial.println();

  storageInit();
  Serial.print(F("storage: boardId=")); Serial.print(storageBoardId());
  Serial.print(F(" hasActive=")); Serial.print(storageHasActive() ? F("yes") : F("no"));
  Serial.print(F(" hasPrevious=")); Serial.println(storageHasPrevious() ? F("yes") : F("no"));

  initServoDefaults();
  applyCustomServoSetup(servoConfig, servoState);
  // Servo calibration overrides for any channel the operator has saved
  // via POST /calibration. Channels still on defaults keep the values
  // from applyCustomServoSetup (which are hand-tuned per sculpture).
  calibrationInit();
  calibrationApplyToServoConfig(servoConfig);
  calibrationPrintBootStatus();
  motorInit();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  // Wi-Fi is best-effort. If it fails the servo loop still works over USB.
  wifiAndOtaBegin();
}

static void maintainWifi() {
  static unsigned long lastCheckMs = 0;
  static unsigned long reconnectStartedMs = 0;
  static bool reconnecting = false;
  const unsigned long CHECK_INTERVAL_MS = 5000;
  const unsigned long RECONNECT_TIMEOUT_MS = 15000;

  unsigned long now = millis();
  if (now - lastCheckMs < CHECK_INTERVAL_MS) return;
  lastCheckMs = now;

  if (wifiHasUsableIp()) {
    if (!otaReady || reconnecting) {
      reconnecting = false;
      networkServicesBegin();
      Serial.println(F("WiFi ready; HTTP/OTA/sync restarted."));
      Serial.print(F("Control: http://")); Serial.print(WiFi.localIP()); Serial.println(F("/"));
      printWifiStatus();
    }
    return;
  }

  if (!reconnecting) {
    Serial.println(F("WiFi link lost; attempting reconnect..."));
    otaReady = false;
    otaInProgress = false;
    WiFi.disconnect();
    delay(100);
    WiFi.begin(wifiSsid, wifiPass);
    reconnectStartedMs = now;
    reconnecting = true;
    return;
  }

  if (now - reconnectStartedMs > RECONNECT_TIMEOUT_MS) {
    Serial.println(F("WiFi reconnect timed out; will retry."));
    reconnecting = false;
  }
}

void loop() {
  // Per-section timing. Anything over LOOP_WARN_MS prints a one-line
  // breakdown so we can see which call stalled the loop.
  const unsigned long LOOP_WARN_MS = 250;
  unsigned long t0 = millis();
  unsigned long tMaint = 0, tOta = 0, tWeb = 0, tSync = 0, tAnim = 0;

  unsigned long s = millis();
  maintainWifi();             tMaint = millis() - s;

  if (otaReady) {
    s = millis();
    ArduinoOTA.poll();          tOta   = millis() - s;
    s = millis();
    webPoll();                  tWeb   = millis() - s;
    s = millis();
    syncPoll();                 tSync  = millis() - s;
  }

  // Pause animation work during an active OTA upload so the flash isn't
  // contended. Servo positions hold; sequences resume after reboot.
  if (!otaInProgress) {
    unsigned long s = millis();
    updateAnimations();
    updateSpeedRamps();
    updateMotion();
    updateSequenceRunner();
    updateSequence();
    updateSpeedSequence();
    updateSequenceProgram();
    tAnim = millis() - s;
  }

  unsigned long total = millis() - t0;
  if (total >= LOOP_WARN_MS) {
    Serial.print(F("SLOW loop "));
    Serial.print(total); Serial.print(F("ms"));
    Serial.print(F(" maint=")); Serial.print(tMaint);
    Serial.print(F(" ota="));   Serial.print(tOta);
    Serial.print(F(" web="));   Serial.print(tWeb);
    Serial.print(F(" sync="));  Serial.print(tSync);
    Serial.print(F(" anim="));  Serial.println(tAnim);
  }

  // Read serial input into fixed buffer
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer[inputIndex] = '\0';
      inputComplete = true;
    } else if (inputIndex < INPUT_BUFFER_SIZE - 1) {
      inputBuffer[inputIndex++] = c;
    }
    // Silently ignore characters beyond buffer size
  }

  // Process complete command
  if (inputComplete) {
    dispatchCommand(inputBuffer, false);
    inputIndex = 0;
    inputComplete = false;
  }
}
