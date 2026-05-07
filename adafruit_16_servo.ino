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
      CENTER <n>     - Move servo n to center (90 degrees)
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
      RIG <UP|DOWN> <pct> <spd> [ms] - Manual winch + rotation test
      WAVE <start> <end> [speed] [offset] [amp] - Wave pattern
      PLAY <n> [LOOP]    - Play keyframe sequence
      SPLAY <n> [LOOP]   - Play speed sequence (continuous servos)
      STOP               - Stop wave/sequence
      MODE <n> STD|CONT  - Set servo mode (standard/continuous)
      ROTATE <spd>       - Set installation rotation speed
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
#include "servo_setup.h"
#include "sequence_setup.h"

#include "servo_control.h"
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
  ArduinoOTA.onStart(onOtaStart);
  ArduinoOTA.beforeApply(onOtaApply);
  ArduinoOTA.onError(onOtaError);
  ArduinoOTA.begin(WiFi.localIP(), OTA_HOSTNAME, OTA_PASSWORD, InternalStorage);
  webBegin();
  syncBegin();
  otaReady = true;
  Serial.println(F("OTA + sync ready."));
  Serial.print(F("Node id: ")); Serial.println(syncNodeId());
  Serial.print(F("Sync UDP port: ")); Serial.println(SYNC_PORT);
  Serial.print(F("Control: http://")); Serial.print(WiFi.localIP()); Serial.println(F("/"));
  printWifiStatus();
}

// Streamed JSON snapshot for /status.json. Keep keys short to keep the
// payload under the TCP MTU on a single send.
void writeStatusJson(WiFiClient& client) {
  client.print(F("{\"node\":")); client.print(syncNodeId());
  client.print(F(",\"ip\":\"")); client.print(WiFi.localIP()); client.print('"');
  client.print(F(",\"uptimeMs\":")); client.print(millis());
  client.print(F(",\"otaInProgress\":")); client.print(otaInProgress ? F("true") : F("false"));
  client.print(F(",\"sequence\":{\"active\":")); client.print(sequenceActive ? F("true") : F("false"));
  client.print(F(",\"loop\":")); client.print(sequenceLoop ? F("true") : F("false"));
  client.print(F(",\"len\":")); client.print(currentSequenceLength);
  client.print(F(",\"startedMs\":")); client.print(sequenceActive ? (millis() - sequenceStartTime) : 0UL);
  client.print('}');
  client.print(F(",\"speedSeq\":{\"active\":")); client.print(speedSeqActive ? F("true") : F("false"));
  client.print(F(",\"loop\":")); client.print(speedSeqLoop ? F("true") : F("false"));
  client.print(F(",\"len\":")); client.print(currentSpeedSeqLength);
  client.print(F(",\"startedMs\":")); client.print(speedSeqActive ? (millis() - speedSeqStartTime) : 0UL);
  client.print('}');
  client.print(F(",\"wave\":{\"active\":")); client.print(waveActive ? F("true") : F("false"));
  client.print(F(",\"start\":")); client.print(waveStartServo);
  client.print(F(",\"end\":")); client.print(waveEndServo);
  client.print('}');
  client.print(F(",\"timescale\":")); client.print(timeMultiplier);
  client.print(F(",\"servos\":["));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (i) client.print(',');
    client.print(F("{\"i\":")); client.print(i);
    client.print(F(",\"pulse\":")); client.print(servoState[i].posPulse);
    client.print(F(",\"target\":")); client.print(servoState[i].targetPulse);
    client.print(F(",\"moving\":")); client.print(servoState[i].moving ? F("true") : F("false"));
    client.print(F(",\"cont\":")); client.print(servoConfig[i].continuous ? F("true") : F("false"));
    client.print('}');
  }
  client.print(F("]}"));
}

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ServoConfig servoConfig[NUM_SERVOS];
ServoState servoState[NUM_SERVOS];

// Wave pattern state
bool waveActive = false;
uint8_t waveStartServo = 0;
uint8_t waveEndServo = 7;
uint16_t waveSpeed = 50;        // Period in ms per degree
uint8_t wavePhaseOffset = 30;   // Degrees offset between adjacent servos
uint16_t waveAmplitude = 90;    // Degrees of motion (center +/- amplitude/2)
uint16_t waveCenter = 90;       // Center position in degrees
unsigned long waveStartTime = 0;

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

// Default calibration values
#define DEFAULT_MIN 150
#define DEFAULT_MAX 600

/**
 * @brief Initialize per-servo calibration and runtime state to sane defaults.
 *
 * Sets each servo's min/max pulse to DEFAULT_MIN/DEFAULT_MAX, sets mode to standard
 * (not continuous), sets the stop/center pulse to the midpoint of the default range,
 * and resets all runtime state (current/target/start pulse, movement timers/flags,
 * and speed ramp fields).
 */
void initServoDefaults() {
  uint16_t defaultCenter = (DEFAULT_MIN + DEFAULT_MAX) / 2;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servoConfig[i].minPulse = DEFAULT_MIN;
    servoConfig[i].maxPulse = DEFAULT_MAX;
    servoConfig[i].continuous = false;
    servoConfig[i].stopPulse = defaultCenter;
    servoConfig[i].totalDegrees = 180;
    servoConfig[i].allowRelease = true;

    servoState[i].posPulse = defaultCenter;
    servoState[i].targetPulse = defaultCenter;
    servoState[i].startPulse = defaultCenter;
    servoState[i].moveStartMs = 0;
    servoState[i].moveDurationMs = 0;
    servoState[i].moving = false;

    servoState[i].targetSpeed = 0;
    servoState[i].startSpeed = 0;
    servoState[i].speedRampStartMs = 0;
    servoState[i].speedRampDurationMs = 0;
    servoState[i].speedRamping = false;
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

  initServoDefaults();
  applyCustomServoSetup(servoConfig, servoState);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  // Wi-Fi is best-effort. If it fails the servo loop still works over USB.
  wifiAndOtaBegin();
}

void loop() {
  if (otaReady) {
    ArduinoOTA.poll();
    webPoll();
    syncPoll();
  }

  // Pause animation work during an active OTA upload so the flash isn't
  // contended. Servo positions hold; sequences resume after reboot.
  if (!otaInProgress) {
    updateAnimations();
    updateSpeedRamps();
    updateWave();
    updateSequence();
    updateSpeedSequence();
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
