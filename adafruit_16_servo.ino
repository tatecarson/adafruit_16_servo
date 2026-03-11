/***************************************************
  Adafruit 16-channel PWM & Servo driver - Calibration & Control

  Based on Adafruit example code.
  ------> http://www.adafruit.com/products/815

  Serial Commands:
    S<n> <pos>     - Move servo n to position (degrees, range per servo)
    P<n> <pulse>   - Move servo n to raw pulse (150-600)
    CAL <n> <min> <max> - Set servo n calibration (pulse values)
    SWEEP <n>      - Test sweep servo n
    CENTER <n>     - Move servo n to center (90 degrees)
    OFF <n>        - Turn off servo n
    MOVE <n> <deg> <ms> - Animated move with easing
    L<n> <pct>     - Move servo n to percent of total travel
    LMOVE <n> <pct> <ms> - Animated move to percent of travel
    UP <n> <pct>   - Move servo n to absolute percent up
    DOWN <n> <pct> - Move servo n to absolute percent down
    UMOVE <n> <pct> <ms> - Animated move to absolute percent up
    DMOVE <n> <pct> <ms> - Animated move to absolute percent down
    ALLUP <pct> [ms]   - Move all protected winch servos up together
    ALLDOWN <pct> [ms] - Move all protected winch servos down together
    WAVE <start> <end> [speed] [offset] [amp] - Wave pattern
    PLAY <n> [LOOP]    - Play keyframe sequence
    SPLAY <n> [LOOP]   - Play speed sequence (continuous servos)
    STOP               - Stop wave/sequence
    MODE <n> STD|CONT  - Set servo mode (standard/continuous)
    SPEED <n> <-100 to 100> - Set continuous servo speed
    STATUS         - Show all servo calibrations
    HELP           - Show commands
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "servo_runtime.h"
#include "servo_setup.h"
#include "sequence_setup.h"

#include "servo_control.h"
#include "animation_engine.h"
#include "servo_maintenance.h"
#include "command_interface.h"

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
  Serial.begin(9600);
  Serial.println(F("Servo Calibration & Control"));
  Serial.println(F("Type HELP for commands"));
  Serial.println();

  initServoDefaults();
  applyCustomServoSetup(servoConfig, servoState);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);
}

void loop() {
  updateAnimations();
  updateSpeedRamps();
  updateWave();
  updateSequence();
  updateSpeedSequence();

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
    processCommand(inputBuffer);
    inputIndex = 0;
    inputComplete = false;
  }
}
