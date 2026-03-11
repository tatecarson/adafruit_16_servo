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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include "servo_setup.h"

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

#include "sequence_setup.h"

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

#include "servo_control.h"

/**
 * @brief Advance active servo animations and apply resulting PWM updates.
 *
 * For each servo with motion in progress, either completes the animation by
 * setting the servo to its target pulse and clearing the moving flag, or
 * interpolates the current pulse (using the configured easing) and writes the
 * updated pulse to the PWM driver.
 */
void updateAnimations() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servoState[i].moving) continue;

    unsigned long elapsed = now - servoState[i].moveStartMs;

    if (elapsed >= servoState[i].moveDurationMs) {
      // Animation complete
      servoState[i].posPulse = servoState[i].targetPulse;
      pwm.setPWM(i, 0, servoState[i].posPulse);
      servoState[i].moving = false;
    } else {
      // Interpolate position
      float progress = (float)elapsed / (float)servoState[i].moveDurationMs;
      uint16_t newPos = lerpEased(servoState[i].startPulse, servoState[i].targetPulse, progress);
      if (newPos != servoState[i].posPulse) {
        servoState[i].posPulse = newPos;
        pwm.setPWM(i, 0, newPos);
      }
    }
  }
}

/**
 * @brief Advance and apply active speed ramps for continuous servos.
 *
 * Checks each servo for an active speed ramp, computes progress based on millis(),
 * applies a cubic ease-in-out interpolation between the stored start and target speeds,
 * updates the servo PWM output and posPulse accordingly, and completes ramps by
 * setting the final speed and clearing the ramping flag when the ramp duration elapses.
 */
void updateSpeedRamps() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servoState[i].speedRamping) continue;

    unsigned long elapsed = now - servoState[i].speedRampStartMs;

    if (elapsed >= servoState[i].speedRampDurationMs) {
      // Ramp complete
      setServoSpeed(i, servoState[i].targetSpeed);
      servoState[i].speedRamping = false;
    } else {
      // Calculate eased speed
      float t = (float)elapsed / servoState[i].speedRampDurationMs;
      // Cubic ease in-out (same as position animation)
      if (t < 0.5) {
        t = 4 * t * t * t;
      } else {
        t = 1 - pow(-2 * t + 2, 3) / 2;
      }

      int8_t speed = servoState[i].startSpeed + (servoState[i].targetSpeed - servoState[i].startSpeed) * t;

      // Set speed without printing (to avoid spam)
      uint16_t pulse = speedToPulse(i, speed);
      servoState[i].posPulse = pulse;
      pwm.setPWM(i, 0, pulse);
    }
  }
}

/**
 * @brief Applies an active sine-wave motion across a contiguous range of servos.
 *
 * When wave mode is active, computes a time- and index-offset sine pattern using
 * the global wave parameters (waveStartTime, waveSpeed, wavePhaseOffset,
 * waveAmplitude, waveCenter, waveStartServo, waveEndServo) and updates each
 * servo's PWM output to the corresponding calibrated pulse. No action is taken
 * if wave mode is not active. Each servo channel is written only when the
 * computed pulse differs from its current position.
 */
void updateWave() {
  if (!waveActive) return;

  unsigned long elapsed = millis() - waveStartTime;

  for (uint8_t i = waveStartServo; i <= waveEndServo; i++) {
    // Calculate phase for this servo
    uint8_t servoIndex = i - waveStartServo;
    float phase = (float)(elapsed) / (float)(waveSpeed * 360 / 1000);
    phase += (float)(servoIndex * wavePhaseOffset) / 360.0f;

    // Sine wave: -1 to 1
    float sineVal = sin(phase * 2.0f * PI);

    // Map to servo position
    float degrees = waveCenter + (sineVal * (float)waveAmplitude / 2.0f);
    uint16_t pulse = degreesToPulse(i, (uint16_t)constrain(degrees, 0, servoConfig[i].totalDegrees));

    if (pulse != servoState[i].posPulse) {
      servoState[i].posPulse = pulse;
      pwm.setPWM(i, 0, pulse);
    }
  }
}

// Update sequence playback - call from loop()
// Reads keyframes from PROGMEM using memcpy_P
void updateSequence() {
  if (!sequenceActive || currentSequence == nullptr) return;

  unsigned long elapsed = millis() - sequenceStartTime;
  Keyframe kf;  // Local copy for PROGMEM read

  // Find keyframes to trigger
  for (uint8_t i = lastTriggeredKeyframe; i < currentSequenceLength; i++) {
    memcpy_P(&kf, &currentSequence[i], sizeof(Keyframe));

    // Time not reached yet — stop scanning (frames are in ascending order)
    if (elapsed < (uint32_t)kf.time * timeMultiplier) break;

    // End marker check (time-gated: only reached when elapsed >= end marker time)
    if (kf.servo == SEQUENCE_END_MARKER_SERVO) {
      if (sequenceLoop) {
        sequenceStartTime = millis();
        lastTriggeredKeyframe = 0;
        Serial.println(F("Sequence looping"));
      } else {
        sequenceActive = false;
        Serial.println(F("Sequence complete"));
      }
      return;
    }

    // Trigger this keyframe (scale duration by timeMultiplier)
    moveServoDegrees(kf.servo, kf.degrees, (uint32_t)kf.duration * timeMultiplier);
    lastTriggeredKeyframe = i + 1;
  }
}

// Update speed sequence playback - call from loop()
// Reads speed frames from PROGMEM using memcpy_P
void updateSpeedSequence() {
  if (!speedSeqActive || currentSpeedSeq == nullptr) return;

  unsigned long elapsed = millis() - speedSeqStartTime;
  SpeedFrame sf;  // Local copy for PROGMEM read

  // Find speed frames to trigger
  for (uint8_t i = lastTriggeredSpeedFrame; i < currentSpeedSeqLength; i++) {
    memcpy_P(&sf, &currentSpeedSeq[i], sizeof(SpeedFrame));

    // Time not reached yet — stop scanning (frames are in ascending order)
    if (elapsed < (uint32_t)sf.time * timeMultiplier) break;

    // End marker check (time-gated: only reached when elapsed >= end marker time)
    if (sf.servo == SEQUENCE_END_MARKER_SERVO) {
      if (speedSeqLoop) {
        speedSeqStartTime = millis();
        lastTriggeredSpeedFrame = 0;
        Serial.println(F("Speed sequence looping"));
      } else {
        speedSeqActive = false;
        // Stop all continuous servos
        for (uint8_t j = 0; j < NUM_SERVOS; j++) {
          if (servoConfig[j].continuous) {
            setServoSpeed(j, 0);
          }
        }
        Serial.println(F("Speed sequence complete"));
      }
      return;
    }

    // Trigger regular speed frame (scale ramp duration by timeMultiplier)
    rampServoSpeed(sf.servo, sf.speed, (uint32_t)sf.rampMs * timeMultiplier);
    lastTriggeredSpeedFrame = i + 1;
  }
}

// Sweep a servo through its full range (BLOCKING - for calibration/debug only)
// Note: Uses delay() which blocks all other operations for several seconds.
// This is intentional for calibration where you want to observe the full range.
void sweepServo(uint8_t servo) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }

  Serial.print(F("Sweeping servo ")); Serial.println(servo);
  Serial.print(F("Range: ")); Serial.print(servoConfig[servo].minPulse);
  Serial.print(F(" - ")); Serial.println(servoConfig[servo].maxPulse);

  // Sweep to max
  for (uint16_t p = servoConfig[servo].minPulse; p <= servoConfig[servo].maxPulse; p += 2) {
    pwm.setPWM(servo, 0, p);
    delay(5);
  }
  delay(300);

  // Sweep to min (loop condition avoids uint16_t underflow)
  for (uint16_t p = servoConfig[servo].maxPulse; p > servoConfig[servo].minPulse + 1; p -= 2) {
    pwm.setPWM(servo, 0, p);
    delay(5);
  }
  pwm.setPWM(servo, 0, servoConfig[servo].minPulse);  // Ensure we hit min exactly

  // Return to center
  uint16_t center = (servoConfig[servo].minPulse + servoConfig[servo].maxPulse) / 2;
  pwm.setPWM(servo, 0, center);
  servoState[servo].posPulse = center;
  Serial.println(F("Sweep complete"));
}

/**
 * @brief Update the pulse calibration range for a servo.
 *
 * Sets the servo's minimum and maximum PWM pulse values used for degree and speed conversions.
 * If `servo` is outside the valid range [0, NUM_SERVOS-1], the call returns without modifying any calibration.
 *
 * @param servo Servo channel index (0 .. NUM_SERVOS-1).
 * @param minVal Minimum pulse value for the servo.
 * @param maxVal Maximum pulse value for the servo.
 */
void setCalibration(uint8_t servo, uint16_t minVal, uint16_t maxVal) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  servoConfig[servo].minPulse = minVal;
  servoConfig[servo].maxPulse = maxVal;
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" calibration: ")); Serial.print(minVal);
  Serial.print(F(" - ")); Serial.println(maxVal);
}

/**
 * @brief Print the current configuration and position for every servo to Serial.
 *
 * Prints each servo's index, mode (STD or CONT), calibrated minimum and maximum pulse
 * values, and current pulse position. For continuous servos, also prints the configured
 * stop (center) pulse.
 */
void showStatus() {
  Serial.println(F("\n--- Servo Status ---"));
  Serial.print(F("Time multiplier: ")); Serial.println(timeMultiplier);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("Servo ")); Serial.print(i);
    Serial.print(servoConfig[i].continuous ? F(" [CONT]") : F(" [STD] "));
    Serial.print(F(": min=")); Serial.print(servoConfig[i].minPulse);
    Serial.print(F(" max=")); Serial.print(servoConfig[i].maxPulse);
    Serial.print(F(" pos=")); Serial.print(servoState[i].posPulse);
    Serial.print(F(" range=0-")); Serial.print(servoConfig[i].totalDegrees);
    if (servoConfig[i].continuous) {
      Serial.print(F(" stop=")); Serial.print(servoConfig[i].stopPulse);
    }
    Serial.println();
  }
  Serial.println();
}

#include "command_interface.h"

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
