/***************************************************
  Adafruit 16-channel PWM & Servo driver - Calibration & Control

  Based on Adafruit example code.
  ------> http://www.adafruit.com/products/815

  Serial Commands:
    S<n> <pos>     - Move servo n to position (0-180 degrees)
    P<n> <pulse>   - Move servo n to raw pulse (150-600)
    CAL <n> <min> <max> - Set servo n calibration (pulse values)
    SWEEP <n>      - Test sweep servo n
    CENTER <n>     - Move servo n to center (90 degrees)
    OFF <n>        - Turn off servo n
    MOVE <n> <deg> <ms> - Animated move with easing
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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define NUM_SERVOS 16
#define SERVO_FREQ 50

struct ServoConfig {
  uint16_t minPulse;
  uint16_t maxPulse;
  bool continuous;      // true = continuous rotation servo
  uint16_t stopPulse;   // Center/stop pulse for continuous servos
};

struct ServoState {
  uint16_t posPulse;     // Current position (pulse)

  // Position animation
  uint16_t targetPulse;
  uint16_t startPulse;
  unsigned long moveStartMs;
  uint16_t moveDurationMs;
  bool moving;

  // Continuous speed ramping (-100..100, 0=stop)
  int8_t targetSpeed;
  int8_t startSpeed;
  unsigned long speedRampStartMs;
  uint16_t speedRampDurationMs;
  bool speedRamping;
};

#include "servo_setup.h"

ServoConfig servoConfig[NUM_SERVOS];
ServoState servoState[NUM_SERVOS];

// Wave pattern state
bool waveActive = false;
uint8_t waveStartServo = 0;
uint8_t waveEndServo = 7;
uint16_t waveSpeed = 50;        // Period in ms per degree
uint8_t wavePhaseOffset = 30;   // Degrees offset between adjacent servos
uint8_t waveAmplitude = 90;     // Degrees of motion (center +/- amplitude/2)
uint8_t waveCenter = 90;        // Center position in degrees
unsigned long waveStartTime = 0;

// Keyframe sequence structure
struct Keyframe {
  uint8_t servo;      // Which servo (0-15), 255 = end marker
  uint8_t degrees;    // Target position
  uint16_t time;      // Time offset from sequence start (ms)
  uint16_t duration;  // Move duration (ms)
};

// Speed sequence structure for continuous servos
struct SpeedFrame {
  uint8_t servo;      // Which servo (255 = end marker)
  int8_t speed;       // Target speed (-100 to 100, 0 = stop)
  uint16_t time;      // Time offset from sequence start (ms)
  uint16_t rampMs;    // Ramp duration to reach speed (0 = instant)
};

#include "sequence_setup.h"

// Sequence playback state
bool sequenceActive = false;
bool sequenceLoop = false;
Keyframe* currentSequence = nullptr;
uint8_t currentSequenceLength = 0;
unsigned long sequenceStartTime = 0;
uint8_t lastTriggeredKeyframe = 0;

// Speed sequence playback state
bool speedSeqActive = false;
bool speedSeqLoop = false;
SpeedFrame* currentSpeedSeq = nullptr;
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

// Serial input buffer
String inputString = "";
bool stringComplete = false;

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
  inputString.reserve(50);
}

/**
 * @brief Convert a servo angle (0–180°) to the calibrated PWM pulse for a specific servo.
 *
 * Uses the servo's configured min/max pulse calibration to produce a pulse value.
 *
 * @param servo Servo channel index (0..NUM_SERVOS-1).
 * @param degrees Desired angle in degrees; values are clamped to the 0–180 range.
 * @return uint16_t Pulse value mapped into the servo's configured minPulse..maxPulse range.
 */
uint16_t degreesToPulse(uint8_t servo, uint8_t degrees) {
  degrees = constrain(degrees, 0, 180);
  return map(degrees, 0, 180, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
}

// Convert speed (-100 to 100) to pulse for continuous servo
/**
 * Convert a signed speed (-100..100) into the calibrated PWM pulse for a servo.
 *
 * Maps 0 to the servo's configured stop pulse; positive speeds 1..100 map linearly
 * from stopPulse+1 up to the servo's configured maxPulse; negative speeds -1..-100
 * map linearly from stopPulse-1 down to the servo's configured minPulse.
 *
 * @param servo Index of the servo whose calibration (minPulse, maxPulse, stopPulse) is used.
 * @param speed Desired speed in the range -100 to 100, where 0 is stop, negative is one direction, positive the other.
 * @return uint16_t PWM pulse value within the servo's calibrated min/max corresponding to the requested speed; `stopPulse` when speed is 0.
 */
uint16_t speedToPulse(uint8_t servo, int8_t speed) {
  speed = constrain(speed, -100, 100);
  uint16_t stopPulse = servoConfig[servo].stopPulse;
  if (speed == 0) {
    return stopPulse;
  } else if (speed > 0) {
    // Map 1-100 to range from stopPulse to max pulse
    return map(speed, 1, 100, stopPulse + 1, servoConfig[servo].maxPulse);
  } else {
    // Map -100 to -1 to range from min pulse to stopPulse
    return map(speed, -100, -1, servoConfig[servo].minPulse, stopPulse - 1);
  }
}

// Easing function: ease-in-out cubic for smooth organic motion
// t = progress (0.0 to 1.0), returns eased value (0.0 to 1.0)
float easeInOutCubic(float t) {
  if (t < 0.5f) {
    return 4.0f * t * t * t;
  } else {
    float f = (2.0f * t) - 2.0f;
    return 0.5f * f * f * f + 1.0f;
  }
}

// Linear interpolation with easing
uint16_t lerpEased(uint16_t start, uint16_t end, float progress) {
  float easedProgress = easeInOutCubic(progress);
  return start + (uint16_t)((float)(end - start) * easedProgress);
}

/**
 * @brief Move a servo channel to the given PWM pulse (within the servo's calibrated range).
 *
 * Clamps the provided pulse to the servo's configured min/max, updates the servo's stored
 * position pulse, and writes the PWM value to the driver. Prints an error if the servo
 * index is out of range and prints the resulting pulse when applied.
 *
 * @param servo Index of the servo channel (0..NUM_SERVOS-1).
 * @param pulse Desired PWM pulse value (will be constrained to the servo's calibration).
 */
void setServoPulse(uint8_t servo, uint16_t pulse) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  pulse = constrain(pulse, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
  servoState[servo].posPulse = pulse;
  pwm.setPWM(servo, 0, pulse);
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" -> pulse ")); Serial.println(pulse);
}

// Move servo to degrees
void setServoDegrees(uint8_t servo, uint8_t degrees) {
  uint16_t pulse = degreesToPulse(servo, degrees);
  setServoPulse(servo, pulse);
}

/**
 * @brief Set the speed of a continuous-rotation servo and apply the corresponding PWM.
 *
 * Updates the servo's stored pulse position and writes the computed pulse for the
 * requested speed to the PWM driver. If the servo index is out of range or the
 * servo is not configured as continuous, no change is applied.
 *
 * @param servo Index of the servo channel (0..NUM_SERVOS-1).
 * @param speed Desired speed from -100 to 100, where 0 is stop.
 */
void setServoSpeed(uint8_t servo, int8_t speed) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  if (!servoConfig[servo].continuous) {
    Serial.println(F("Not a continuous servo (use MODE command)"));
    return;
  }
  uint16_t pulse = speedToPulse(servo, speed);
  servoState[servo].posPulse = pulse;
  pwm.setPWM(servo, 0, pulse);
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" speed ")); Serial.print(speed);
  Serial.print(F("% -> pulse ")); Serial.println(pulse);
}

/**
 * @brief Ramp a continuous-rotation servo from its current speed to a target speed.
 *
 * Approximates the servo's current speed from its last PWM pulse and either applies
 * the target speed immediately (when rampMs is 0) or starts a timed speed ramp that
 * will be progressed by the regular updateSpeedRamps() loop.
 *
 * @param servo Index of the servo channel (0..NUM_SERVOS-1).
 * @param targetSpeed Desired speed in percent, from -100 (full reverse) to 100 (full forward).
 * @param rampMs Duration of the speed ramp in milliseconds; 0 applies the target speed instantly.
 */
void rampServoSpeed(uint8_t servo, int8_t targetSpeed, uint16_t rampMs) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  if (!servoConfig[servo].continuous) {
    Serial.println(F("Not a continuous servo"));
    return;
  }

  // Get current speed from pulse
  int8_t currentSpeed = 0;
  if (servoState[servo].posPulse != servoConfig[servo].stopPulse) {
    // Approximate current speed from pulse position
    if (servoState[servo].posPulse > servoConfig[servo].stopPulse) {
      currentSpeed = map(servoState[servo].posPulse, servoConfig[servo].stopPulse, servoConfig[servo].maxPulse, 0, 100);
    } else {
      currentSpeed = map(servoState[servo].posPulse, servoConfig[servo].minPulse, servoConfig[servo].stopPulse, -100, 0);
    }
  }

  if (rampMs == 0) {
    // Instant speed change
    setServoSpeed(servo, targetSpeed);
    servoState[servo].speedRamping = false;
  } else {
    // Start ramping
    servoState[servo].startSpeed = currentSpeed;
    servoState[servo].targetSpeed = targetSpeed;
    servoState[servo].speedRampStartMs = millis();
    servoState[servo].speedRampDurationMs = rampMs;
    servoState[servo].speedRamping = true;

    Serial.print(F("Servo ")); Serial.print(servo);
    Serial.print(F(" ramping ")); Serial.print(currentSpeed);
    Serial.print(F("% -> ")); Serial.print(targetSpeed);
    Serial.print(F("% over ")); Serial.print(rampMs);
    Serial.println(F("ms"));
  }
}

/**
 * @brief Initiates an animated move of a servo from its current position to a target pulse over a given duration.
 *
 * The target pulse is clamped to the servo's configured min/max pulse range and the servo's animation state is
 * initialized so subsequent update calls will perform the interpolation.
 *
 * @param servo Index of the servo channel (0-based).
 * @param targetPulse Desired PWM pulse value; will be constrained to the servo's configured range.
 * @param duration Duration of the animation in milliseconds.
 */
void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint16_t duration) {
  if (servo >= NUM_SERVOS) return;
  targetPulse = constrain(targetPulse, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);

  servoState[servo].startPulse = servoState[servo].posPulse;
  servoState[servo].targetPulse = targetPulse;
  servoState[servo].moveDurationMs = duration;
  servoState[servo].moveStartMs = millis();
  servoState[servo].moving = true;
}

// Animated move using degrees
void moveServoDegrees(uint8_t servo, uint8_t degrees, uint16_t duration) {
  uint16_t pulse = degreesToPulse(servo, degrees);
  moveServoAnimated(servo, pulse, duration);
}

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
    uint16_t pulse = degreesToPulse(i, (uint8_t)constrain(degrees, 0, 180));

    if (pulse != servoState[i].posPulse) {
      servoState[i].posPulse = pulse;
      pwm.setPWM(i, 0, pulse);
    }
  }
}

// Update sequence playback - call from loop()
void updateSequence() {
  if (!sequenceActive || currentSequence == nullptr) return;

  unsigned long elapsed = millis() - sequenceStartTime;

  // Find keyframes to trigger
  for (uint8_t i = lastTriggeredKeyframe; i < currentSequenceLength; i++) {
    Keyframe* kf = &currentSequence[i];

    // End marker check
    if (kf->servo == 255) {
      if (sequenceLoop) {
        // Restart sequence
        sequenceStartTime = millis();
        lastTriggeredKeyframe = 0;
        Serial.println(F("Sequence looping"));
      } else {
        sequenceActive = false;
        Serial.println(F("Sequence complete"));
      }
      return;
    }

    // Trigger keyframe if time reached
    if (elapsed >= kf->time && i >= lastTriggeredKeyframe) {
      moveServoDegrees(kf->servo, kf->degrees, kf->duration);
      lastTriggeredKeyframe = i + 1;
    }
  }
}

/**
 * @brief Advance and apply the active speed-frame sequence.
 *
 * Checks the currently selected speed sequence against elapsed time and triggers scheduled speed changes.
 * When a frame's time is reached, the function initiates a speed ramp for the frame's servo using the frame's
 * target speed and ramp duration. A frame with servo value 255 marks the sequence end: if looping is enabled the
 * sequence restarts, otherwise playback stops and all continuous servos are commanded to stop (speed 0).
 *
 * @note Intended to be called regularly from the main loop.
 */
void updateSpeedSequence() {
  if (!speedSeqActive || currentSpeedSeq == nullptr) return;

  unsigned long elapsed = millis() - speedSeqStartTime;

  // Find speed frames to trigger
  for (uint8_t i = lastTriggeredSpeedFrame; i < currentSpeedSeqLength; i++) {
    SpeedFrame* sf = &currentSpeedSeq[i];

    // Trigger speed frame if time reached
    if (elapsed >= sf->time) {
      // End marker check
      if (sf->servo == 255) {
        if (speedSeqLoop) {
          // Restart sequence
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

      // Trigger regular speed frame
      rampServoSpeed(sf->servo, sf->speed, sf->rampMs);
      lastTriggeredSpeedFrame = i + 1;
    }
  }
}

/**
 * @brief Sweeps a servo across its calibrated pulse range and returns it to center.
 *
 * Performs a full sweep from the servo's configured minPulse to maxPulse and back, writing PWM
 * values to the channel with short delays for visible motion. Prints progress and range info to
 * Serial and updates servoState[servo].posPulse to the center pulse when complete.
 *
 * @param servo Servo channel index (0..NUM_SERVOS-1). If the index is invalid the function prints
 *              an error and returns without modifying outputs.
 */
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

  // Sweep to min
  for (uint16_t p = servoConfig[servo].maxPulse; p >= servoConfig[servo].minPulse; p -= 2) {
    pwm.setPWM(servo, 0, p);
    delay(5);
    if (p < servoConfig[servo].minPulse + 2) break;  // Prevent underflow
  }

  // Return to center
  uint16_t center = (servoConfig[servo].minPulse + servoConfig[servo].maxPulse) / 2;
  pwm.setPWM(servo, 0, center);
  servoState[servo].posPulse = center;
  Serial.println(F("Sweep complete"));
}

// Turn off a servo (stop sending PWM)
void servoOff(uint8_t servo) {
  if (servo >= NUM_SERVOS) return;
  pwm.setPWM(servo, 0, 0);
  Serial.print(F("Servo ")); Serial.print(servo); Serial.println(F(" off"));
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
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("Servo ")); Serial.print(i);
    Serial.print(servoConfig[i].continuous ? F(" [CONT]") : F(" [STD] "));
    Serial.print(F(": min=")); Serial.print(servoConfig[i].minPulse);
    Serial.print(F(" max=")); Serial.print(servoConfig[i].maxPulse);
    Serial.print(F(" pos=")); Serial.print(servoState[i].posPulse);
    if (servoConfig[i].continuous) {
      Serial.print(F(" stop=")); Serial.print(servoConfig[i].stopPulse);
    }
    Serial.println();
  }
  Serial.println();
}

void showHelp() {
  Serial.println(F("\n--- Commands ---"));
  Serial.println(F("S<n> <deg>       Move servo n to degrees (0-180)"));
  Serial.println(F("P<n> <pulse>     Move servo n to raw pulse"));
  Serial.println(F("CAL <n> <min> <max>  Set calibration"));
  Serial.println(F("SWEEP <n>        Test sweep servo n"));
  Serial.println(F("CENTER <n>       Move to center (90 deg) / stop"));
  Serial.println(F("OFF <n>          Turn off servo n"));
  Serial.println(F("MOVE <n> <deg> <ms>  Animated move (eased)"));
  Serial.println(F("WAVE <s> <e> [spd] [off] [amp]  Start wave pattern"));
  Serial.println(F("PLAY <n> [LOOP]      Play sequence n"));
  Serial.println(F("SPLAY <n> [LOOP]     Speed sequence (continuous)"));
  Serial.println(F("STOP                 Stop wave/sequence"));
  Serial.println(F("MODE <n> STD|CONT    Set servo mode"));
  Serial.println(F("SPEED <n> <-100:100> Continuous servo speed"));
  Serial.println(F("STATUS           Show all servos"));
  Serial.println();
}

/**
 * @brief Parse a single serial command string and execute the corresponding servo control action.
 *
 * Recognizes and executes commands for direct position/pulse control, calibration, animated moves,
 * waves, sequences, speed ramps, mode switching, status and help. Commands are parsed case-insensitively.
 *
 * @param cmd Raw command string received from the serial input (may include parameters separated by spaces).
 */
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("S") && cmd.charAt(1) >= '0' && cmd.charAt(1) <= '9') {
    // S<n> <degrees> - move servo to degrees (S0, S1, ... S15)
    int space = cmd.indexOf(' ');
    if (space > 1) {
      uint8_t servo = cmd.substring(1, space).toInt();
      uint8_t degrees = cmd.substring(space + 1).toInt();
      setServoDegrees(servo, degrees);
    }
  }
  else if (cmd.startsWith("P")) {
    // P<n> <pulse> - move servo to raw pulse
    int space = cmd.indexOf(' ');
    if (space > 1) {
      uint8_t servo = cmd.substring(1, space).toInt();
      uint16_t pulse = cmd.substring(space + 1).toInt();
      setServoPulse(servo, pulse);
    }
  }
  else if (cmd.startsWith("CAL")) {
    // CAL <n> <min> <max>
    int idx = 4;
    int space1 = cmd.indexOf(' ', idx);
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = cmd.substring(idx, space1).toInt();
      uint16_t minVal = cmd.substring(space1 + 1, space2).toInt();
      uint16_t maxVal = cmd.substring(space2 + 1).toInt();
      setCalibration(servo, minVal, maxVal);
    }
  }
  else if (cmd.startsWith("SWEEP")) {
    int space = cmd.indexOf(' ');
    if (space > 0) {
      uint8_t servo = cmd.substring(space + 1).toInt();
      sweepServo(servo);
    }
  }
  else if (cmd.startsWith("CENTER")) {
    int space = cmd.indexOf(' ');
    if (space > 0) {
      uint8_t servo = cmd.substring(space + 1).toInt();
      if (servo < NUM_SERVOS && servoConfig[servo].continuous) {
        // For continuous servo, center = stop
        pwm.setPWM(servo, 0, servoConfig[servo].stopPulse);
        servoState[servo].posPulse = servoConfig[servo].stopPulse;
        Serial.print(F("Servo ")); Serial.print(servo);
        Serial.println(F(" stopped"));
      } else {
        setServoDegrees(servo, 90);
      }
    }
  }
  else if (cmd.startsWith("OFF")) {
    int space = cmd.indexOf(' ');
    if (space > 0) {
      uint8_t servo = cmd.substring(space + 1).toInt();
      servoOff(servo);
    }
  }
  else if (cmd.startsWith("MOVE") || cmd.startsWith("M ")) {
    // MOVE <servo> <degrees> <duration_ms>
    // or M <servo> <degrees> <duration_ms>
    int idx = cmd.startsWith("MOVE") ? 5 : 2;
    int space1 = cmd.indexOf(' ', idx);
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = cmd.substring(idx, space1).toInt();
      uint8_t degrees = cmd.substring(space1 + 1, space2).toInt();
      uint16_t duration = cmd.substring(space2 + 1).toInt();
      moveServoDegrees(servo, degrees, duration);
      Serial.print(F("Moving servo ")); Serial.print(servo);
      Serial.print(F(" to ")); Serial.print(degrees);
      Serial.print(F(" deg over ")); Serial.print(duration);
      Serial.println(F("ms"));
    }
  }
  else if (cmd.startsWith("WAVE")) {
    // WAVE <start> <end> <speed> <offset> <amplitude>
    // Example: WAVE 0 7 50 30 90
    int idx = 5;
    int s1 = cmd.indexOf(' ', idx);
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);
    int s4 = cmd.indexOf(' ', s3 + 1);

    if (s1 > 0 && s2 > 0) {
      waveStartServo = cmd.substring(idx, s1).toInt();
      waveEndServo = cmd.substring(s1 + 1, s2).toInt();

      if (s3 > 0) waveSpeed = cmd.substring(s2 + 1, s3).toInt();
      else waveSpeed = 50;

      if (s4 > 0) wavePhaseOffset = cmd.substring(s3 + 1, s4).toInt();
      else wavePhaseOffset = 30;

      if (s4 > 0) waveAmplitude = cmd.substring(s4 + 1).toInt();
      else waveAmplitude = 90;

      waveStartTime = millis();
      waveActive = true;

      Serial.print(F("Wave: servos ")); Serial.print(waveStartServo);
      Serial.print(F("-")); Serial.print(waveEndServo);
      Serial.print(F(" speed=")); Serial.print(waveSpeed);
      Serial.print(F(" offset=")); Serial.print(wavePhaseOffset);
      Serial.print(F(" amp=")); Serial.println(waveAmplitude);
    }
  }
  else if (cmd.startsWith("PLAY")) {
    // PLAY <sequence_num> [LOOP]
    int space = cmd.indexOf(' ');
    if (space > 0) {
      uint8_t seqNum = cmd.substring(space + 1).toInt();
      sequenceLoop = (cmd.indexOf("LOOP") > 0);

      if (!selectPositionSequence(seqNum, currentSequence, currentSequenceLength)) {
        Serial.println(F("Unknown sequence"));
        return;
      }

      sequenceStartTime = millis();
      lastTriggeredKeyframe = 0;
      sequenceActive = true;
      waveActive = false;  // Stop wave if running

      Serial.print(F("Playing sequence ")); Serial.print(seqNum);
      if (sequenceLoop) Serial.print(F(" (looping)"));
      Serial.println();
    }
  }
  else if (cmd.startsWith("SPLAY")) {
    // SPLAY <sequence_num> [LOOP]
    int space = cmd.indexOf(' ');
    if (space > 0) {
      uint8_t seqNum = cmd.substring(space + 1).toInt();
      bool loop = (cmd.indexOf("LOOP") > 0);

      // Stop any running sequences
      speedSeqActive = false;
      sequenceActive = false;
      waveActive = false;

      if (!selectSpeedSequence(seqNum, currentSpeedSeq, currentSpeedSeqLength)) {
        Serial.println(F("Unknown speed sequence"));
        return;
      }

      speedSeqActive = true;
      speedSeqLoop = loop;
      speedSeqStartTime = millis();
      lastTriggeredSpeedFrame = 0;

      Serial.print(F("Playing speed sequence ")); Serial.print(seqNum);
      if (loop) Serial.print(F(" (looping)"));
      Serial.println();
    }
  }
  else if (cmd.startsWith("STOP")) {
    waveActive = false;
    sequenceActive = false;
    speedSeqActive = false;
    // Stop all speed ramps
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      servoState[i].speedRamping = false;
      if (servoConfig[i].continuous) {
        setServoSpeed(i, 0);
      }
    }
    Serial.println(F("Stopped"));
  }
  else if (cmd.startsWith("MODE")) {
    // MODE <n> STD|CONT
    int space = cmd.indexOf(' ');
    if (space > 0) {
      uint8_t servo = cmd.substring(space + 1).toInt();
      if (servo >= NUM_SERVOS) {
        Serial.println(F("Invalid servo"));
      } else if (cmd.indexOf("CONT") > 0) {
        servoConfig[servo].continuous = true;
        servoConfig[servo].stopPulse = (servoConfig[servo].minPulse + servoConfig[servo].maxPulse) / 2;
        // Don't send PWM - let user set speed manually
        // The calculated stop pulse may not be the servo's actual stop point
        Serial.print(F("Servo ")); Serial.print(servo);
        Serial.print(F(" set to CONTINUOUS (stop="));
        Serial.print(servoConfig[servo].stopPulse);
        Serial.println(F(") - use SPEED to control"));
      } else if (cmd.indexOf("STD") > 0) {
        servoConfig[servo].continuous = false;
        Serial.print(F("Servo ")); Serial.print(servo);
        Serial.println(F(" set to STANDARD"));
      } else {
        Serial.println(F("Use: MODE <n> STD or MODE <n> CONT"));
      }
    }
  }
  else if (cmd.startsWith("SPEED")) {
    // SPEED <n> <-100 to 100>
    // Parse: "SPEED 0 50" or "SPEED 0 -50" or "SPEED 0 0"
    int idx = 6;  // Skip "SPEED "
    int space1 = cmd.indexOf(' ', idx);
    if (space1 > 0) {
      uint8_t servo = cmd.substring(idx).toInt();
      int16_t speed = cmd.substring(space1 + 1).toInt();
      setServoSpeed(servo, (int8_t)constrain(speed, -100, 100));
    } else {
      Serial.println(F("Use: SPEED <n> <-100 to 100>"));
    }
  }
  else if (cmd.startsWith("STATUS")) {
    showStatus();
  }
  else if (cmd.startsWith("HELP")) {
    showHelp();
  }
  else if (cmd.length() > 0) {
    Serial.println(F("Unknown command. Type HELP"));
  }
}

void loop() {
  updateAnimations();
  updateSpeedRamps();
  updateWave();
  updateSequence();
  updateSpeedSequence();

  // Read serial input
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      stringComplete = true;
    } else {
      inputString += c;
    }
  }

  // Process complete command
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}