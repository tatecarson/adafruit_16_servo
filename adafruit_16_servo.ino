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
  uint8_t servo;      // Which servo (255 = all servos use same pos)
  uint8_t degrees;    // Target position
  uint16_t time;      // Time offset from sequence start (ms)
  uint16_t duration;  // Move duration (ms)
};

// Example sequence storage (modify for your choreography)
#define MAX_KEYFRAMES 32
Keyframe sequence1[MAX_KEYFRAMES] = {
  {0, 0, 0, 500},       // Servo 0 to 0° at t=0, over 500ms
  {1, 0, 100, 500},     // Servo 1 to 0° at t=100ms
  {2, 0, 200, 500},     // Servo 2 to 0° at t=200ms
  {0, 180, 1000, 500},  // Servo 0 to 180° at t=1s
  {1, 180, 1100, 500},
  {2, 180, 1200, 500},
  {0, 90, 2000, 500},   // Return to center
  {1, 90, 2100, 500},
  {2, 90, 2200, 500},
  {255, 0, 0, 0}        // End marker (servo=255)
};
uint8_t sequence1Length = 9;

// Speed sequence structure for continuous servos
struct SpeedFrame {
  uint8_t servo;      // Which servo (255 = end marker)
  int8_t speed;       // Target speed (-100 to 100, 0 = stop)
  uint16_t time;      // Time offset from sequence start (ms)
  uint16_t rampMs;    // Ramp duration to reach speed (0 = instant)
};

// Example speed sequence storage
#define MAX_SPEEDFRAMES 32
SpeedFrame speedSeq1[MAX_SPEEDFRAMES] = {
  {0, 50, 0, 500},       // Servo 0 ramp to 50% over 500ms at t=0
  {1, -50, 0, 500},      // Servo 1 ramp to -50% (opposite direction)
  {0, 0, 3000, 500},     // Servo 0 ramp to stop at t=3s
  {1, 0, 3000, 500},     // Servo 1 ramp to stop
  {0, -50, 4000, 500},   // Reverse directions
  {1, 50, 4000, 500},
  {0, 0, 7000, 500},     // Stop
  {1, 0, 7000, 500},
  {255, 0, 7500, 0}      // End marker (triggers after last frame completes)
};
uint8_t speedSeq1Length = 9;  // Includes end marker

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

// Convert degrees (0-180) to pulse value for a specific servo
uint16_t degreesToPulse(uint8_t servo, uint8_t degrees) {
  degrees = constrain(degrees, 0, 180);
  return map(degrees, 0, 180, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
}

// Convert speed (-100 to 100) to pulse for continuous servo
// 0 = stop, negative = one direction, positive = other direction
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

// Move servo to a pulse value
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

// Set continuous servo speed (-100 to 100, 0 = stop)
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

// Ramp continuous servo to target speed over duration
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

// Start an animated move to target position over duration ms
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

// Update all servo animations - call from loop()
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

// Update all active speed ramps - call from loop()
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

// Update wave pattern - call from loop()
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

// Update speed sequence playback - call from loop()
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

// Sweep a servo through its full range
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

// Set calibration for a servo
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

// Show status of all servos with non-default calibration
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

      // Select sequence (add more sequences as needed)
      if (seqNum == 1) {
        currentSequence = sequence1;
        currentSequenceLength = sequence1Length;
      } else {
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

      // Select speed sequence
      if (seqNum == 1) {
        currentSpeedSeq = speedSeq1;
        currentSpeedSeqLength = speedSeq1Length;
      } else {
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
