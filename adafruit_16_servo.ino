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
    STATUS         - Show all servo calibrations
    HELP           - Show commands
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define NUM_SERVOS 16
#define SERVO_FREQ 50

// Per-servo calibration (min/max pulse values)
uint16_t servoMin[NUM_SERVOS];
uint16_t servoMax[NUM_SERVOS];
uint16_t servoPos[NUM_SERVOS];  // Current position (pulse)

// Animation state per servo
uint16_t servoTarget[NUM_SERVOS];    // Target position
uint16_t servoStart[NUM_SERVOS];     // Starting position for current move
unsigned long servoMoveStart[NUM_SERVOS];  // When move started (millis)
uint16_t servoMoveDuration[NUM_SERVOS];    // Duration in ms (0 = instant)
bool servoMoving[NUM_SERVOS];        // Is this servo currently animating?

// Default calibration values
#define DEFAULT_MIN 150
#define DEFAULT_MAX 600

// Serial input buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Servo Calibration & Control"));
  Serial.println(F("Type HELP for commands"));
  Serial.println();

  // Initialize calibration to defaults
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servoMin[i] = DEFAULT_MIN;
    servoMax[i] = DEFAULT_MAX;
    servoPos[i] = (DEFAULT_MIN + DEFAULT_MAX) / 2;
    servoTarget[i] = servoPos[i];
    servoStart[i] = servoPos[i];
    servoMoveStart[i] = 0;
    servoMoveDuration[i] = 0;
    servoMoving[i] = false;
  }

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);
  inputString.reserve(50);
}

// Convert degrees (0-180) to pulse value for a specific servo
uint16_t degreesToPulse(uint8_t servo, uint8_t degrees) {
  degrees = constrain(degrees, 0, 180);
  return map(degrees, 0, 180, servoMin[servo], servoMax[servo]);
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
  pulse = constrain(pulse, servoMin[servo], servoMax[servo]);
  servoPos[servo] = pulse;
  pwm.setPWM(servo, 0, pulse);
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" -> pulse ")); Serial.println(pulse);
}

// Move servo to degrees
void setServoDegrees(uint8_t servo, uint8_t degrees) {
  uint16_t pulse = degreesToPulse(servo, degrees);
  setServoPulse(servo, pulse);
}

// Start an animated move to target position over duration ms
void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint16_t duration) {
  if (servo >= NUM_SERVOS) return;
  targetPulse = constrain(targetPulse, servoMin[servo], servoMax[servo]);

  servoStart[servo] = servoPos[servo];
  servoTarget[servo] = targetPulse;
  servoMoveDuration[servo] = duration;
  servoMoveStart[servo] = millis();
  servoMoving[servo] = true;
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
    if (!servoMoving[i]) continue;

    unsigned long elapsed = now - servoMoveStart[i];

    if (elapsed >= servoMoveDuration[i]) {
      // Animation complete
      servoPos[i] = servoTarget[i];
      pwm.setPWM(i, 0, servoPos[i]);
      servoMoving[i] = false;
    } else {
      // Interpolate position
      float progress = (float)elapsed / (float)servoMoveDuration[i];
      uint16_t newPos = lerpEased(servoStart[i], servoTarget[i], progress);
      if (newPos != servoPos[i]) {
        servoPos[i] = newPos;
        pwm.setPWM(i, 0, newPos);
      }
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
  Serial.print(F("Range: ")); Serial.print(servoMin[servo]);
  Serial.print(F(" - ")); Serial.println(servoMax[servo]);

  // Sweep to max
  for (uint16_t p = servoMin[servo]; p <= servoMax[servo]; p += 2) {
    pwm.setPWM(servo, 0, p);
    delay(5);
  }
  delay(300);

  // Sweep to min
  for (uint16_t p = servoMax[servo]; p >= servoMin[servo]; p -= 2) {
    pwm.setPWM(servo, 0, p);
    delay(5);
    if (p < servoMin[servo] + 2) break;  // Prevent underflow
  }

  // Return to center
  uint16_t center = (servoMin[servo] + servoMax[servo]) / 2;
  pwm.setPWM(servo, 0, center);
  servoPos[servo] = center;
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
  servoMin[servo] = minVal;
  servoMax[servo] = maxVal;
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" calibration: ")); Serial.print(minVal);
  Serial.print(F(" - ")); Serial.println(maxVal);
}

// Show status of all servos with non-default calibration
void showStatus() {
  Serial.println(F("\n--- Servo Status ---"));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("Servo ")); Serial.print(i);
    Serial.print(F(": min=")); Serial.print(servoMin[i]);
    Serial.print(F(" max=")); Serial.print(servoMax[i]);
    Serial.print(F(" pos=")); Serial.println(servoPos[i]);
  }
  Serial.println();
}

void showHelp() {
  Serial.println(F("\n--- Commands ---"));
  Serial.println(F("S<n> <deg>       Move servo n to degrees (0-180)"));
  Serial.println(F("P<n> <pulse>     Move servo n to raw pulse"));
  Serial.println(F("CAL <n> <min> <max>  Set calibration"));
  Serial.println(F("SWEEP <n>        Test sweep servo n"));
  Serial.println(F("CENTER <n>       Move to center (90 deg)"));
  Serial.println(F("OFF <n>          Turn off servo n"));
  Serial.println(F("MOVE <n> <deg> <ms>  Animated move (eased)"));
  Serial.println(F("STATUS           Show all servos"));
  Serial.println();
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("S") && cmd.charAt(1) != 'T' && cmd.charAt(1) != 'W') {
    // S<n> <degrees> - move servo to degrees
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
      setServoDegrees(servo, 90);
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
