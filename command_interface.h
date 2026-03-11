#pragma once

#include "servo_runtime.h"

void showHelp() {
  Serial.println(F("\n--- Commands ---"));
  Serial.println(F("S<n> <deg>       Move servo n to degrees"));
  Serial.println(F("P<n> <pulse>     Move servo n to raw pulse"));
  Serial.println(F("CAL <n> <min> <max>  Set calibration"));
  Serial.println(F("SWEEP <n>        Test sweep servo n"));
  Serial.println(F("CENTER <n>       Move to center (90 deg) / stop"));
  Serial.println(F("OFF <n>          Turn off servo n (blocked on protected winches)"));
  Serial.println(F("RELEASE <n>      Force-release servo n"));
  Serial.println(F("MOVE <n> <deg> <ms>  Animated move (eased)"));
  Serial.println(F("L<n> <pct>       Move servo n to percent of travel"));
  Serial.println(F("LMOVE <n> <pct> <ms> Animated move to percent"));
  Serial.println(F("UP <n> <pct>     Move servo n to absolute percent up"));
  Serial.println(F("DOWN <n> <pct>   Move servo n to absolute percent down"));
  Serial.println(F("UMOVE <n> <pct> <ms> Animated move to absolute percent up"));
  Serial.println(F("DMOVE <n> <pct> <ms> Animated move to absolute percent down"));
  Serial.println(F("ALLUP <pct> [ms] Move all protected winches up together"));
  Serial.println(F("ALLDOWN <pct> [ms] Move all protected winches down together"));
  Serial.println(F("RIG <UP|DOWN> <pct> <spd> [ms]  Winches + rotation test"));
  Serial.println(F("WAVE <s> <e> [spd] [off] [amp]  Start wave pattern"));
  Serial.println(F("PLAY <n> [LOOP]      Play sequence n"));
  Serial.println(F("SPLAY <n> [LOOP]     Speed sequence (continuous)"));
  Serial.println(F("STOP                 Stop wave/sequence"));
  Serial.println(F("TIMESCALE <n>        Scale sequence timing n times slower"));
  Serial.println(F("MODE <n> STD|CONT    Set servo mode"));
  Serial.println(F("SPEED <n> <-100:100> Continuous servo speed"));
  Serial.println(F("STATUS           Show all servos"));
  Serial.println();
}

// Helper: trim leading/trailing whitespace in place
void trimString(char* str) {
  char* start = str;
  while (*start && isspace(*start)) start++;
  if (start != str) memmove(str, start, strlen(start) + 1);

  char* end = str + strlen(str) - 1;
  while (end > str && isspace(*end)) *end-- = '\0';
}

// Helper: convert string to uppercase in place
void toUpperCase(char* str) {
  while (*str) {
    *str = toupper(*str);
    str++;
  }
}

// Helper: find character in string starting at offset, returns -1 if not found
int findChar(const char* str, char c, int startIdx) {
  const char* p = strchr(str + startIdx, c);
  return p ? (p - str) : -1;
}

// Helper: check if string starts with prefix
bool startsWith(const char* str, const char* prefix) {
  return strncmp(str, prefix, strlen(prefix)) == 0;
}

// Helper: check if string contains substring
bool containsStr(const char* str, const char* substr) {
  return strstr(str, substr) != nullptr;
}

bool parseRigDirection(const char* cmd, bool& percentUp, uint8_t& percent, int8_t& speed, uint32_t& duration) {
  char direction[6] = {0};
  int speedIn = 0;
  int parsed = sscanf(cmd, "RIG %5s %hhu %d %lu", direction, &percent, &speedIn, &duration);
  if (parsed < 4) {
    duration = 0;
    parsed = sscanf(cmd, "RIG %5s %hhu %d", direction, &percent, &speedIn);
  }

  if (parsed < 3) {
    Serial.println(F("Use: RIG <UP|DOWN> <pct> <spd> [ms]"));
    return false;
  }

  if (strcmp(direction, "UP") == 0) {
    percentUp = true;
  } else if (strcmp(direction, "DOWN") == 0) {
    percentUp = false;
  } else {
    Serial.println(F("RIG direction must be UP or DOWN"));
    return false;
  }

  percent = constrain(percent, 0, 100);
  speed = (int8_t)constrain(speedIn, -100, 100);
  return true;
}

void processCommand(char* cmd) {
  trimString(cmd);
  toUpperCase(cmd);

  if (cmd[0] == 'S' && cmd[1] >= '0' && cmd[1] <= '9') {
    int space = findChar(cmd, ' ', 0);
    if (space > 1) {
      uint8_t servo = atoi(cmd + 1);
      uint16_t degrees = atoi(cmd + space + 1);
      setServoDegrees(servo, degrees);
    }
  }
  else if (cmd[0] == 'L' && cmd[1] >= '0' && cmd[1] <= '9') {
    int space = findChar(cmd, ' ', 0);
    if (space > 1) {
      uint8_t servo = atoi(cmd + 1);
      uint8_t percent = atoi(cmd + space + 1);
      setServoPercent(servo, percent);
      Serial.print(F("Servo ")); Serial.print(servo);
      Serial.print(F(" -> ")); Serial.print(percent);
      Serial.println(F("% of travel"));
    }
  }
  else if (startsWith(cmd, "UP")) {
    int space1 = findChar(cmd, ' ', 3);
    if (space1 > 0) {
      uint8_t servo = atoi(cmd + 3);
      uint8_t percent = atoi(cmd + space1 + 1);
      setServoPercentUp(servo, percent);
      Serial.print(F("Servo ")); Serial.print(servo);
      Serial.print(F(" -> ")); Serial.print(percent);
      Serial.println(F("% up"));
    }
  }
  else if (startsWith(cmd, "DOWN")) {
    int space1 = findChar(cmd, ' ', 5);
    if (space1 > 0) {
      uint8_t servo = atoi(cmd + 5);
      uint8_t percent = atoi(cmd + space1 + 1);
      setServoPercent(servo, percent);
      Serial.print(F("Servo ")); Serial.print(servo);
      Serial.print(F(" -> ")); Serial.print(percent);
      Serial.println(F("% down"));
    }
  }
  else if (startsWith(cmd, "ALLUP")) {
    int space1 = findChar(cmd, ' ', 0);
    int space2 = (space1 > 0) ? findChar(cmd, ' ', space1 + 1) : -1;
    if (space1 > 0) {
      uint8_t percent = atoi(cmd + space1 + 1);
      if (space2 > 0) {
        uint16_t duration = atoi(cmd + space2 + 1);
        moveAllProtectedWinchesPercent(true, percent, duration);
        Serial.print(F("Moving all protected winches to "));
        Serial.print(percent);
        Serial.print(F("% up over "));
        Serial.print(duration);
        Serial.println(F("ms"));
      } else {
        setAllProtectedWinchesPercent(true, percent);
        Serial.print(F("All protected winches -> "));
        Serial.print(percent);
        Serial.println(F("% up"));
      }
    }
  }
  else if (startsWith(cmd, "ALLDOWN")) {
    int space1 = findChar(cmd, ' ', 0);
    int space2 = (space1 > 0) ? findChar(cmd, ' ', space1 + 1) : -1;
    if (space1 > 0) {
      uint8_t percent = atoi(cmd + space1 + 1);
      if (space2 > 0) {
        uint16_t duration = atoi(cmd + space2 + 1);
        moveAllProtectedWinchesPercent(false, percent, duration);
        Serial.print(F("Moving all protected winches to "));
        Serial.print(percent);
        Serial.print(F("% down over "));
        Serial.print(duration);
        Serial.println(F("ms"));
      } else {
        setAllProtectedWinchesPercent(false, percent);
        Serial.print(F("All protected winches -> "));
        Serial.print(percent);
        Serial.println(F("% down"));
      }
    }
  }
  else if (startsWith(cmd, "RIG")) {
    bool percentUp = false;
    uint8_t percent = 0;
    int8_t speed = 0;
    uint32_t duration = 0;
    uint8_t rotationServo = 0;
    if (parseRigDirection(cmd, percentUp, percent, speed, duration) &&
        setTestRigState(percentUp, percent, speed, duration, rotationServo)) {
      Serial.print(F("Rig test: winches -> "));
      Serial.print(percent);
      Serial.print(percentUp ? F("% up, ") : F("% down, "));
      Serial.print(F("rotation servo "));
      Serial.print(rotationServo);
      Serial.print(F(" -> "));
      Serial.print(speed);
      Serial.print(F("%"));
      if (duration > 0) {
        Serial.print(F(" over "));
        Serial.print(duration);
        Serial.print(F("ms"));
      }
      Serial.println();
    }
  }
  else if (startsWith(cmd, "PLAY")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t seqNum = atoi(cmd + space + 1);
      sequenceLoop = containsStr(cmd, "LOOP");

      if (!selectPositionSequence(seqNum, currentSequence, currentSequenceLength)) {
        Serial.println(F("Unknown sequence"));
        return;
      }

      sequenceStartTime = millis();
      lastTriggeredKeyframe = 0;
      sequenceActive = true;
      waveActive = false;

      Serial.print(F("Playing sequence ")); Serial.print(seqNum);
      if (sequenceLoop) Serial.print(F(" (looping)"));
      Serial.println();
    }
  }
  else if (startsWith(cmd, "SPLAY")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t seqNum = atoi(cmd + space + 1);
      bool loop = containsStr(cmd, "LOOP");

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
  else if (cmd[0] == 'P' && cmd[1] >= '0' && cmd[1] <= '9') {
    int space = findChar(cmd, ' ', 0);
    if (space > 1) {
      uint8_t servo = atoi(cmd + 1);
      uint16_t pulse = atoi(cmd + space + 1);
      setServoPulse(servo, pulse);
    }
  }
  else if (startsWith(cmd, "CAL")) {
    int space1 = findChar(cmd, ' ', 4);
    int space2 = (space1 > 0) ? findChar(cmd, ' ', space1 + 1) : -1;
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = atoi(cmd + 4);
      uint16_t minVal = atoi(cmd + space1 + 1);
      uint16_t maxVal = atoi(cmd + space2 + 1);
      setCalibration(servo, minVal, maxVal);
    }
  }
  else if (startsWith(cmd, "SWEEP")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t servo = atoi(cmd + space + 1);
      sweepServo(servo);
    }
  }
  else if (startsWith(cmd, "CENTER")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t servo = atoi(cmd + space + 1);
      if (servo < NUM_SERVOS && servoConfig[servo].continuous) {
        pwm.setPWM(servo, 0, servoConfig[servo].stopPulse);
        servoState[servo].posPulse = servoConfig[servo].stopPulse;
        Serial.print(F("Servo ")); Serial.print(servo);
        Serial.println(F(" stopped"));
      } else {
        setServoDegrees(servo, servoConfig[servo].totalDegrees / 2);
      }
    }
  }
  else if (startsWith(cmd, "OFF")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t servo = atoi(cmd + space + 1);
      servoOff(servo);
    }
  }
  else if (startsWith(cmd, "RELEASE")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t servo = atoi(cmd + space + 1);
      releaseServo(servo);
    }
  }
  else if (startsWith(cmd, "MOVE") || startsWith(cmd, "M ")) {
    int idx = startsWith(cmd, "MOVE") ? 5 : 2;
    int space1 = findChar(cmd, ' ', idx);
    int space2 = (space1 > 0) ? findChar(cmd, ' ', space1 + 1) : -1;
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = atoi(cmd + idx);
      uint16_t degrees = atoi(cmd + space1 + 1);
      uint16_t duration = atoi(cmd + space2 + 1);
      moveServoDegrees(servo, degrees, duration);
      Serial.print(F("Moving servo ")); Serial.print(servo);
      Serial.print(F(" to ")); Serial.print(degrees);
      Serial.print(F(" deg over ")); Serial.print(duration);
      Serial.println(F("ms"));
    }
  }
  else if (startsWith(cmd, "LMOVE")) {
    int space1 = findChar(cmd, ' ', 6);
    int space2 = (space1 > 0) ? findChar(cmd, ' ', space1 + 1) : -1;
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = atoi(cmd + 6);
      uint8_t percent = atoi(cmd + space1 + 1);
      uint16_t duration = atoi(cmd + space2 + 1);
      moveServoPercent(servo, percent, duration);
      Serial.print(F("Moving servo ")); Serial.print(servo);
      Serial.print(F(" to ")); Serial.print(percent);
      Serial.print(F("% over ")); Serial.print(duration);
      Serial.println(F("ms"));
    }
  }
  else if (startsWith(cmd, "UMOVE")) {
    int space1 = findChar(cmd, ' ', 6);
    int space2 = (space1 > 0) ? findChar(cmd, ' ', space1 + 1) : -1;
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = atoi(cmd + 6);
      uint8_t percent = atoi(cmd + space1 + 1);
      uint16_t duration = atoi(cmd + space2 + 1);
      moveServoPercentUp(servo, percent, duration);
      Serial.print(F("Moving servo ")); Serial.print(servo);
      Serial.print(F(" to ")); Serial.print(percent);
      Serial.print(F("% up over ")); Serial.print(duration);
      Serial.println(F("ms"));
    }
  }
  else if (startsWith(cmd, "DMOVE")) {
    int space1 = findChar(cmd, ' ', 6);
    int space2 = (space1 > 0) ? findChar(cmd, ' ', space1 + 1) : -1;
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = atoi(cmd + 6);
      uint8_t percent = atoi(cmd + space1 + 1);
      uint16_t duration = atoi(cmd + space2 + 1);
      moveServoPercent(servo, percent, duration);
      Serial.print(F("Moving servo ")); Serial.print(servo);
      Serial.print(F(" to ")); Serial.print(percent);
      Serial.print(F("% down over ")); Serial.print(duration);
      Serial.println(F("ms"));
    }
  }
  else if (startsWith(cmd, "WAVE")) {
    int s1 = findChar(cmd, ' ', 5);
    int s2 = (s1 > 0) ? findChar(cmd, ' ', s1 + 1) : -1;
    int s3 = (s2 > 0) ? findChar(cmd, ' ', s2 + 1) : -1;
    int s4 = (s3 > 0) ? findChar(cmd, ' ', s3 + 1) : -1;

    if (s1 > 0 && s2 > 0) {
      waveStartServo = atoi(cmd + 5);
      waveEndServo = atoi(cmd + s1 + 1);

      if (waveStartServo > waveEndServo || waveEndServo >= NUM_SERVOS) {
        Serial.println(F("Invalid servo range"));
        return;
      }

      if (s3 > 0) waveSpeed = atoi(cmd + s2 + 1);
      else waveSpeed = 50;

      if (s4 > 0) wavePhaseOffset = atoi(cmd + s3 + 1);
      else wavePhaseOffset = 30;

      if (s4 > 0) waveAmplitude = atoi(cmd + s4 + 1);
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
  else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "STOP ALL") == 0) {
    waveActive = false;
    sequenceActive = false;
    speedSeqActive = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      servoState[i].moving = false;
      servoState[i].speedRamping = false;
      if (servoConfig[i].continuous) {
        setServoSpeed(i, 0);
      }
    }
    Serial.println(F("Stopped"));
  }
  else if (startsWith(cmd, "MODE")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t servo = atoi(cmd + space + 1);
      if (servo >= NUM_SERVOS) {
        Serial.println(F("Invalid servo"));
      } else if (containsStr(cmd, "CONT")) {
        servoConfig[servo].continuous = true;
        servoConfig[servo].stopPulse = (servoConfig[servo].minPulse + servoConfig[servo].maxPulse) / 2;
        Serial.print(F("Servo ")); Serial.print(servo);
        Serial.print(F(" set to CONTINUOUS (stop="));
        Serial.print(servoConfig[servo].stopPulse);
        Serial.println(F(") - use SPEED to control"));
      } else if (containsStr(cmd, "STD")) {
        servoConfig[servo].continuous = false;
        Serial.print(F("Servo ")); Serial.print(servo);
        Serial.println(F(" set to STANDARD"));
      } else {
        Serial.println(F("Use: MODE <n> STD or MODE <n> CONT"));
      }
    }
  }
  else if (startsWith(cmd, "SPEED")) {
    int space1 = findChar(cmd, ' ', 6);
    if (space1 > 0) {
      uint8_t servo = atoi(cmd + 6);
      int16_t speed = atoi(cmd + space1 + 1);
      setServoSpeed(servo, (int8_t)constrain(speed, -100, 100));
    } else {
      Serial.println(F("Use: SPEED <n> <-100 to 100>"));
    }
  }
  else if (startsWith(cmd, "TIMESCALE")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint16_t val = (uint16_t)atoi(cmd + space + 1);
      if (val < 1) val = 1;
      timeMultiplier = val;
      Serial.print(F("Time multiplier set to ")); Serial.println(timeMultiplier);
    } else {
      Serial.print(F("TIMESCALE: ")); Serial.println(timeMultiplier);
    }
  }
  else if (startsWith(cmd, "STATUS")) {
    showStatus();
  }
  else if (startsWith(cmd, "HELP")) {
    showHelp();
  }
  else if (strlen(cmd) > 0) {
    Serial.println(F("Unknown command. Type HELP"));
  }
}
