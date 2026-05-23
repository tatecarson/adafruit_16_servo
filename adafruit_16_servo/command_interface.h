#pragma once

#include "servo_runtime.h"
#include "dc_motor.h"
#include "Sync.h"

void showHelp() {
  Serial.println(F("\n--- Commands ---"));
  Serial.println(F("Testing / Calibration:"));
  Serial.println(F("S<n> <deg>            Move servo n to degrees"));
  Serial.println(F("P<n> <pulse>          Move servo n to raw pulse"));
  Serial.println(F("TPULSE <pulse>        Set servos 0-2 to one raw pulse"));
  Serial.println(F("CAL <n> <min> <max>   Set calibration"));
  Serial.println(F("SWEEP <n>             Test sweep servo n"));
  Serial.println(F("OFF <n>               Turn off servo n (blocked on protected winches)"));
  Serial.println(F("RELEASE <n>           Force-release servo n"));
  Serial.println(F("STATUS                Show all servos"));
  Serial.println();
  Serial.println(F("Performance / Installation:"));
  Serial.println(F("UP <n> <pct>          Move servo n to absolute percent up"));
  Serial.println(F("DOWN <n> <pct>        Move servo n to absolute percent down"));
  Serial.println(F("MOVE <n> <deg> <ms>   Animated move (eased)"));
  Serial.println(F("UMOVE <n> <pct> <ms>  Animated move to absolute percent up"));
  Serial.println(F("DMOVE <n> <pct> <ms>  Animated move to absolute percent down"));
  Serial.println(F("ALLUP <pct> [ms]      Move all protected winches up together"));
  Serial.println(F("ALLDOWN <pct> [ms]    Move all protected winches down together"));
  Serial.println(F("RIG <UP|DOWN> <pct> <spd> [ms]  Winches + DC motor test"));
  Serial.println(F("MOTION <id>           Play baked browser Motion"));
  Serial.println(F("PLAY <n> [LOOP]       Play sequence n"));
  Serial.println(F("SPLAY <n> [LOOP]      Speed sequence (DC motor)"));
  Serial.println(F("RUN <n> [LOOP]        Run a chained sequence program"));
  Serial.println(F("STOP [n]              Stop all motion or hold one servo"));
  Serial.println(F("TIMESCALE <n>         Scale sequence timing n times slower"));
  Serial.println(F("ROTATE <spd>          DC motor rotation speed"));
  Serial.println(F("STORAGEINFO           Show EEPROM slot + boardId state"));
  Serial.println(F("BOARDID [n]           Get or set boardId (1..3)"));
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
  unsigned long durationIn = 0;
  int parsed = sscanf(cmd, "RIG %5s %hhu %d %lu", direction, &percent, &speedIn, &durationIn);
  if (parsed < 4) {
    durationIn = 0;
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
  duration = (uint32_t)durationIn;
  return true;
}

bool startPositionSequence(uint8_t seqNum, bool loop, bool announce) {
  if (!selectPositionSequence(seqNum, currentSequence, currentSequenceLength)) {
    Serial.println(F("Unknown sequence"));
    return false;
  }

  cancelMotionPlayback();
  sequenceLoop = loop;
  sequenceStartTime = millis();
  lastTriggeredKeyframe = 0;
  sequenceActive = true;

  if (announce) {
    Serial.print(F("Playing sequence "));
    Serial.print(seqNum);
    if (loop) Serial.print(F(" (looping)"));
    Serial.println();
  }
  return true;
}

bool startSpeedSequence(uint8_t seqNum, bool loop, bool announce) {
  if (!selectSpeedSequence(seqNum, currentSpeedSeq, currentSpeedSeqLength)) {
    Serial.println(F("Unknown speed sequence"));
    return false;
  }

  speedSeqActive = true;
  cancelMotionPlayback();
  speedSeqLoop = loop;
  speedSeqStartTime = millis();
  lastTriggeredSpeedFrame = 0;

  if (announce) {
    Serial.print(F("Playing speed sequence "));
    Serial.print(seqNum);
    if (loop) Serial.print(F(" (looping)"));
    Serial.println();
  }
  return true;
}

bool startSequenceProgram(uint8_t programNum, bool loop) {
  if (!selectSequenceProgram(programNum, currentProgram)) {
    Serial.println(F("Unknown program"));
    return false;
  }

  sequenceActive = false;
  speedSeqActive = false;
  cancelMotionPlayback();
  programActive = true;
  programLoop = loop;
  programPositionDone = (currentProgram->positionLength == 0);
  programSpeedDone = (currentProgram->speedLength == 0);
  currentProgramPositionStepIndex = 0;
  currentProgramPositionIteration = 0;
  currentProgramSpeedStepIndex = 0;
  currentProgramSpeedIteration = 0;

  Serial.print(F("Running program "));
  Serial.print(programNum);
  if (loop) Serial.print(F(" (looping)"));
  Serial.println();

  if (!programPositionDone) startNextProgramPositionStep();
  if (!programSpeedDone) startNextProgramSpeedStep();
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
    if (parseRigDirection(cmd, percentUp, percent, speed, duration) &&
        setTestRigState(percentUp, percent, speed, duration)) {
      Serial.print(F("Rig test: winches -> "));
      Serial.print(percent);
      Serial.print(percentUp ? F("% up, ") : F("% down, "));
      Serial.print(F("motor -> "));
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
      programActive = false;
      speedSeqActive = false;
      startPositionSequence(seqNum, containsStr(cmd, "LOOP"), true);
    }
  }
  else if (startsWith(cmd, "SPLAY")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t seqNum = atoi(cmd + space + 1);
      programActive = false;
      sequenceActive = false;
      startSpeedSequence(seqNum, containsStr(cmd, "LOOP"), true);
    }
  }
  else if (startsWith(cmd, "MOTION")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0 && cmd[space + 1] != '\0') {
      startMotionFromStorage(cmd + space + 1, true);
    } else {
      Serial.println(F("Use: MOTION <id>"));
    }
  }
  else if (startsWith(cmd, "RUN")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t programNum = atoi(cmd + space + 1);
      startSequenceProgram(programNum, containsStr(cmd, "LOOP"));
    }
  }
  else if (startsWith(cmd, "TPULSE")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint16_t pulse = atoi(cmd + space + 1);
      setTestPulse(pulse);
      Serial.print(F("Test pulse -> "));
      Serial.println(pulse);
    } else {
      Serial.println(F("Use: TPULSE <pulse>"));
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
    // WAVE command removed for OTA partition headroom (servo-dz7). Kept as
    // a recognized no-op so old gallery scripts don't error out — they just
    // get an informational reply instead of executing.
    Serial.println(F("WAVE not supported in this firmware"));
  }
  else if (startsWith(cmd, "STOP ")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      uint8_t servo = atoi(cmd + space + 1);
      stopServoNow(servo);
    }
  }
  else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "STOP ALL") == 0) {
    sequenceActive = false;
    speedSeqActive = false;
    programActive = false;
    cancelMotionPlayback();
    stopMotor();
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      servoState[i].moving = false;
      servoState[i].stopped = false;
    }
    Serial.println(F("Stopped"));
  }
  else if (startsWith(cmd, "ROTATE")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0) {
      int16_t speed = atoi(cmd + space + 1);
      cancelMotionPlayback();
      setMotorSpeed((int8_t)constrain(speed, -100, 100));
    } else {
      Serial.println(F("Use: ROTATE <-100 to 100>"));
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
  else if (startsWith(cmd, "BOARDID")) {
    const char* rest = cmd + 7; while (*rest == ' ') rest++;
    if (*rest == '\0') {
        Serial.print(F("boardId=")); Serial.println(storageBoardId());
    } else {
        int v = atoi(rest);
        if (storageSetBoardId((uint8_t)v)) Serial.print(F("set boardId="));
        else Serial.print(F("bad boardId="));
        Serial.println(v);
    }
    return;
  }
  else if (startsWith(cmd, "STORAGEINFO")) {
    Serial.print(F("boardId=")); Serial.println(storageBoardId());
    Serial.print(F("hasActive=")); Serial.println(storageHasActive() ? F("yes") : F("no"));
    Serial.print(F("hasPrevious=")); Serial.println(storageHasPrevious() ? F("yes") : F("no"));
    Serial.print(F("slotPayloadMax=")); Serial.println((int)STORAGE_PAYLOAD_MAX);
    return;
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

// Whitelist of commands that should be mirrored to all peers when issued
// locally. Per-servo pokes (S0 90, P1 ...) stay local-only on purpose.
// Compared after trim+uppercase.
inline bool shouldMirrorCommand(const char* upperCmd) {
  return startsWith(upperCmd, "PLAY ") || strcmp(upperCmd, "PLAY") == 0
      || startsWith(upperCmd, "SPLAY ") || strcmp(upperCmd, "SPLAY") == 0
      || startsWith(upperCmd, "MOTION ") || strcmp(upperCmd, "MOTION") == 0
      || startsWith(upperCmd, "ROTATE ") || strcmp(upperCmd, "ROTATE") == 0
      || startsWith(upperCmd, "RIG ") || strcmp(upperCmd, "RIG") == 0
      || strcmp(upperCmd, "STOP") == 0;
}

// Single entry point used by Serial input, the HTTP /cmd handler, and the
// UDP sync receive path. fromNetwork=true means "this came from a peer";
// in that case we run the command locally but never re-broadcast (avoiding
// feedback loops). When fromNetwork=false we run locally AND, if the
// command is in the mirror whitelist, broadcast its canonical form.
void dispatchCommand(const char* cmd, bool fromNetwork) {
  if (cmd == nullptr || cmd[0] == '\0') return;

  // processCommand mutates its argument; keep an untouched canonical copy
  // for the broadcast payload + mirror check.
  char buf[64];
  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char upper[64];
  strncpy(upper, buf, sizeof(upper) - 1);
  upper[sizeof(upper) - 1] = '\0';
  trimString(upper);
  toUpperCase(upper);

  processCommand(buf);

  if (!fromNetwork && shouldMirrorCommand(upper)) {
    broadcastEvent(upper);
  }
}
