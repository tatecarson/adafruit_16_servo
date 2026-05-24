#pragma once

#include "servo_runtime.h"
#include "dc_motor.h"
#include "Sync.h"

void showHelp() {
  Serial.println(F("Commands: S/P/CAL/SWEEP/TPULSE/STATUS"));
  Serial.println(F("UP/DOWN/ROTATE"));
  Serial.println(F("MOTION/PLAY/SPLAY/RUN/STOP/TIMESCALE"));
  Serial.println(F("STORAGEINFO/BOARDID. See README for syntax."));
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

bool startPositionSequence(uint8_t seqNum, bool loop, bool announce) {
  if (!selectPositionSequence(seqNum, currentSequence, currentSequenceLength)) {
    Serial.println(F("Unknown sequence"));
    return false;
  }

  cancelMotionPlayback();
  cancelSequencePlayback();
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
  cancelSequencePlayback();
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
  cancelSequencePlayback();
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
    if (space > 0 && cmd[space + 1] != '\0') {
      // Disambiguate by argument shape:
      //   RUN 1          → legacy in-firmware program by number
      //   RUN evening-arc → schema v1 Sequence from active EEPROM bake
      // Schema id regex (^[a-z][a-z0-9-]{0,31}$) forbids leading
      // digits, so a digit-prefixed token unambiguously routes to the
      // legacy path. Trailing " LOOP" works for both.
      char first = cmd[space + 1];
      bool loop = containsStr(cmd, "LOOP");
      if (first >= '0' && first <= '9') {
        uint8_t programNum = atoi(cmd + space + 1);
        startSequenceProgram(programNum, loop);
      } else {
        // Extract just the id token (stop at space or null) so the
        // optional LOOP suffix doesn't get passed to the loader.
        char idBuf[SEQ_ID_MAX_LEN + 1] = {0};
        int i = 0;
        int src = space + 1;
        while (cmd[src] && cmd[src] != ' ' && i < (int)sizeof(idBuf) - 1) {
          idBuf[i++] = cmd[src++];
        }
        startSequenceFromStorage(idBuf, loop, true);
      }
    } else {
      Serial.println(F("Use: RUN <n> | RUN <id> [LOOP]"));
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
    cancelSequencePlayback();
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
      cancelSequencePlayback();
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
      // RUN (both legacy numeric and schema-v1 id forms) is mirrored
      // so all boards start the same sequence in lock-step. Per-step
      // target filtering happens inside each board's runner.
      || startsWith(upperCmd, "RUN ") || strcmp(upperCmd, "RUN") == 0
      || startsWith(upperCmd, "ROTATE ") || strcmp(upperCmd, "ROTATE") == 0
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
