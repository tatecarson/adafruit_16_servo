#pragma once

#include "servo_runtime.h"
#include "dc_motor.h"
#include "Sync.h"
#include "servo_calibration.h"

// servo-vna: how far ahead a synchronized Motion is scheduled. The originator
// broadcasts MOTION_START with this lead and arms locally at millis()+LEAD;
// each peer arms at ITS OWN millis()+LEAD on receipt. The lead just has to
// exceed UDP broadcast propagation (~1-5ms on a LAN) so every board arms
// together; small enough to feel responsive.
#define MOTION_START_LEAD_MS 150

void showHelp() {
  Serial.println(F("Commands: S/P/CAL/SWEEP/TPULSE/STATUS"));
  Serial.println(F("CAL_GET/CAL_SET/CAL_RESET/CAL_PULSE (persisted calibration)"));
  Serial.println(F("UP/DOWN/UMOVE/DMOVE/ROTATE"));
  Serial.println(F("MOTION <id>/RUN <id>/STOP"));
  Serial.println(F("STORAGEINFO/BOARDID/GALLERY. See README for syntax."));
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

// Legacy startPositionSequence/startSpeedSequence/startSequenceProgram
// (PLAY/SPLAY/RUN <n>) removed in servo-voc.

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
  else if (startsWith(cmd, "DMOVE ")) {
    // DMOVE <ch> <pct> <duration_ms> — animated DOWN. Browser motion
    // editor uses this for smooth per-segment playback so the servo
    // glides between keyframes locally (firmware interpolates pulse)
    // instead of being slammed to each commanded position by per-tick
    // DOWN commands and then sitting idle between them (servo-79q).
    int s1 = findChar(cmd, ' ', 0);
    int s2 = (s1 > 0) ? findChar(cmd, ' ', s1 + 1) : -1;
    int s3 = (s2 > 0) ? findChar(cmd, ' ', s2 + 1) : -1;
    if (s1 > 0 && s2 > 0 && s3 > 0) {
      long ch = atol(cmd + s1 + 1);
      long pct = atol(cmd + s2 + 1);
      long dur = atol(cmd + s3 + 1);
      if (ch >= 0 && ch < NUM_SERVOS && pct >= 0 && pct <= 100 && dur > 0 && dur <= 60000) {
        moveServoPercent((uint8_t)ch, (uint8_t)pct, (uint32_t)dur);
        Serial.print(F("Servo ")); Serial.print((int)ch);
        Serial.print(F(" -> ")); Serial.print((int)pct);
        Serial.print(F("% down over ")); Serial.print((long)dur); Serial.println(F("ms"));
      } else {
        Serial.println(F("DMOVE rejected: 0<=ch<NUM_SERVOS, 0<=pct<=100, 0<dur<=60000"));
      }
    } else {
      Serial.println(F("Use: DMOVE <ch> <pct> <duration_ms>"));
    }
  }
  else if (startsWith(cmd, "UMOVE ")) {
    // UMOVE <ch> <pct> <duration_ms> — animated UP (mirror of DMOVE).
    int s1 = findChar(cmd, ' ', 0);
    int s2 = (s1 > 0) ? findChar(cmd, ' ', s1 + 1) : -1;
    int s3 = (s2 > 0) ? findChar(cmd, ' ', s2 + 1) : -1;
    if (s1 > 0 && s2 > 0 && s3 > 0) {
      long ch = atol(cmd + s1 + 1);
      long pct = atol(cmd + s2 + 1);
      long dur = atol(cmd + s3 + 1);
      if (ch >= 0 && ch < NUM_SERVOS && pct >= 0 && pct <= 100 && dur > 0 && dur <= 60000) {
        moveServoPercentUp((uint8_t)ch, (uint8_t)pct, (uint32_t)dur);
        Serial.print(F("Servo ")); Serial.print((int)ch);
        Serial.print(F(" -> ")); Serial.print((int)pct);
        Serial.print(F("% up over ")); Serial.print((long)dur); Serial.println(F("ms"));
      } else {
        Serial.println(F("UMOVE rejected: 0<=ch<NUM_SERVOS, 0<=pct<=100, 0<dur<=60000"));
      }
    } else {
      Serial.println(F("Use: UMOVE <ch> <pct> <duration_ms>"));
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
  // Legacy PLAY/SPLAY handlers (numbered keyframe/speed sequences) removed in
  // servo-voc. Use schema-v1 MOTION <id> / RUN <id> / RUN AUTO instead.
  else if (startsWith(cmd, "MOTION")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0 && cmd[space + 1] != '\0') {
      // Public form: MOTION <id>. Baked Sequences may append the compact
      // internal form `PREP <ms>` so the board glides to keyframe zero before
      // the real Motion begins, without storing bridge Motions in EEPROM.
      char idBuf[MOTION_ID_MAX_LEN + 1] = {0};
      const char* p = cmd + space + 1;
      while (*p == ' ') p++;
      int i = 0;
      while (*p && *p != ' ' && i < (int)sizeof(idBuf) - 1) {
        char c = *p++;
        if (c >= 'A' && c <= 'Z') c = (char)(c + ('a' - 'A'));
        idBuf[i++] = c;
      }
      while (*p == ' ') p++;

      if (*p == '\0') {
        startMotionFromStorage(idBuf, true);
      } else if (startsWith(p, "PREP ")) {
        long prepareMs = atol(p + 5);
        if (prepareMs > 0 && prepareMs <= 60000) {
          startMotionPreparedFromStorage(idBuf, (uint32_t)prepareMs, true);
        } else {
          Serial.println(F("MOTION PREP rejected: 0<ms<=60000"));
        }
      } else {
        Serial.println(F("Use: MOTION <id>"));
      }
    } else {
      Serial.println(F("Use: MOTION <id>"));
    }
  }
  else if (startsWith(cmd, "RUN")) {
    int space = findChar(cmd, ' ', 0);
    if (space > 0 && strcmp(cmd + space + 1, "AUTO") == 0) {
      // RUN AUTO — run the active Setlist forever (servo-dos). Leader-gated
      // inside startActiveSetlist; followers ride the mirrored RUN/STOP.
      startActiveSetlist();
    } else if (space > 0 && cmd[space + 1] != '\0') {
      // Schema-v1 only (legacy RUN <n> program runner removed in servo-voc):
      //   RUN evening-arc → Sequence from active EEPROM bake
      // Trailing " LOOP" loops the sequence.
      size_t cmdLen = strlen(cmd);
      bool loop = (cmdLen >= 5 && strcmp(cmd + cmdLen - 5, " LOOP") == 0);
      // Extract just the id token (stop at space or null) so the optional
      // LOOP suffix isn't passed to the loader. processCommand uppercased the
      // whole cmd, but schema ids are lowercase kebab-case, so re-lower as we
      // copy — keeps sequenceRunner.id / /status.json in the documented shape.
      char idBuf[SEQ_ID_MAX_LEN + 1] = {0};
      int i = 0;
      int src = space + 1;
      while (cmd[src] && cmd[src] != ' ' && i < (int)sizeof(idBuf) - 1) {
        char c = cmd[src++];
        if (c >= 'A' && c <= 'Z') c = (char)(c + ('a' - 'A'));
        idBuf[i++] = c;
      }
      startSequenceFromStorage(idBuf, loop, true);
    } else {
      Serial.println(F("Use: RUN <id> [LOOP] | RUN AUTO"));
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
  // Persisted calibration commands (servo-2n9). Distinct from the legacy
  // CAL <servo> <min> <max> path below which operates on PCA9685 ticks
  // and writes only to RAM. These take microseconds and persist to EEPROM.
  else if (strcmp(cmd, "CAL_GET") == 0) {
    calibrationPrintBootStatus();
  }
  else if (startsWith(cmd, "CAL_SET ")) {
    // CAL_SET <ch> <minUs> <maxUs> [<offsetDeg>]
    // Parse into wide signed temporaries so out-of-range inputs (negative,
    // > uint16, > int8) don't silently wrap before we validate them.
    int s1 = findChar(cmd, ' ', 0);
    int s2 = (s1 > 0) ? findChar(cmd, ' ', s1 + 1) : -1;
    int s3 = (s2 > 0) ? findChar(cmd, ' ', s2 + 1) : -1;
    int s4 = (s3 > 0) ? findChar(cmd, ' ', s3 + 1) : -1;
    if (s1 > 0 && s2 > 0 && s3 > 0) {
      long ch     = atol(cmd + s1 + 1);
      long minUs  = atol(cmd + s2 + 1);
      long maxUs  = atol(cmd + s3 + 1);
      long offset = (s4 > 0) ? atol(cmd + s4 + 1) : 0;
      if (ch < 0 || ch >= CALIB_NUM_CHANNELS) {
        Serial.print(F("CAL_SET rejected: ch must be 0.."));
        Serial.println((int)(CALIB_NUM_CHANNELS - 1));
      } else if (minUs < CALIB_BOUND_MIN_US || minUs > CALIB_BOUND_MAX_US ||
                 maxUs < CALIB_BOUND_MIN_US || maxUs > CALIB_BOUND_MAX_US) {
        Serial.print(F("CAL_SET rejected: minUs/maxUs must be "));
        Serial.print((int)CALIB_BOUND_MIN_US);
        Serial.print(F(".."));
        Serial.println((int)CALIB_BOUND_MAX_US);
      } else if (minUs >= maxUs) {
        Serial.println(F("CAL_SET rejected: minUs must be < maxUs"));
      } else if (offset < -CALIB_BOUND_OFFSET_DEG || offset > CALIB_BOUND_OFFSET_DEG) {
        Serial.print(F("CAL_SET rejected: offsetDeg must be -"));
        Serial.print((int)CALIB_BOUND_OFFSET_DEG);
        Serial.print(F("..+"));
        Serial.println((int)CALIB_BOUND_OFFSET_DEG);
      } else {
        calibrationSet((uint8_t)ch, (uint16_t)minUs, (uint16_t)maxUs, (int8_t)offset);
        calibrationApplyToServoConfig(servoConfig);
        Serial.print(F("CAL_SET S")); Serial.print((int)ch);
        Serial.print(F(" minUs=")); Serial.print((int)minUs);
        Serial.print(F(" maxUs=")); Serial.print((int)maxUs);
        Serial.print(F(" offsetDeg=")); Serial.println((int)offset);
      }
    } else {
      Serial.println(F("Use: CAL_SET <ch> <minUs> <maxUs> [<offsetDeg>]"));
    }
  }
  else if (startsWith(cmd, "CAL_RESET ")) {
    int s = findChar(cmd, ' ', 0);
    if (s > 0) {
      long ch = atol(cmd + s + 1);
      if (ch < 0 || ch >= CALIB_NUM_CHANNELS) {
        Serial.print(F("CAL_RESET rejected: ch must be 0.."));
        Serial.println((int)(CALIB_NUM_CHANNELS - 1));
      } else {
        calibrationReset((uint8_t)ch);
        Serial.print(F("CAL_RESET S")); Serial.println((int)ch);
        Serial.println(F("(takes effect after reboot — flag cleared but live servoConfig unchanged)"));
      }
    }
  }
  else if (startsWith(cmd, "CAL_PULSE ")) {
    // CAL_PULSE <ch> <us>  — raw microseconds, bypasses calibration.
    // Used by the // 08 UI to find each servo's true min/max pulse before
    // saving via CAL_SET. Not used by sequences or motion playback.
    // setPWM() takes 12-bit ticks; convert via calibUsToTicks so we don't
    // depend on Adafruit_PWMServoDriver::writeMicroseconds (which isn't
    // in the host-test mock).
    int s1 = findChar(cmd, ' ', 0);
    int s2 = (s1 > 0) ? findChar(cmd, ' ', s1 + 1) : -1;
    if (s1 > 0 && s2 > 0) {
      uint8_t ch = atoi(cmd + s1 + 1);
      uint16_t us = atoi(cmd + s2 + 1);
      if (ch < NUM_SERVOS && us >= 400 && us <= 2600) {
        pwm.setPWM(ch, 0, calibUsToTicks(us));
        Serial.print(F("CAL_PULSE S")); Serial.print(ch);
        Serial.print(F(" -> ")); Serial.print(us); Serial.println(F("us"));
      } else {
        Serial.println(F("Use: CAL_PULSE <ch> <us 400..2600>"));
      }
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
    cancelMotionPlayback();
    cancelSequencePlayback();
    cancelSetlistPlayback();   // RUN AUTO halts cleanly on STOP (servo-dos)
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
  // TIMESCALE removed in servo-voc — it only scaled legacy PLAY/SPLAY timing.
  // Schema-v1 Motions/Sequences author their timing directly in the browser.
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
  else if (startsWith(cmd, "GALLERY")) {
    // GALLERY            → print the persistent gallery_mode flag
    // GALLERY ON|OFF|1|0 → set it. Takes effect on the next boot's grace
    // window (servo-4gl). Mirror of the BOARDID handler.
    const char* rest = cmd + 7; while (*rest == ' ') rest++;
    if (*rest == '\0') {
      Serial.print(F("gallery=")); Serial.println(storageGalleryMode() ? F("on") : F("off"));
    } else if (strcmp(rest, "ON") == 0 || strcmp(rest, "1") == 0) {
      storageSetGalleryMode(true);
      Serial.println(F("gallery=on (auto RUN AUTO after grace on next boot)"));
    } else if (strcmp(rest, "OFF") == 0 || strcmp(rest, "0") == 0) {
      storageSetGalleryMode(false);
      Serial.println(F("gallery=off"));
    } else {
      Serial.println(F("Use: GALLERY [ON|OFF]"));
    }
    return;
  }
  else if (startsWith(cmd, "STORAGEINFO")) {
    Serial.print(F("boardId=")); Serial.println(storageBoardId());
    Serial.print(F("hasActive=")); Serial.println(storageHasActive() ? F("yes") : F("no"));
    Serial.print(F("hasPrevious=")); Serial.println(storageHasPrevious() ? F("yes") : F("no"));
    Serial.print(F("gallery=")); Serial.println(storageGalleryMode() ? F("on") : F("off"));
    Serial.print(F("slotPayloadMax=")); Serial.println((int)STORAGE_PAYLOAD_MAX);
    Serial.print(F("rollbackPayloadMax=")); Serial.println((int)STORAGE_DUAL_PAYLOAD_MAX);
    Serial.print(F("storageMode=")); Serial.println(storageActiveIsLarge() ? F("large") : F("dual"));
    Serial.print(F("rollbackSafe=")); Serial.println(storageActiveIsLarge() ? F("no") : F("yes"));
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
  // MOTION is intentionally NOT here: a locally-originated MOTION is sent as a
  // timestamped MOTION_START (synchronized cluster start, servo-vna) by the
  // dispatchCommand intercept below — not as a plain fire-on-arrival EVT.
      // RUN (both legacy numeric and schema-v1 id forms) is mirrored
      // so all boards start the same sequence in lock-step. Per-step
      // target filtering happens inside each board's runner.
  return startsWith(upperCmd, "RUN ") || strcmp(upperCmd, "RUN") == 0
      || startsWith(upperCmd, "ROTATE ") || strcmp(upperCmd, "ROTATE") == 0
      || strcmp(upperCmd, "STOP") == 0;
}

// Originate a synchronized Motion (servo-vna). Broadcast MOTION_START with the
// lead and arm locally at millis()+LEAD; peers arm at their own millis()+LEAD on
// receipt. Relative timing means no board's clock offset can desync the start —
// the only spread is UDP propagation (a few ms), well inside the 20ms budget.
inline void triggerMotionSynced(const char* motionId) {
  broadcastMotionStart(motionId, MOTION_START_LEAD_MS);
  startMotionFromStorageAt(motionId, millis() + MOTION_START_LEAD_MS, true);
}

// Called by Sync.cpp when a peer's MOTION_START arrives. localStartMs is already
// in this board's millis() frame (Sync.cpp converted it from the synced clock).
void onMotionStartSync(const char* motionId, unsigned long localStartMs) {
  startMotionFromStorageAt(motionId, localStartMs, false);
}

// Single entry point used by Serial input, the HTTP /cmd handler, and the
// UDP sync receive path. fromNetwork=true means "this came from a peer";
// in that case we run the command locally but never re-broadcast (avoiding
// feedback loops). When fromNetwork=false we run locally AND, if the
// command is in the mirror whitelist, broadcast its canonical form.
void dispatchCommand(const char* cmd, bool fromNetwork) {
  if (cmd == nullptr || cmd[0] == '\0') return;

  // Gallery Mode: any real command (serial, HTTP /cmd, or a peer's mirror)
  // means someone is in control, so cancel a pending boot auto-run before it
  // fires (servo-4gl). The scheduler's own RUN/STOP dispatch happens only
  // after the grace window already cleared, so this never self-cancels.
  galleryGraceCancel();

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

  // Synchronized Motion (servo-vna): a locally-originated `MOTION <id>` becomes
  // a cluster-synced start (broadcast MOTION_START + arm at a shared instant)
  // instead of an immediate local start. Peers arm via onMotionStartSync, so
  // MOTION is no longer EVT-mirrored. Sequence steps call processCommand()
  // directly (bypassing dispatchCommand), so each board still runs its own step
  // locally and never fans out a conflicting MOTION_START. A bare `MOTION` with
  // no id falls through to processCommand's usage message.
  if (!fromNetwork && startsWith(upper, "MOTION ")) {
    // Extract the id from `upper` (already trim+uppercased). Motion id matching
    // is case-insensitive on both the wire and at load, so the upper-cased id
    // is fine and sidesteps any leading-whitespace in the raw `buf`.
    char idBuf[40] = {0};
    int sp = findChar(upper, ' ', 0);
    if (sp > 0) {
      const char* p = upper + sp + 1;
      while (*p == ' ') p++;
      int i = 0;
      while (*p && *p != ' ' && i < (int)sizeof(idBuf) - 1) idBuf[i++] = *p++;
      idBuf[i] = '\0';
    }
    if (idBuf[0]) {
      triggerMotionSynced(idBuf);
      return;
    }
  }

  processCommand(buf);

  if (!fromNetwork && shouldMirrorCommand(upper)) {
    broadcastEvent(upper);
  }
}
