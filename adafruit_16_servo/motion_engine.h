#pragma once

#include <string.h>
#include "servo_runtime.h"
#include "storage.h"
#include "dc_motor.h"

static bool motionAsciiEq(char a, char b) {
  if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
  if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
  return a == b;
}

static int motionSkipWs(const uint8_t* data, int pos, int end) {
  while (pos < end) {
    char c = (char)data[pos];
    if (c != ' ' && c != '\t' && c != '\n' && c != '\r') break;
    pos++;
  }
  return pos;
}

static bool motionKeyEquals(const uint8_t* data, int start, int end, const char* key) {
  int i = 0;
  for (int p = start; p < end; p++, i++) {
    if (key[i] == '\0' || (char)data[p] != key[i]) return false;
  }
  return key[i] == '\0';
}

static bool motionStringEqualsIgnoreCase(const uint8_t* data, int pos, int end, const char* value) {
  if (pos >= end || data[pos] != '"') return false;
  int p = pos + 1;
  int i = 0;
  while (p < end && data[p] != '"') {
    if (data[p] == '\\') return false;
    if (value[i] == '\0' || !motionAsciiEq((char)data[p], value[i])) return false;
    p++;
    i++;
  }
  return p < end && data[p] == '"' && value[i] == '\0';
}

static bool motionCopyString(const uint8_t* data, int pos, int end, char* out, uint8_t outLen) {
  if (pos >= end || data[pos] != '"' || outLen == 0) return false;
  int p = pos + 1;
  uint8_t n = 0;
  while (p < end && data[p] != '"') {
    if (data[p] == '\\' || n >= outLen - 1) return false;
    out[n++] = (char)data[p++];
  }
  if (p >= end || data[p] != '"') return false;
  out[n] = '\0';
  return true;
}

static bool motionFindValue(const uint8_t* data, int start, int end, const char* key, int& valuePos) {
  int curly = 0;
  int square = 0;
  bool inString = false;
  bool escape = false;
  int stringStart = -1;
  bool keyMatched = false;

  for (int p = start; p < end; p++) {
    char c = (char)data[p];
    if (escape) {
      escape = false;
      continue;
    }
    if (inString) {
      if (c == '\\') {
        escape = true;
      } else if (c == '"') {
        inString = false;
        if (curly == 0 && square == 0 && stringStart >= 0) {
          keyMatched = motionKeyEquals(data, stringStart, p, key);
        }
      }
      continue;
    }

    if (c == '"') {
      inString = true;
      stringStart = p + 1;
    } else if (c == '{') {
      curly++;
      keyMatched = false;
    } else if (c == '}') {
      curly--;
      keyMatched = false;
    } else if (c == '[') {
      square++;
      keyMatched = false;
    } else if (c == ']') {
      square--;
      keyMatched = false;
    } else if (c == ':' && curly == 0 && square == 0 && keyMatched) {
      valuePos = motionSkipWs(data, p + 1, end);
      return valuePos < end;
    } else if (c == ',') {
      keyMatched = false;
    }
  }
  return false;
}

static int motionFindContainerEnd(const uint8_t* data, int start, int end, char openChar, char closeChar) {
  if (start >= end || data[start] != openChar) return -1;
  int depth = 0;
  bool inString = false;
  bool escape = false;
  for (int p = start; p < end; p++) {
    char c = (char)data[p];
    if (escape) {
      escape = false;
      continue;
    }
    if (inString) {
      if (c == '\\') escape = true;
      else if (c == '"') inString = false;
      continue;
    }
    if (c == '"') inString = true;
    else if (c == openChar) depth++;
    else if (c == closeChar && --depth == 0) return p;
  }
  return -1;
}

static bool motionParseInteger(const uint8_t* data, int pos, int end, long& value) {
  pos = motionSkipWs(data, pos, end);
  bool neg = false;
  if (pos < end && data[pos] == '-') {
    neg = true;
    pos++;
  }
  if (pos >= end || data[pos] < '0' || data[pos] > '9') return false;
  long v = 0;
  while (pos < end && data[pos] >= '0' && data[pos] <= '9') {
    v = (v * 10) + (data[pos] - '0');
    pos++;
  }
  if (pos < end && data[pos] == '.') {
    pos++;
    while (pos < end && data[pos] >= '0' && data[pos] <= '9') pos++;
  }
  value = neg ? -v : v;
  return true;
}

static bool motionNextObjectInArray(const uint8_t* data, int arrayStart, int arrayEnd, int& pos, int& objStart, int& objEnd) {
  if (pos <= arrayStart) pos = arrayStart + 1;
  pos = motionSkipWs(data, pos, arrayEnd);
  if (pos < arrayEnd && data[pos] == ',') pos = motionSkipWs(data, pos + 1, arrayEnd);
  if (pos >= arrayEnd || data[pos] == ']') return false;
  if (data[pos] != '{') return false;
  objStart = pos;
  objEnd = motionFindContainerEnd(data, objStart, arrayEnd + 1, '{', '}');
  if (objEnd < 0) return false;
  pos = objEnd + 1;
  return true;
}

static bool motionParseKeyframe(
  const uint8_t* data,
  int objStart,
  int objEnd,
  MotionKeyframe& frame
) {
  int valuePos = 0;
  long parsed = 0;
  if (!motionFindValue(data, objStart + 1, objEnd, "atMs", valuePos) ||
      !motionParseInteger(data, valuePos, objEnd, parsed) ||
      parsed < 0) {
    return false;
  }
  frame.atMs = (uint32_t)parsed;

  if (!motionFindValue(data, objStart + 1, objEnd, "value", valuePos) ||
      !motionParseInteger(data, valuePos, objEnd, parsed)) {
    return false;
  }
  frame.value = (int16_t)parsed;
  return true;
}

static bool motionParseTrack(
  const uint8_t* data,
  int objStart,
  int objEnd,
  uint32_t motionDurationMs,
  MotionRuntime& out,
  bool& included
) {
  included = false;
  int valuePos = 0;
  uint8_t kind = MOTION_TRACK_NONE;

  if (!motionFindValue(data, objStart + 1, objEnd, "kind", valuePos)) return false;
  if (motionStringEqualsIgnoreCase(data, valuePos, objEnd, "servo")) {
    kind = MOTION_TRACK_SERVO;
  } else if (motionStringEqualsIgnoreCase(data, valuePos, objEnd, "dc")) {
    kind = MOTION_TRACK_DC;
  } else {
    return false;
  }

  long parsed = 0;
  if (!motionFindValue(data, objStart + 1, objEnd, "channel", valuePos) ||
      !motionParseInteger(data, valuePos, objEnd, parsed) ||
      parsed < 0 || parsed > 255) {
    return false;
  }
  uint8_t channel = (uint8_t)parsed;
  if (kind == MOTION_TRACK_SERVO && channel >= NUM_SERVOS) return false;
  if (kind == MOTION_TRACK_DC && channel != 0) return false;

  if (motionFindValue(data, objStart + 1, objEnd, "boardId", valuePos)) {
    // Present-but-unparseable boardId means the bake is malformed. Treat the
    // track as not-for-us rather than silently including it on every board —
    // which is what the prior `&&`-guarded path did when motionParseInteger
    // failed.
    if (!motionParseInteger(data, valuePos, objEnd, parsed) || parsed < 0 || parsed > 255) {
      return true;
    }
    uint8_t localBoard = storageBoardId();
    if (localBoard != 0 && (uint8_t)parsed != localBoard) return true;
  }

  if (out.trackCount >= MOTION_MAX_TRACKS) return false;
  if (!motionFindValue(data, objStart + 1, objEnd, "keyframes", valuePos)) return false;
  int arrayEnd = motionFindContainerEnd(data, valuePos, objEnd, '[', ']');
  if (arrayEnd < 0) return false;

  uint8_t first = out.keyframeCount;
  uint8_t count = 0;
  int pos = valuePos + 1;
  int kfStart = 0;
  int kfEnd = 0;
  uint32_t previousAt = 0;
  while (motionNextObjectInArray(data, valuePos, arrayEnd, pos, kfStart, kfEnd)) {
    if (out.keyframeCount >= MOTION_MAX_KEYFRAMES) return false;
    MotionKeyframe frame;
    if (!motionParseKeyframe(data, kfStart, kfEnd, frame)) return false;
    if (count == 0) {
      if (frame.atMs != 0) return false;
    } else if (frame.atMs <= previousAt) {
      return false;
    }
    if (frame.atMs > motionDurationMs) return false;
    out.keyframes[out.keyframeCount++] = frame;
    previousAt = frame.atMs;
    count++;
  }

  if (count < 2 || out.keyframes[out.keyframeCount - 1].atMs != motionDurationMs) {
    out.keyframeCount = first;
    return false;
  }

  MotionTrack& track = out.tracks[out.trackCount++];
  track.kind = kind;
  track.channel = channel;
  track.firstKeyframe = first;
  track.keyframeCount = count;
  track.segmentIndex = 0;
  track.lastAppliedValue = MOTION_VALUE_UNSET;
  included = true;
  return true;
}

inline bool motionLoadFromBuffer(
  const uint8_t* data,
  int len,
  const char* motionId,
  MotionRuntime& out,
  const char** error
) {
  if (error) *error = nullptr;
  memset(&out, 0, sizeof(out));
  if (data == nullptr || len < 2 || data[0] != '{') {
    if (error) *error = "bad-json";
    return false;
  }

  int valuePos = 0;
  if (!motionFindValue(data, 1, len - 1, "motions", valuePos)) {
    if (error) *error = "missing-motions";
    return false;
  }
  int motionsEnd = motionFindContainerEnd(data, valuePos, len, '[', ']');
  if (motionsEnd < 0) {
    if (error) *error = "bad-motions";
    return false;
  }

  int pos = valuePos + 1;
  int motionStart = 0;
  int motionEnd = 0;
  while (motionNextObjectInArray(data, valuePos, motionsEnd, pos, motionStart, motionEnd)) {
    int idPos = 0;
    if (!motionFindValue(data, motionStart + 1, motionEnd, "id", idPos)) continue;
    if (!motionStringEqualsIgnoreCase(data, idPos, motionEnd, motionId)) continue;

    if (!motionCopyString(data, idPos, motionEnd, out.id, sizeof(out.id))) {
      if (error) *error = "bad-motion-id";
      return false;
    }

    long duration = 0;
    if (!motionFindValue(data, motionStart + 1, motionEnd, "durationMs", valuePos) ||
        !motionParseInteger(data, valuePos, motionEnd, duration) ||
        duration < 1) {
      if (error) *error = "bad-duration";
      return false;
    }
    out.durationMs = (uint32_t)duration;

    if (!motionFindValue(data, motionStart + 1, motionEnd, "tracks", valuePos)) {
      if (error) *error = "missing-tracks";
      return false;
    }
    int tracksEnd = motionFindContainerEnd(data, valuePos, motionEnd, '[', ']');
    if (tracksEnd < 0) {
      if (error) *error = "bad-tracks";
      return false;
    }

    int trackPos = valuePos + 1;
    int trackStart = 0;
    int trackEnd = 0;
    while (motionNextObjectInArray(data, valuePos, tracksEnd, trackPos, trackStart, trackEnd)) {
      bool included = false;
      if (!motionParseTrack(data, trackStart, trackEnd, out.durationMs, out, included)) {
        if (error) *error = "bad-track";
        return false;
      }
    }

    if (out.trackCount == 0) {
      if (error) *error = "no-local-tracks";
      return false;
    }
    return true;
  }

  if (error) *error = "motion-not-found";
  return false;
}

inline void cancelMotionPlayback() {
  if (!motionRuntime.active) return;
  bool hadDcTrack = false;
  for (uint8_t i = 0; i < motionRuntime.trackCount; i++) {
    if (motionRuntime.tracks[i].kind == MOTION_TRACK_DC) {
      hadDcTrack = true;
      break;
    }
  }
  motionRuntime.active = false;
  if (hadDcTrack) setMotorSpeedQuiet(0);
}

static void motionApplyServo(uint8_t channel, int16_t value) {
  if (channel >= NUM_SERVOS || servoState[channel].stopped) return;
  uint16_t degrees = (uint16_t)constrain(value, 0, 180);
  uint16_t pulse = sequenceDegreesToPulse(channel, degrees);
  if (servoState[channel].posPulse == pulse && !servoState[channel].moving) return;
  servoState[channel].moving = false;
  servoState[channel].stopped = false;
  servoState[channel].posPulse = pulse;
  servoState[channel].targetPulse = pulse;
  pwm.setPWM(channel, 0, pulse);
}

static void motionApplyTrackValue(MotionTrack& track, int16_t value) {
  if (track.lastAppliedValue == value) return;
  track.lastAppliedValue = value;
  if (track.kind == MOTION_TRACK_SERVO) {
    motionApplyServo(track.channel, value);
  } else if (track.kind == MOTION_TRACK_DC) {
    setMotorSpeedQuiet((int8_t)constrain(value, -100, 100));
  }
}

inline void updateMotion() {
  if (!motionRuntime.active) return;

  uint32_t elapsed = (uint32_t)(millis() - motionRuntime.startMs);
  bool complete = elapsed >= motionRuntime.durationMs;
  if (complete) elapsed = motionRuntime.durationMs;

  for (uint8_t i = 0; i < motionRuntime.trackCount; i++) {
    MotionTrack& track = motionRuntime.tracks[i];
    MotionKeyframe* frames = motionRuntime.keyframes + track.firstKeyframe;

    while (track.segmentIndex + 1 < track.keyframeCount &&
           elapsed >= frames[track.segmentIndex + 1].atMs) {
      track.segmentIndex++;
    }

    int16_t value = frames[track.segmentIndex].value;
    if (track.segmentIndex + 1 < track.keyframeCount) {
      MotionKeyframe& a = frames[track.segmentIndex];
      MotionKeyframe& b = frames[track.segmentIndex + 1];
      uint32_t span = b.atMs - a.atMs;
      if (span > 0) {
        int32_t delta = (int32_t)b.value - (int32_t)a.value;
        int32_t offset = (int32_t)(elapsed - a.atMs);
        int32_t scaled = delta * offset;
        if (scaled >= 0) scaled += (int32_t)(span / 2);
        else scaled -= (int32_t)(span / 2);
        value = (int16_t)(a.value + (scaled / (int32_t)span));
      }
    }
    motionApplyTrackValue(track, value);
  }

  if (complete) {
    motionRuntime.active = false;
    Serial.print(F("Motion complete "));
    Serial.println(motionRuntime.id);
  }
}

inline bool startMotionFromStorage(const char* motionId, bool announce) {
  uint8_t* buf = storageScratchBuffer();
  int len = storageReadActive(buf, STORAGE_PAYLOAD_MAX);
  if (len <= 0) {
    Serial.println(F("No baked library"));
    return false;
  }

  // Cancel any in-flight Motion BEFORE we let motionLoadFromBuffer overwrite
  // motionRuntime. The loader memsets the runtime to zero on entry, which
  // would otherwise leave a DC track from the previous Motion physically
  // running while motionRuntime.active reads false — meaning subsequent
  // cancel attempts no-op and the motor stays at the last applied speed
  // until a manual STOP/ROTATE.
  cancelMotionPlayback();
  // Same logic applies to any running Sequence. Gated by the
  // sequenceFiringStep flag inside cancelSequencePlayback, so a step
  // that dispatches MOTION doesn't abort its own runner.
  cancelSequencePlayback();

  const char* error = nullptr;
  if (!motionLoadFromBuffer(buf, len, motionId, motionRuntime, &error)) {
    Serial.print(F("MOTION failed: "));
    Serial.println(error ? error : "unknown");
    return false;
  }

  sequenceActive = false;
  speedSeqActive = false;
  programActive = false;
  for (uint8_t i = 0; i < motionRuntime.trackCount; i++) {
    MotionTrack& track = motionRuntime.tracks[i];
    track.segmentIndex = 0;
    track.lastAppliedValue = MOTION_VALUE_UNSET;
    if (track.kind == MOTION_TRACK_SERVO && track.channel < NUM_SERVOS) {
      servoState[track.channel].moving = false;
      servoState[track.channel].stopped = false;
    }
  }

  motionRuntime.startMs = millis();
  motionRuntime.active = true;
  updateMotion();

  if (announce) {
    Serial.print(F("Playing motion "));
    Serial.print(motionRuntime.id);
    Serial.print(F(" ("));
    Serial.print(motionRuntime.trackCount);
    Serial.println(F(" tracks)"));
  }
  return true;
}
