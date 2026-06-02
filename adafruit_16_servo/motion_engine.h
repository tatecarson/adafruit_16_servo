#pragma once

#include <string.h>
#include "bake_parse.h"
#include "servo_runtime.h"
#include "storage.h"
#include "dc_motor.h"

static bool motionParseKeyframe(
  const uint8_t* data,
  int objStart,
  int objEnd,
  MotionKeyframe& frame
) {
  int valuePos = 0;
  long parsed = 0;
  if (!bakeFindValue(data, objStart + 1, objEnd, "atMs", valuePos) ||
      !bakeParseInteger(data, valuePos, objEnd, parsed) ||
      parsed < 0) {
    return false;
  }
  frame.atMs = (uint32_t)parsed;

  if (!bakeFindValue(data, objStart + 1, objEnd, "value", valuePos) ||
      !bakeParseInteger(data, valuePos, objEnd, parsed)) {
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

  if (!bakeFindValue(data, objStart + 1, objEnd, "kind", valuePos)) return false;
  if (bakeStringEqualsIgnoreCase(data, valuePos, objEnd, "servo")) {
    kind = MOTION_TRACK_SERVO;
  } else if (bakeStringEqualsIgnoreCase(data, valuePos, objEnd, "dc")) {
    kind = MOTION_TRACK_DC;
  } else {
    return false;
  }

  long parsed = 0;
  if (!bakeFindValue(data, objStart + 1, objEnd, "channel", valuePos) ||
      !bakeParseInteger(data, valuePos, objEnd, parsed) ||
      parsed < 0 || parsed > 255) {
    return false;
  }
  uint8_t channel = (uint8_t)parsed;
  if (kind == MOTION_TRACK_SERVO && channel >= NUM_SERVOS) return false;
  if (kind == MOTION_TRACK_DC && channel != 0) return false;

  if (bakeFindValue(data, objStart + 1, objEnd, "boardId", valuePos)) {
    // Present-but-unparseable boardId means the bake is malformed. Treat the
    // track as not-for-us rather than silently including it on every board —
    // which is what the prior `&&`-guarded path did when motionParseInteger
    // failed.
    if (!bakeParseInteger(data, valuePos, objEnd, parsed) || parsed < 0 || parsed > 255) {
      return true;
    }
    uint8_t localBoard = storageBoardId();
    if (localBoard != 0 && (uint8_t)parsed != localBoard) return true;
  }

  if (out.trackCount >= MOTION_MAX_TRACKS) return false;
  if (!bakeFindValue(data, objStart + 1, objEnd, "keyframes", valuePos)) return false;
  int arrayEnd = bakeFindContainerEnd(data, valuePos, objEnd, '[', ']');
  if (arrayEnd < 0) return false;

  uint8_t first = out.keyframeCount;
  uint8_t count = 0;
  int pos = valuePos + 1;
  int kfStart = 0;
  int kfEnd = 0;
  uint32_t previousAt = 0;
  while (bakeNextObjectInArray(data, valuePos, arrayEnd, pos, kfStart, kfEnd)) {
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
  if (!bakeFindValue(data, 1, len - 1, "motions", valuePos)) {
    if (error) *error = "missing-motions";
    return false;
  }
  int motionsEnd = bakeFindContainerEnd(data, valuePos, len, '[', ']');
  if (motionsEnd < 0) {
    if (error) *error = "bad-motions";
    return false;
  }

  int pos = valuePos + 1;
  int motionStart = 0;
  int motionEnd = 0;
  while (bakeNextObjectInArray(data, valuePos, motionsEnd, pos, motionStart, motionEnd)) {
    int idPos = 0;
    if (!bakeFindValue(data, motionStart + 1, motionEnd, "id", idPos)) continue;
    if (!bakeStringEqualsIgnoreCase(data, idPos, motionEnd, motionId)) continue;

    if (!bakeCopyString(data, idPos, motionEnd, out.id, sizeof(out.id))) {
      if (error) *error = "bad-motion-id";
      return false;
    }

    long duration = 0;
    if (!bakeFindValue(data, motionStart + 1, motionEnd, "durationMs", valuePos) ||
        !bakeParseInteger(data, valuePos, motionEnd, duration) ||
        duration < 1) {
      if (error) *error = "bad-duration";
      return false;
    }
    out.durationMs = (uint32_t)duration;

    if (!bakeFindValue(data, motionStart + 1, motionEnd, "tracks", valuePos)) {
      if (error) *error = "missing-tracks";
      return false;
    }
    int tracksEnd = bakeFindContainerEnd(data, valuePos, motionEnd, '[', ']');
    if (tracksEnd < 0) {
      if (error) *error = "bad-tracks";
      return false;
    }

    int trackPos = valuePos + 1;
    int trackStart = 0;
    int trackEnd = 0;
    while (bakeNextObjectInArray(data, valuePos, tracksEnd, trackPos, trackStart, trackEnd)) {
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
  uint8_t percent = (uint8_t)constrain(value, 0, 100);
  uint16_t pulse = degreesToPulse(channel, percentToDegrees(channel, percent));
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

  // startMs may sit in the future when a Motion is armed for a cluster-shared
  // start instant (servo-vna). Compare as a signed delta so an un-started
  // Motion holds its pose instead of underflowing to a huge unsigned elapsed
  // (which would read as instantly complete). A negative delta on a fresh
  // arm = "not yet"; a positive delta on a late/rebooted board = catch-up to
  // the correct phase, keeping the cluster in lockstep.
  int32_t signedElapsed = (int32_t)(millis() - motionRuntime.startMs);
  if (signedElapsed < 0) return;

  uint32_t elapsed = (uint32_t)signedElapsed;
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

// Load and arm a Motion to begin at an explicit local time (servo-vna).
// localStartMs is in this board's millis() frame. Pass millis() for an
// immediate start; pass a future time to arm a cluster-synchronized start;
// a past time catches up to the correct phase. updateMotion() gates the
// actual playback on the signed millis()-startMs delta.
inline bool startMotionFromStorageAt(const char* motionId, unsigned long localStartMs, bool announce) {
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

  // Legacy sequence/speedSeq/program cancel removed in servo-voc.
  for (uint8_t i = 0; i < motionRuntime.trackCount; i++) {
    MotionTrack& track = motionRuntime.tracks[i];
    track.segmentIndex = 0;
    track.lastAppliedValue = MOTION_VALUE_UNSET;
    if (track.kind == MOTION_TRACK_SERVO && track.channel < NUM_SERVOS) {
      servoState[track.channel].moving = false;
      servoState[track.channel].stopped = false;
    }
  }

  motionRuntime.startMs = localStartMs;
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

// Immediate, local-only start (no cluster sync). Thin wrapper kept for the
// serial/standalone path and existing callers/tests.
inline bool startMotionFromStorage(const char* motionId, bool announce) {
  return startMotionFromStorageAt(motionId, millis(), announce);
}
