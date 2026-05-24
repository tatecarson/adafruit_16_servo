#pragma once

// ============================================================
// Sequence runner — RUN <sequenceId> [LOOP]
//
// Plays a schema v1 Sequence from the active EEPROM bake by stepping
// through its `steps` array on a millis() clock. Each step has a
// `cmd` (max 96 chars per schema §3) and a `durationMs`. When the
// step fires, the cmd is dispatched through the existing processCommand
// parser so PLAY/SPLAY/ROTATE/MOTION/MOVE/Sn/STOP/TIMESCALE all work
// as step contents — free composition.
//
// Target filtering: each step has a `target` field (0/all, 1, 2, 3).
// Per-board skipping is done at runtime, not parse time — the clock
// still advances through steps targeted at other boards so the runner
// stays in lock-step across the cluster. Only the cmd dispatch is
// skipped when target != myBoardId.
//
// Numeric disambiguation: RUN <n> where <n> is a digit-prefixed token
// runs the legacy in-firmware program runner (servo_setup.h
// SequenceProgramDefinition). RUN <kebab-case> runs the schema v1
// Sequence from the active bake. Schema id regex forbids leading
// digits so there's no collision.
// ============================================================

#include <string.h>
#include "servo_runtime.h"
#include "storage.h"

// Forward declarations from command_interface.h. Included after this
// header in the .ino, so they need to be visible here for the step
// dispatch loop. processCommand is also declared in servo_runtime.h
// (no extern "C" — definitions live in C++ files).
void processCommand(char* cmd);
void cancelMotionPlayback();

// ---- bounded JSON parsing helpers --------------------------------
// Copy of the parsing primitives from motion_engine.h, prefixed with
// `seq` to avoid ODR collisions. Should eventually move into a shared
// bake_parse.h — tracked as a follow-up issue.

static bool seqAsciiEq(char a, char b) {
  if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
  if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
  return a == b;
}

static int seqSkipWs(const uint8_t* data, int pos, int end) {
  while (pos < end) {
    char c = (char)data[pos];
    if (c != ' ' && c != '\t' && c != '\n' && c != '\r') break;
    pos++;
  }
  return pos;
}

static bool seqKeyEquals(const uint8_t* data, int start, int end, const char* key) {
  int i = 0;
  for (int p = start; p < end; p++, i++) {
    if (key[i] == '\0' || (char)data[p] != key[i]) return false;
  }
  return key[i] == '\0';
}

static bool seqStringEqualsIgnoreCase(const uint8_t* data, int pos, int end, const char* value) {
  if (pos >= end || data[pos] != '"') return false;
  int p = pos + 1;
  int i = 0;
  while (p < end && data[p] != '"') {
    if (data[p] == '\\') return false;
    if (value[i] == '\0' || !seqAsciiEq((char)data[p], value[i])) return false;
    p++;
    i++;
  }
  return p < end && data[p] == '"' && value[i] == '\0';
}

static bool seqCopyString(const uint8_t* data, int pos, int end, char* out, uint8_t outLen) {
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

static bool seqFindValue(const uint8_t* data, int start, int end, const char* key, int& valuePos) {
  int curly = 0;
  int square = 0;
  bool inString = false;
  bool escape = false;
  int stringStart = -1;
  bool keyMatched = false;
  for (int p = start; p < end; p++) {
    char c = (char)data[p];
    if (escape) { escape = false; continue; }
    if (inString) {
      if (c == '\\') escape = true;
      else if (c == '"') {
        inString = false;
        if (curly == 0 && square == 0 && stringStart >= 0) {
          keyMatched = seqKeyEquals(data, stringStart, p, key);
        }
      }
      continue;
    }
    if (c == '"') { inString = true; stringStart = p + 1; }
    else if (c == '{') { curly++; keyMatched = false; }
    else if (c == '}') { curly--; keyMatched = false; }
    else if (c == '[') { square++; keyMatched = false; }
    else if (c == ']') { square--; keyMatched = false; }
    else if (c == ':' && curly == 0 && square == 0 && keyMatched) {
      valuePos = seqSkipWs(data, p + 1, end);
      return valuePos < end;
    } else if (c == ',') { keyMatched = false; }
  }
  return false;
}

static int seqFindContainerEnd(const uint8_t* data, int start, int end, char openChar, char closeChar) {
  if (start >= end || data[start] != openChar) return -1;
  int depth = 0;
  bool inString = false;
  bool escape = false;
  for (int p = start; p < end; p++) {
    char c = (char)data[p];
    if (escape) { escape = false; continue; }
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

static bool seqParseInteger(const uint8_t* data, int pos, int end, long& value) {
  pos = seqSkipWs(data, pos, end);
  bool neg = false;
  if (pos < end && data[pos] == '-') { neg = true; pos++; }
  if (pos >= end || data[pos] < '0' || data[pos] > '9') return false;
  long v = 0;
  while (pos < end && data[pos] >= '0' && data[pos] <= '9') {
    v = (v * 10) + (data[pos] - '0'); pos++;
  }
  if (pos < end && data[pos] == '.') {
    pos++;
    while (pos < end && data[pos] >= '0' && data[pos] <= '9') pos++;
  }
  value = neg ? -v : v;
  return true;
}

static bool seqNextObjectInArray(const uint8_t* data, int arrayStart, int arrayEnd,
                                  int& pos, int& objStart, int& objEnd) {
  if (pos <= arrayStart) pos = arrayStart + 1;
  pos = seqSkipWs(data, pos, arrayEnd);
  if (pos < arrayEnd && data[pos] == ',') pos = seqSkipWs(data, pos + 1, arrayEnd);
  if (pos >= arrayEnd || data[pos] == ']') return false;
  if (data[pos] != '{') return false;
  objStart = pos;
  objEnd = seqFindContainerEnd(data, objStart, arrayEnd + 1, '{', '}');
  if (objEnd < 0) return false;
  pos = objEnd + 1;
  return true;
}

// ---- step parsing ------------------------------------------------

// Parse one step object: { "cmd":"…", "durationMs":N, "target":"all"|N }.
// `target` is normalized to 0 (all) or 1/2/3. Missing target = 0.
static bool seqParseStep(const uint8_t* data, int objStart, int objEnd, SequenceStep& out) {
  int valuePos = 0;
  if (!seqFindValue(data, objStart + 1, objEnd, "cmd", valuePos)) return false;
  if (!seqCopyString(data, valuePos, objEnd, out.cmd, sizeof(out.cmd))) return false;

  long parsed = 0;
  if (!seqFindValue(data, objStart + 1, objEnd, "durationMs", valuePos) ||
      !seqParseInteger(data, valuePos, objEnd, parsed) ||
      parsed < 0) {
    return false;
  }
  out.durationMs = (uint32_t)parsed;

  out.target = 0;
  if (seqFindValue(data, objStart + 1, objEnd, "target", valuePos)) {
    if (seqStringEqualsIgnoreCase(data, valuePos, objEnd, "all")) {
      out.target = 0;
    } else if (seqParseInteger(data, valuePos, objEnd, parsed) &&
               parsed >= 1 && parsed <= 3) {
      out.target = (uint8_t)parsed;
    } else {
      // Present-but-malformed target: same defensive choice as
      // motion_engine.h's boardId — exclude (target = an impossible
      // board id so no board ever fires this step).
      out.target = 255;
    }
  }
  return true;
}

// ---- sequence loader ---------------------------------------------

// Find the named Sequence inside the bake blob and populate `out`.
// Returns false (and sets *error) on parse failure or missing id.
inline bool sequenceLoadFromBuffer(const uint8_t* data, int len,
                                    const char* sequenceId,
                                    SequenceRuntime& out,
                                    const char** error) {
  if (error) *error = nullptr;
  memset(&out, 0, sizeof(out));
  if (data == nullptr || len < 2 || data[0] != '{') {
    if (error) *error = "bad-json";
    return false;
  }

  int valuePos = 0;
  if (!seqFindValue(data, 1, len - 1, "sequences", valuePos)) {
    if (error) *error = "missing-sequences";
    return false;
  }
  int sequencesEnd = seqFindContainerEnd(data, valuePos, len, '[', ']');
  if (sequencesEnd < 0) {
    if (error) *error = "bad-sequences";
    return false;
  }

  int pos = valuePos + 1;
  int seqStart = 0;
  int seqEnd = 0;
  while (seqNextObjectInArray(data, valuePos, sequencesEnd, pos, seqStart, seqEnd)) {
    int idPos = 0;
    if (!seqFindValue(data, seqStart + 1, seqEnd, "id", idPos)) continue;
    if (!seqStringEqualsIgnoreCase(data, idPos, seqEnd, sequenceId)) continue;

    if (!seqCopyString(data, idPos, seqEnd, out.id, sizeof(out.id))) {
      if (error) *error = "bad-sequence-id";
      return false;
    }

    if (!seqFindValue(data, seqStart + 1, seqEnd, "steps", valuePos)) {
      if (error) *error = "missing-steps";
      return false;
    }
    int stepsEnd = seqFindContainerEnd(data, valuePos, seqEnd, '[', ']');
    if (stepsEnd < 0) {
      if (error) *error = "bad-steps";
      return false;
    }

    int stepPos = valuePos + 1;
    int stepStart = 0, stepEnd = 0;
    while (seqNextObjectInArray(data, valuePos, stepsEnd, stepPos, stepStart, stepEnd)) {
      if (out.stepCount >= SEQ_MAX_STEPS) {
        if (error) *error = "too-many-steps";
        return false;
      }
      if (!seqParseStep(data, stepStart, stepEnd, out.steps[out.stepCount])) {
        if (error) *error = "bad-step";
        return false;
      }
      out.stepCount++;
    }

    if (out.stepCount == 0) {
      if (error) *error = "empty-sequence";
      return false;
    }
    return true;
  }

  if (error) *error = "sequence-not-found";
  return false;
}

// ---- runtime -----------------------------------------------------

// Re-entry guard. When a sequence step dispatches a cmd (PLAY/SPLAY/
// MOTION/Sn/STOP/…) the cancellation hooks inside those commands
// would otherwise cancel the sequence dispatching them — every step
// would abort its own runner. While this flag is set, sequence
// cancellation no-ops; the dispatched cmd still cancels MOTION state,
// servo motion, etc., but the sequence runner keeps walking.
static bool sequenceFiringStep = false;

inline void cancelSequencePlayback() {
  if (sequenceFiringStep) return;
  if (!sequenceRunner.active) return;
  sequenceRunner.active = false;
  // Note: we don't cancel the in-flight inner cmd here (e.g. a MOTION
  // step that's still playing). Callers that fully halt the board
  // (STOP, stopActivePatterns) also call cancelMotionPlayback and
  // the other cancellation hooks. The sequence runner stops scheduling
  // new steps; downstream actuator engines do their own cleanup.
}

// Fire the current step's cmd if it targets this board (or all
// boards). Step duration runs regardless so cluster timing stays
// aligned even when most steps belong to other boards.
inline void sequenceFireCurrentStep() {
  SequenceStep& step = sequenceRunner.steps[sequenceRunner.currentStep];
  uint8_t myBoard = storageBoardId();
  bool forMe = (step.target == 0) || (myBoard != 0 && step.target == myBoard);
  if (forMe && step.cmd[0] != '\0') {
    // processCommand mutates the buffer in place (trim, uppercase),
    // so copy into a writable scratch first.
    char scratch[SEQ_CMD_MAX_LEN + 1];
    strncpy(scratch, step.cmd, sizeof(scratch));
    scratch[sizeof(scratch) - 1] = '\0';
    sequenceFiringStep = true;
    processCommand(scratch);
    sequenceFiringStep = false;
  }
}

inline void updateSequenceRunner() {
  if (!sequenceRunner.active) return;
  uint32_t elapsed = (uint32_t)(millis() - sequenceRunner.stepStartMs);
  // Pointer (not reference!) so re-assignment actually rebinds. With a
  // reference, `step = sequenceRunner.steps[i]` copies-into the
  // currently-referenced slot — silently corrupting steps[0] with
  // steps[1]'s data on the first iteration. Took a debug session to
  // catch; do not "simplify" this back to a reference.
  SequenceStep* step = &sequenceRunner.steps[sequenceRunner.currentStep];
  if (elapsed < step->durationMs) return;

  // Advance. We consume the elapsed budget step-by-step rather than
  // resetting to zero each iteration — that way a slow tick (or a
  // chord of zero-duration steps, schema §3) chains forward without
  // losing time. stepStartMs accumulates the previous step's
  // durationMs so the next step's "elapsed" math stays anchored to
  // the original sequence-start moment.
  while (sequenceRunner.active && elapsed >= step->durationMs) {
    uint32_t consumedMs = step->durationMs;
    sequenceRunner.currentStep++;
    if (sequenceRunner.currentStep >= sequenceRunner.stepCount) {
      if (sequenceRunner.loop) {
        sequenceRunner.currentStep = 0;
      } else {
        sequenceRunner.active = false;
        Serial.print(F("Sequence complete "));
        Serial.println(sequenceRunner.id);
        return;
      }
    }
    sequenceRunner.stepStartMs += consumedMs;
    elapsed -= consumedMs;
    sequenceFireCurrentStep();
    step = &sequenceRunner.steps[sequenceRunner.currentStep];
  }
}

inline bool startSequenceFromStorage(const char* sequenceId, bool loop, bool announce) {
  uint8_t* buf = storageScratchBuffer();
  int len = storageReadActive(buf, STORAGE_PAYLOAD_MAX);
  if (len <= 0) {
    Serial.println(F("No baked library"));
    return false;
  }

  // Cancel any in-flight sequence + motion BEFORE the loader memsets
  // sequenceRunner. Same defensive pattern as startMotionFromStorage.
  cancelSequencePlayback();

  const char* error = nullptr;
  if (!sequenceLoadFromBuffer(buf, len, sequenceId, sequenceRunner, &error)) {
    Serial.print(F("RUN failed: "));
    Serial.println(error ? error : "unknown");
    return false;
  }

  sequenceRunner.loop = loop;
  sequenceRunner.currentStep = 0;
  sequenceRunner.stepStartMs = millis();
  sequenceRunner.active = true;
  sequenceFireCurrentStep();

  if (announce) {
    Serial.print(F("Running sequence "));
    Serial.print(sequenceRunner.id);
    Serial.print(F(" ("));
    Serial.print(sequenceRunner.stepCount);
    Serial.print(F(" steps"));
    if (loop) Serial.print(F(", LOOP"));
    Serial.println(F(")"));
  }
  return true;
}
