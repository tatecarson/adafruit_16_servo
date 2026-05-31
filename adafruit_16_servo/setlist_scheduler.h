#pragma once

// ============================================================
// Setlist scheduler — RUN AUTO (servo-dos, schema §4)
//
// STUB — implemented test-first. See docs/plans/2026-05-31-setlist-scheduler-design.md
// ============================================================

#include <string.h>
#include "servo_runtime.h"
#include "storage.h"
#include "sequence_engine.h"   // reuse seq* JSON parsing primitives

// Defined in command_interface.h (included after this header). The scheduler
// fires entries through dispatchCommand (NOT processCommand) so the leader's
// RUN/STOP are mirrored to follower boards over UDP — that's what keeps the
// cluster playing the same sequence in lock-step (schema §4).
void dispatchCommand(const char* cmd, bool fromNetwork);

// Parse one entry object: { "seqId":"…", "repeat":N, "gapMs":N, "weight":N }.
// Missing repeat/weight default to 1; missing gapMs defaults to 0.
static bool setlistParseEntry(const uint8_t* data, int objStart, int objEnd, SetlistEntry& out) {
  int valuePos = 0;
  if (!seqFindValue(data, objStart + 1, objEnd, "seqId", valuePos)) return false;
  if (!seqCopyString(data, valuePos, objEnd, out.seqId, sizeof(out.seqId))) return false;

  // Numeric fields are narrowed to uint16_t/uint32_t, so reject out-of-range
  // values before the cast (servo-dos review): a baked repeat/weight > 65535
  // would wrap and silently change scheduling. Missing fields keep the default.
  long parsed = 0;
  out.repeat = 1;
  if (seqFindValue(data, objStart + 1, objEnd, "repeat", valuePos)) {
    if (!seqParseInteger(data, valuePos, objEnd, parsed) || parsed < 1 || parsed > 0xFFFFL) return false;
    out.repeat = (uint16_t)parsed;
  }
  out.gapMs = 0;
  if (seqFindValue(data, objStart + 1, objEnd, "gapMs", valuePos)) {
    // `parsed` is a (≤32-bit) long, so it can't exceed UINT32_MAX; only guard < 0.
    if (!seqParseInteger(data, valuePos, objEnd, parsed) || parsed < 0) return false;
    out.gapMs = (uint32_t)parsed;
  }
  out.weight = 1;
  if (seqFindValue(data, objStart + 1, objEnd, "weight", valuePos)) {
    if (!seqParseInteger(data, valuePos, objEnd, parsed) || parsed < 1 || parsed > 0xFFFFL) return false;
    out.weight = (uint16_t)parsed;
  }
  return true;
}

// Load a Setlist by id from the bake blob into `out`. Returns false on parse
// failure or missing id (sets *error).
inline bool setlistLoadFromBuffer(const uint8_t* data, int len,
                                  const char* setlistId,
                                  SetlistRuntime& out,
                                  const char** error) {
  if (error) *error = nullptr;
  memset(&out, 0, sizeof(out));
  if (data == nullptr || len < 2 || data[0] != '{') {
    if (error) *error = "bad-json";
    return false;
  }

  int valuePos = 0;
  if (!seqFindValue(data, 1, len - 1, "setlists", valuePos)) {
    if (error) *error = "missing-setlists";
    return false;
  }
  int setlistsEnd = seqFindContainerEnd(data, valuePos, len, '[', ']');
  if (setlistsEnd < 0) {
    if (error) *error = "bad-setlists";
    return false;
  }

  int pos = valuePos + 1;
  int slStart = 0, slEnd = 0;
  while (seqNextObjectInArray(data, valuePos, setlistsEnd, pos, slStart, slEnd)) {
    int idPos = 0;
    if (!seqFindValue(data, slStart + 1, slEnd, "id", idPos)) continue;
    if (!seqStringEqualsIgnoreCase(data, idPos, slEnd, setlistId)) continue;

    if (!seqCopyString(data, idPos, slEnd, out.id, sizeof(out.id))) {
      if (error) *error = "bad-setlist-id";
      return false;
    }

    int modePos = 0;
    out.shuffle = seqFindValue(data, slStart + 1, slEnd, "mode", modePos) &&
                  seqStringEqualsIgnoreCase(data, modePos, slEnd, "shuffle");

    // shuffleRules.minGapEntries + seed (optional).
    out.minGapEntries = 0;
    out.rngState = 0;
    int rulesPos = 0;
    if (seqFindValue(data, slStart + 1, slEnd, "shuffleRules", rulesPos)) {
      int rulesEnd = seqFindContainerEnd(data, rulesPos, slEnd, '{', '}');
      if (rulesEnd > rulesPos) {
        long v = 0;
        int p = 0;
        // Reject out-of-range minGapEntries before the uint8_t cast (256 → 0).
        if (seqFindValue(data, rulesPos + 1, rulesEnd, "minGapEntries", p)) {
          if (!seqParseInteger(data, p, rulesEnd, v) || v < 0 || v > 0xFFL) {
            if (error) *error = "bad-shuffle-rules";
            return false;
          }
          out.minGapEntries = (uint8_t)v;
        }
        if (seqFindValue(data, rulesPos + 1, rulesEnd, "seed", p) &&
            seqParseInteger(data, p, rulesEnd, v)) {
          out.rngState = (uint32_t)v;
        }
      }
    }

    if (!seqFindValue(data, slStart + 1, slEnd, "entries", valuePos)) {
      if (error) *error = "missing-entries";
      return false;
    }
    int entriesEnd = seqFindContainerEnd(data, valuePos, slEnd, '[', ']');
    if (entriesEnd < 0) {
      if (error) *error = "bad-entries";
      return false;
    }

    int ePos = valuePos + 1;
    int eStart = 0, eEnd = 0;
    while (seqNextObjectInArray(data, valuePos, entriesEnd, ePos, eStart, eEnd)) {
      if (out.entryCount >= SETLIST_MAX_ENTRIES) {
        if (error) *error = "too-many-entries";
        return false;
      }
      if (!setlistParseEntry(data, eStart, eEnd, out.entries[out.entryCount])) {
        if (error) *error = "bad-entry";
        return false;
      }
      out.entryCount++;
    }
    if (out.entryCount == 0) {
      if (error) *error = "empty-setlist";
      return false;
    }
    return true;
  }

  if (error) *error = "setlist-not-found";
  return false;
}

// Parse schedulerConfig.leaderBoardId (default 1 when absent/out-of-range).
inline uint8_t schedulerLeaderBoardId(const uint8_t* data, int len) {
  if (data == nullptr || len < 2) return 1;
  int cfgPos = 0;
  if (!seqFindValue(data, 1, len - 1, "schedulerConfig", cfgPos)) return 1;
  int cfgEnd = seqFindContainerEnd(data, cfgPos, len, '{', '}');
  if (cfgEnd <= cfgPos) return 1;
  int p = 0; long v = 0;
  if (seqFindValue(data, cfgPos + 1, cfgEnd, "leaderBoardId", p) &&
      seqParseInteger(data, p, cfgEnd, v) && v >= 1 && v <= 3) {
    return (uint8_t)v;
  }
  return 1;
}

// Parse top-level activeSetlistId into `out`. Returns false if absent, null,
// or empty.
inline bool activeSetlistId(const uint8_t* data, int len, char* out, uint8_t outLen) {
  if (data == nullptr || len < 2 || outLen == 0) return false;
  int p = 0;
  if (!seqFindValue(data, 1, len - 1, "activeSetlistId", p)) return false;
  if (p >= len || data[p] != '"') return false;   // null / non-string → none
  if (!seqCopyString(data, p, len - 1, out, outLen)) return false;
  return out[0] != '\0';
}

// ---- runtime -----------------------------------------------------

// Re-entry guard: while the scheduler dispatches its own RUN/STOP, the STOP
// handler's cancelSetlistPlayback() must not tear down the scheduler issuing
// it. A user STOP (flag clear) cancels normally. Mirrors sequenceFiringStep.
static bool setlistInternalDispatch = false;

inline void cancelSetlistPlayback() {
  if (setlistInternalDispatch) return;
  setlistScheduler.active = false;
}

// Dispatch a command as the scheduler (guarded so its own STOP/RUN don't
// self-cancel). Uses dispatchCommand(…, false) so the leader's RUN/STOP mirror
// to follower boards.
static void setlistDispatch(const char* cmd) {
  setlistInternalDispatch = true;
  dispatchCommand(cmd, false);
  setlistInternalDispatch = false;
}

static uint32_t setlistRngNext() {
  uint32_t x = setlistScheduler.rngState;
  if (x == 0) x = 0x9E3779B9u;          // avoid the xorshift fixed point
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  setlistScheduler.rngState = x;
  return x;
}

static bool setlistIndexIsRecent(uint8_t idx) {
  uint8_t window = setlistScheduler.minGapEntries;
  if (window >= setlistScheduler.entryCount) window = setlistScheduler.entryCount - 1;
  uint8_t n = setlistScheduler.recentCount;
  if (n > window) n = window;
  for (uint8_t i = 0; i < n; i++) {
    // recentHead points one past the newest; walk backwards.
    uint8_t pos = (uint8_t)((setlistScheduler.recentHead + SETLIST_MAX_ENTRIES - 1 - i) % SETLIST_MAX_ENTRIES);
    if (setlistScheduler.recent[pos] == idx) return true;
  }
  return false;
}

static void setlistRecordRecent(uint8_t idx) {
  setlistScheduler.recent[setlistScheduler.recentHead] = idx;
  setlistScheduler.recentHead = (uint8_t)((setlistScheduler.recentHead + 1) % SETLIST_MAX_ENTRIES);
  if (setlistScheduler.recentCount < SETLIST_MAX_ENTRIES) setlistScheduler.recentCount++;
}

// Pick the next entry index for shuffle mode: weighted random over entries not
// in the last minGapEntries picks (relaxing to all if over-constrained).
static uint8_t setlistPickShuffle() {
  uint32_t total = 0;
  for (uint8_t i = 0; i < setlistScheduler.entryCount; i++) {
    if (!setlistIndexIsRecent(i)) total += setlistScheduler.entries[i].weight;
  }
  bool relaxed = (total == 0);
  if (relaxed) {
    for (uint8_t i = 0; i < setlistScheduler.entryCount; i++) total += setlistScheduler.entries[i].weight;
  }
  if (total == 0) return 0;
  uint32_t pick = setlistRngNext() % total;
  for (uint8_t i = 0; i < setlistScheduler.entryCount; i++) {
    if (!relaxed && setlistIndexIsRecent(i)) continue;
    uint32_t w = setlistScheduler.entries[i].weight;
    if (pick < w) return i;
    pick -= w;
  }
  return (uint8_t)(setlistScheduler.entryCount - 1);
}

// Fire entry `idx`: dispatch RUN <seqId>, reset repeat counter, enter PLAYING.
static void setlistFireEntry(uint8_t idx) {
  setlistScheduler.currentEntry = idx;
  setlistScheduler.playsDone = 0;
  setlistScheduler.phase = SETLIST_PHASE_PLAYING;
  setlistRecordRecent(idx);
  char cmd[SEQ_CMD_MAX_LEN + 1];
  snprintf(cmd, sizeof(cmd), "RUN %s", setlistScheduler.entries[idx].seqId);
  setlistDispatch(cmd);
}

static void setlistAdvance() {
  uint8_t next;
  if (setlistScheduler.shuffle) {
    next = setlistPickShuffle();
  } else {
    next = (uint8_t)((setlistScheduler.currentEntry + 1) % setlistScheduler.entryCount);
  }
  setlistFireEntry(next);
}

inline void updateSetlistScheduler() {
  if (!setlistScheduler.active) return;
  SetlistEntry& entry = setlistScheduler.entries[setlistScheduler.currentEntry];

  if (setlistScheduler.phase == SETLIST_PHASE_PLAYING) {
    if (sequenceRunner.active) return;          // sequence still playing
    // Sequence finished (or never started — bad id). Count the play.
    setlistScheduler.playsDone++;
    if (setlistScheduler.playsDone < entry.repeat) {
      char cmd[SEQ_CMD_MAX_LEN + 1];
      snprintf(cmd, sizeof(cmd), "RUN %s", entry.seqId);
      setlistDispatch(cmd);                     // re-fire same entry
      return;
    }
    setlistDispatch("STOP");                    // gap begins (mirrors to peers)
    setlistScheduler.phase = SETLIST_PHASE_GAP;
    setlistScheduler.gapStartMs = millis();
    return;
  }

  // GAP phase
  if ((uint32_t)(millis() - setlistScheduler.gapStartMs) < entry.gapMs) return;
  setlistAdvance();
}

inline bool startSetlistFromStorage(const char* setlistId) {
  uint8_t* buf = storageScratchBuffer();
  int len = storageReadActive(buf, STORAGE_PAYLOAD_MAX);
  if (len <= 0) {
    Serial.println(F("No baked library"));
    return false;
  }
  // Leader gate: only the configured leader runs the scheduler. Followers are
  // driven by the leader's mirrored RUN/STOP, so RUN AUTO is a no-op for them.
  if (storageBoardId() != schedulerLeaderBoardId(buf, len)) return false;

  cancelSetlistPlayback();

  const char* error = nullptr;
  if (!setlistLoadFromBuffer(buf, len, setlistId, setlistScheduler, &error)) {
    Serial.print(F("RUN AUTO failed: "));
    Serial.println(error ? error : "unknown");
    return false;
  }

  // Seed the shuffle RNG: shuffleRules.seed if non-zero (reproducible),
  // otherwise millis() so unseeded runs differ.
  if (setlistScheduler.shuffle && setlistScheduler.rngState == 0) {
    setlistScheduler.rngState = (uint32_t)millis() | 1u;
  }
  setlistScheduler.recentCount = 0;
  setlistScheduler.recentHead = 0;
  setlistScheduler.active = true;

  uint8_t first = setlistScheduler.shuffle ? setlistPickShuffle() : 0;
  setlistFireEntry(first);

  Serial.print(F("Running setlist "));
  Serial.println(setlistScheduler.id);
  return true;
}

inline bool startActiveSetlist() {
  uint8_t* buf = storageScratchBuffer();
  int len = storageReadActive(buf, STORAGE_PAYLOAD_MAX);
  if (len <= 0) {
    Serial.println(F("No baked library"));
    return false;
  }
  char id[SETLIST_ID_MAX_LEN + 1];
  if (!activeSetlistId(buf, len, id, sizeof(id))) {
    Serial.println(F("No active setlist"));
    return false;
  }
  return startSetlistFromStorage(id);
}
