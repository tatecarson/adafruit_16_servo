# Setlist Scheduler (RUN AUTO) — Design

**Issue:** servo-dos · **Schema:** docs/sequencer-schema.md §4 · **Date:** 2026-05-31

## Goal

Add `RUN AUTO` (and `RUN <setlistId>`) firmware commands that run the device's
active Setlist forever until `STOP`: pick the next entry per mode (ordered or
weighted shuffle), play its sequence `repeat` times, dwell `gapMs`, repeat.

## Scope (decided)

In scope:
- `ordered` mode: play entries in array order, looping.
- `shuffle` mode: weighted random pick honoring `minGapEntries` (don't repeat a
  seqId within N picks) and per-entry `weight`. Seeded by `shuffleRules.seed`
  (reproducible) or `millis()`.
- per-entry `repeat` and `gapMs` (leader broadcasts `STOP` during the gap).
- Leader-gated: only the board whose `boardId == schedulerConfig.leaderBoardId`
  (default 1) runs the scheduler. Followers ride the existing command mirror.

Deferred (filed as follow-ups):
- `avoidSameTag` — needs each entry's *sequence* tags loaded into RAM; costly on
  the UNO R4. Schema-legal to omit (a rule, not required behavior).
- `moodArc` / `tagEnergy` — schema marks these "effective in schemaVersion: 2"
  (§4, §7). Treated as `random` now.

## Key insight: ride the existing command mirror

`command_interface.h` already mirrors `RUN <id>` and `STOP` to peers over UDP
(`shouldMirrorCommand`, Sync.cpp). So the scheduler needs no new networking:

- The leader fires an entry with `dispatchCommand("RUN <seqId>", false)` — this
  runs the sequence locally **and** mirrors `RUN <seqId>` to followers, so the
  cluster plays in lock-step. (Note: it must be `dispatchCommand`, not
  `processCommand` — only `dispatchCommand` performs the UDP mirror.)
- Between entries / on stop, the leader dispatches `STOP` the same way, which
  likewise mirrors.

Tighter Motion-level timing sync remains servo-vna's job; servo-dos does not
depend on it.

## State (servo_runtime.h)

```c
#define SETLIST_ID_MAX_LEN 32
#define SETLIST_MAX_ENTRIES 12

struct SetlistEntry {
  char     seqId[SEQ_ID_MAX_LEN + 1];
  uint16_t repeat;   // >= 1
  uint32_t gapMs;
  uint16_t weight;   // >= 1 (shuffle only; ignored in ordered)
};

struct SetlistRuntime {
  char     id[SETLIST_ID_MAX_LEN + 1];
  uint8_t  entryCount;
  SetlistEntry entries[SETLIST_MAX_ENTRIES];
  bool     shuffle;          // mode == "shuffle"
  uint8_t  minGapEntries;
  uint32_t rngState;         // xorshift32
  bool     active;
  uint8_t  currentEntry;
  uint16_t playsDone;        // repeats completed for currentEntry
  uint8_t  phase;            // SETLIST_PHASE_PLAYING | SETLIST_PHASE_GAP
  unsigned long gapStartMs;
  uint8_t  recent[SETLIST_MAX_ENTRIES];  // ring buffer of recent picks
  uint8_t  recentCount;
  uint8_t  recentHead;
};
```

## setlist_scheduler.h (new; included after sequence_engine.h, reuses seq* JSON helpers)

- `setlistLoadFromBuffer(data, len, setlistId, out, &err)` — find `setlists[]`,
  match `id`, parse `mode`, `entries[]` (seqId/repeat/gapMs/weight), and
  `shuffleRules.minGapEntries` + `seed`. Defaults: repeat→1, weight→1, gapMs→0.
- `schedulerLeaderBoardId(data, len)` — parse `schedulerConfig.leaderBoardId`
  (default 1).
- `activeSetlistId(data, len, out)` — parse top-level `activeSetlistId`.
- `startSetlistFromStorage(setlistId)` / `startActiveSetlist()` — leader gate,
  load, fire first entry.
- `updateSetlistScheduler()` — state machine (call in loop()).
- `cancelSetlistPlayback()` — STOP halts it (re-entry guarded).

## Scheduler state machine (leader only)

`PLAYING`: a sequence is in flight. Detect completion by watching
`sequenceRunner.active` fall to false (the leader runs the seq locally). On
completion: `playsDone++`. If `playsDone < entry.repeat`, re-fire the same
seqId. Else broadcast `STOP`, set `phase=GAP`, `gapStartMs=millis()`.

`GAP`: when `millis() - gapStartMs >= entry.gapMs`, pick the next entry, reset
`playsDone=0`, record it in `recent`, fire `RUN <seqId>`, `phase=PLAYING`.

Re-entry guard: a module-static `setlistInternalDispatch` flag wraps the
scheduler's own `dispatchCommand` calls so the `STOP`/`RUN` it issues doesn't
cancel the scheduler. A *user* `STOP` (flag clear) cancels normally.

Bad seqId: if firing leaves `sequenceRunner.active == false`, count the play as
done and advance (gapMs provides dwell so we never hot-spin).

## Picking (shuffle)

xorshift32 RNG. Candidate set = entries whose index is not in the last
`min(minGapEntries, entryCount-1)` picks. Weighted choice by `entry.weight`. If
the candidate set is empty (over-constrained), relax to all entries.

## Integration

- `command_interface.h` RUN handler: `RUN AUTO` → `startActiveSetlist()`;
  `RUN <setlistId>` only if the token matches a setlist (try sequence first,
  then setlist) — or simpler: `RUN AUTO` keyword + the existing `RUN <id>`
  stays sequence-only; setlist-by-id via `RUN AUTO` reading activeSetlistId.
  (Start with `RUN AUTO`; `RUN <setlistId>` is a thin add.)
- `STOP` handler also calls `cancelSetlistPlayback()`.
- `loop()` calls `updateSetlistScheduler()` alongside `updateSequenceRunner()`.

## Tests (test/test_setlist_scheduler.cpp, mirrors test_sequence_engine.cpp)

Stub `dispatchCommand` to record dispatched cmds and simulate the sequence
runner (set `sequenceRunner.active=true` on `RUN <id>`; test toggles it false to
simulate completion). Cases:

1. parse ordered setlist (entries, repeat, gapMs defaults).
2. ordered playback fires entries in order, loops.
3. per-entry repeat fires the same seqId N times before advancing.
4. gapMs: `STOP` broadcast then dwell before next entry.
5. shuffle with seed is deterministic; honors minGapEntries; weight respected.
6. leader gate: non-leader `RUN AUTO` no-ops.
7. user `STOP` cancels; scheduler's internal `STOP` does not self-cancel.
8. bad/missing seqId advances without hot-spinning.

Quality gates: `make -C test` (all suites) + `make -C test size` (OTA budget).
Firmware → manual HW test + OTA flash before close.
