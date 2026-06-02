# Setlist 1-Hour Simulation (servo-6j4, Part 2)

Date: 2026-06-02
Issue: servo-6j4 — Browser: verification tools (dry-run + 1-hour simulation)

## Scope

servo-6j4 bundles three verification tools:

1. On-device dry-run (RUN AUTO noop flag, 60s, reports back) — **deferred** (firmware + hardware). Split to its own bead.
2. **Browser-side 1-hour shuffle simulation + timeline — THIS DESIGN.**
3. Masthead "last 10 transitions" log from device telemetry — **deferred** (telemetry). Split to its own bead.

Only Part 2 is built this session: it is pure browser, needs no hardware, and can be
proven correct against the firmware. Parts 1 and 3 become follow-up issues.

## Purpose

Let the operator vet a setlist *before* baking/leaving the gallery, catching:

- **Same sequence too soon** — a sequence replaying too close to itself.
- **Unbalanced play counts** — one sequence dominating airtime while others starve.
- **Dead air / pacing** — gaps too long/short; the rhythm of the hour feels off.

Tag/mood clustering is explicitly out of scope (avoidSameTag/moodArc not yet landed — servo-yxd).

## Section 1 — Simulation core (device-faithful engine)

Pure function `simulateSetlist(lib, setlist, { seed, horizonMs })` → event list. It ports
`adafruit_16_servo/setlist_scheduler.h` 1:1:

- **RNG**: xorshift32 `x ^= x<<13; x ^= x>>17; x ^= x<<5` (32-bit wrap), seeded with
  `shuffleRules.seed` if non-zero, else a JS-supplied stand-in for `millis()`.
- **Shuffle pick** (`setlistPickShuffle`): weighted selection over entries NOT in the last
  `minGapEntries` picks (recency ring), relaxing to ALL entries when the recency filter
  zeroes the weight total; final fallback `return entryCount - 1`; all-zero-weights → `return 0`.
  Recency is recorded once per *fired* entry (matches `setlistFireEntry` → `setlistRecordRecent`),
  NOT on per-repeat re-fires.
- **Ordered pick**: `(current + 1) % entryCount`.
- **Repeat/gap semantics** (from `updateSetlistScheduler`): an entry plays `repeat` times
  back-to-back with NO gap between repeats; the `gapMs` gap occurs only after all repeats,
  then it advances. Entry airtime = `repeat * seqDurationMs + gapMs`.
- **Sequence duration**: `Σ step.durationMs` via existing `lookupSequence(lib, seqId)`.
  Missing/deleted seqId → duration 0 (firmware counts a bad id as a play).
- Both modes run forever; the `horizonMs` cutoff (default 3,600,000) is the only bound.
- **Runaway guard**: hard cap on event count (100k). If hit (all durations + gaps = 0),
  abort and surface "sequences have no duration — check step timings."

Output: `{ seedUsed, events: [{ tStartMs, seqId, seqName, durationMs, repeat, gapAfterMs, missing }] }`.

## Section 2 — UI / output

Trigger: a **"Simulate hour"** button in the existing Setlist section, beside the shuffle-rules
panel. Runs against the *currently selected* setlist (vet before marking active/baking).
Opens an inline, collapsible results panel below the entries (not a modal).

**Region 1 — Representative timeline** (one concrete run):
- Scrollable list: `mm:ss` start · sequence name · duration · gap-after.
- Inline flags: 🔁 too-soon (replays within the minGap window / time threshold),
  🕳 long gap (gap over threshold), ⚠ missing sequence.
- **Re-roll** button shown only for unseeded setlists (new stand-in seed each click).
  Seeded setlists show a "seed N (reproducible)" badge, no re-roll.

**Region 2 — Summary stats**:
- Seeded → stats from the single exact run.
- Unseeded → aggregated over **N=100 simulated hours** (re-seeded each), so counts/pacing
  are statistically meaningful. Cheap: pure arithmetic, no per-run DOM.
- Shows: per-sequence play count (min/avg/max + bar) and % airtime; pacing (min/avg/max gap,
  longest dead-air stretch); too-soon incident count.
- One-line verdict banner: "✓ looks balanced" or "⚠ <seq> plays Nx more; M too-soon repeats."

Single-entry ordered lists are NOT flagged too-soon (intentional single-loop).

## Section 3 — Testing ("matches device behavior")

The repo's C++ test `test_shuffle_is_deterministic_for_seed` pins a known oracle: the
`gallery` setlist, **seed 42, minGapEntries 2**, 3 entries, first 6 picks deterministic.

- Extract the sim core so it runs under `node` (pure, no DOM).
- Add `test/verify_sim_matches_firmware.mjs`: feed the JS core the identical seed + a library
  matching the C++ `kBlob`, assert the first 6 picks equal the firmware's order.
- Capture the firmware's expected order from a one-shot C++ dump, embed as the oracle constant.
- Wire `make sim-verify` (in `test/Makefile`) to run the node check.

This is a runnable regression proof with no hardware and no new JS framework.

Manual UI clicking is added to **servo-cdz**'s manual-smoke scope (automated Playwright is
blocked in this environment).

## Out of scope / follow-ups

- Part 1 (on-device dry-run): new bead.
- Part 3 (masthead transition log): new bead.
- Tag-aware checks: gated on servo-yxd (avoidSameTag) + schema v2 (moodArc).
