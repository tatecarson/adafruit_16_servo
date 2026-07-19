# Sequencer DC lanes — design

Date: 2026-07-19
Status: implemented (servo-i0v)

## Problem

DC motor speed is authored as keyframe tracks inside a Motion, alongside servo
tracks. Two things are wrong with that.

**It does not work.** A Motion whose DC track is dense and non-zero from the
first 400ms does not turn the motor on hardware, while `ROTATE 50` typed as a
sequencer step turns it fine. Host simulation of both the bake path and the
browser live path shows DC commands being issued with correct values, so the
fault is somewhere between those commands and the motor. It was not found.

**It is the wrong place.** A `ROTATE` step occupies the sequence timeline
serially, so the motor cannot run while servos move. There is no way to author
a motor change parallel to servo motion, and no way to see motor state across a
sequence.

This design moves DC authoring out of Motions and into the Sequencer as
parallel lanes. That solves the second problem directly and makes the first
moot: the new path emits plain `ROTATE`, the command already known to work.

## Decisions

1. **DC leaves Motions entirely.** Motions become servo-only. The motor has one
   authoring home. No coexistence, no runtime conflict between two writers.
2. **DC is a per-step property**, not an independent timeline. Each step
   carries an optional motor speed per board; the baker emits a `ROTATE` chord
   step only when a value changes.
3. **Three lanes, one per board.** B1/B2/B3 each hold their own value, matching
   how existing motions drive the motors differently per board.
4. **The ±50 cap stays**, enforced in the lane editor. Firmware `ROTATE`
   remains permissive at ±100 for manual testing.
5. ~~**No firmware change** in phase 1.~~ **Wrong — corrected during implementation.**
   Two firmware lines had to change; see [Firmware corrections](#firmware-corrections).

## Authoring model

One optional field per step, in `library.json` only:

```json
{ "cmd": "MOTION library-test", "durationMs": 8000, "target": "all",
  "dc": { "1": -50, "3": 25 } }
```

Keys are board ids, values signed speeds clamped to ±50. A board absent from the
map is **unchanged** — the motor holds its last commanded speed. Sticky is the
only sane model: a motor has a running state, not a pose.

`normalizeStep` (servo_controller.html) gains `dc` defaulting to `{}`, so every
existing sequence stays valid without migration.

The device schema is untouched. `compactStepForDevice` already emits only
`cmd` / `durationMs` / `target`, so the `dc` field is stripped at bake by
construction. Nothing about the EEPROM payload format changes.

## Flattening

One pure function, `flattenDcLanes(steps)`, walks the steps carrying a
per-board current speed and emits a targeted zero-duration chord before each
step whose value changed:

```
ROTATE -50            duration 0     target 1
ROTATE 25             duration 0     target 3
MOTION library-test   duration 8000  target all
```

Three rules follow from existing firmware behavior:

- A `STOP` step zeroes every motor, so flattening resets tracked state to 0
  there. Otherwise a later "unchanged" would be a lie.
- End of a non-looping sequence emits `ROTATE 0` for any board still running.
- A loop wrap is an ordinary transition; carried state emits correctly.

**The same function must drive the bake and the browser live player.** Divergence
between those two paths is the exact failure mode that produced this design.

## Ordering against pre-rolls

`rewriteSequencePreRolls` turns `MOTION x` into `MOTION x PREP 7700` and extends
the step's duration, so the motion's real start is 7.7s after the step begins.
Flattening runs **after** pre-roll rewriting.

That forces a choice. A DC chord emitted before a `PREP` step starts the motor
during the servo glide, well before the motion it belongs to. Firing it at the
true motion start instead would cost two extra steps per transition — a split
step, the chord, and a dwell — which is expensive against a 16-step budget.

**Decision:** DC changes take effect at the step boundary, pre-roll included.
The Arrange lane draws across the pre-roll block, so the motor running during
the glide is visible while authoring rather than discovered on the rig.

## UI

**Step table** gains one cell per row: three narrow number inputs, B1/B2/B3.
Blank means unchanged, `0` means stop. That distinction carries the whole
model, so blanks render as `–` with an explanatory tooltip.

**Arrange dialog** gains three read-only lanes below the step lane, sharing its
x-axis and zoom, rendering the *flattened* result so the view matches the bake.

Rendering does **not** reuse `motionConnectionPoints` as planned. That maps a
time axis to percentages, but the Arrange track's block widths are floored at
`minBlockPx`, so pixel position is not proportional to time and a time-based
polyline would drift out of alignment with the blocks above it. `renderDcLanes`
instead emits one bar per display block, reusing the block geometry directly —
exact alignment, and a step function by construction, which is what a held
motor speed is anyway.

## Deletions

Dropping the `dc` entries from `MOTION_TRACK_SPECS` strips out, in
servo_controller.html:

- `analyzeMotionDcLiveFeasibility`, `analyzeDcLiveEventPair`, `_dcRotateEvents`
- `formatDcLiveFeasibilityFinding`, `dcLiveFeasibilityWarningSuffix`
- the `DC_LIVE_MIN_SPACING_MS` throttle in `sendMotionFrame`
- `dcStopBoardIds` in the pre-roll planner

Roughly 120 lines of the most intricate code in the file.

## Firmware corrections

The design claimed phase 1 was browser-only. Implementation disproved that
twice, both times because a guard written for DC-inside-Motions became actively
harmful once DC moved out.

**1. `startMotionPreparedFromStorage` zeroed the motor unconditionally.** The
pre-roll called `setMotorSpeedQuiet(0)` to stop a completed DC track running on
through a servo glide. DC chords now land immediately before that command, so
the zero wipes them and the motor stays dead for the whole Motion. Removed.

This is a hazard *created by* this design, not the original bug. Verified by
running the pre-change firmware against a post-change bake: the chord fires,
the pre-roll zeroes it, and the motor never moves again for the full 15.7s.
Under the old arrangement the same zero was harmless — it silenced the motor
only for the pre-roll, after which the Motion's own DC track took over
normally (`t=7711 motor 0 -> -1`, ramping to -50). **This is why boards must be
reflashed before DC lanes work.**

**2. `ROTATE` cancelled Motion playback.** Sensible when both drove the motor
and had to arbitrate. Now they touch disjoint hardware, so a lane chord landing
mid-Sequence would abort the very Motion it was authored to run alongside.
`ROTATE` still cancels Sequences (a manual override should take over the show);
the `sequenceFiringStep` guard keeps a chord from aborting its own runner.

The pre-roll fix is pinned by `test/test_motion_engine.cpp`. The `ROTATE`
fix is pinned by source assertions in `test/verify_dc_lanes.mjs`, since the
C++ host suite does not link `command_interface.h`.

## A pre-existing bug found on the way

`compactStepForDevice` baked `target` as whatever the `<select>` produced — the
string `"2"`. Firmware parses it with `bakeParseInteger`, which fails on the
quote and falls back to 255, "addressed to no board". **Every board-targeted
step in every baked Sequence has been silently dead.** Schema §3 always
specified a bare integer. Now coerced with `Number(target)` and pinned by a
test. Unrelated to DC lanes, but it blocked them: the first end-to-end run
showed chords reaching EEPROM and never firing.

## Phasing

**Phase 1 — browser, plus the two firmware corrections above.** Boards need a
reflash for lanes to work; without it, chords are wiped by the pre-roll.

**Phase 2 — optional, separate.** Strip `MOTION_TRACK_DC` handling from
`motion_engine.h` and `servo_runtime.h` to reclaim OTA flash against
`OTA_BUDGET`. Kept as its own change so a flashing problem cannot be confused
with an authoring problem.

## Migration

`normalizeMotion` rebuilds tracks from the now servo-only `MOTION_TRACK_SPECS`,
so legacy DC tracks stop surviving a save on their own. Dropping an authored
envelope silently would be rude, so `reportLegacyDcTracks` names each one once
per page load (console plus the library status line).

`library.json` in this repo was migrated in place: the three inert DC tracks
were removed from `library-test`, and `calm-drift` was seeded with
`dc: { "1": -40, "3": 25 }` so there is something to verify on hardware
immediately. That seed is a placeholder, not a reconstruction — the original
25-keyframe envelope has no equivalent in a per-step model and was not
translated.

## Validation

Feed the flattened step count to the existing `seqArrangeWarnEl` budget warning,
and add a sequence-level check to `analyzeBakeConformance` so an over-budget
sequence blocks the bake and names itself.

## Testing

New `test/verify_dc_lanes.mjs`, extracting `flattenDcLanes` the way the other
verifiers extract source blocks:

- sticky carry across steps
- `STOP` resets tracked state to 0
- end of non-looping sequence emits `ROTATE 0` per running board
- loop wrap emits correctly
- per-board targeting on the emitted chords
- budget overflow is reported, not silently truncated
- source assertion that the live player and baker call the same
  `flattenDcLanes`

Existing `verify_bake_payload.mjs` and `verify_seq_bridges.mjs` need updating
for DC removal. The DC assertions in `test/test_motion_engine.cpp` stay valid
through phase 1 and are removed with phase 2.

## Open

The original DC-in-Motion bug was never root-caused, and nothing found during
implementation explains it. The pre-roll zeroing was briefly assumed to be the
cause; host simulation of the pre-change firmware against a pre-change bake
disproves that — the motor was commanded correctly through the whole Motion.
Both the bake path and the browser live path issue correct DC values, and the
firmware applies them. The fault lies somewhere past `setMotorSpeedQuiet()`:
`analogWrite` on pins 10/11, the driver board, or wiring. If DC ever returns to
Motions, it returns unexplained.

## Verified

- `test/verify_dc_lanes.mjs` — 31 checks on the flattener, its consumers, and
  the firmware invariants the lanes depend on
- Full host suite green (`make -C test test`)
- End-to-end: real `library.json` → browser bake code → firmware
  `sequence_engine.h` + `motion_engine.h` compiled on host. Board 1 holds −40
  and board 3 holds +25 across the pre-roll and the whole Motion; board 2,
  which has no lane values, stays at 0.
- Browser: DOM round-trip through the lane inputs, `target` baking as a number,
  and the Arrange lanes rendering across the pre-roll block.
