# Sequencer DC lanes — design

Date: 2026-07-19
Status: designed, not implemented

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
5. **No firmware change** in phase 1. The design is entirely browser-side.

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

Rendering reuses `motionConnectionPoints(points, duration, { kind: "dc" })`,
which already produces the horizontal-hold-then-vertical-jump polyline DC step
semantics need and is already covered by `test/verify_editor_keyframes.mjs`.

## Deletions

Dropping the `dc` entries from `MOTION_TRACK_SPECS` strips out, in
servo_controller.html:

- `analyzeMotionDcLiveFeasibility`, `analyzeDcLiveEventPair`, `_dcRotateEvents`
- `formatDcLiveFeasibilityFinding`, `dcLiveFeasibilityWarningSuffix`
- the `DC_LIVE_MIN_SPACING_MS` throttle in `sendMotionFrame`
- `dcStopBoardIds` in the pre-roll planner

Roughly 120 lines of the most intricate code in the file.

## Phasing

**Phase 1 — browser only.** No firmware change, no reflash. Existing firmware
ignores the now-absent DC tracks.

**Phase 2 — optional, separate.** Strip `MOTION_TRACK_DC` handling from
`motion_engine.h` and `servo_runtime.h` to reclaim OTA flash against
`OTA_BUDGET`. Kept as its own change so a flashing problem cannot be confused
with an authoring problem.

## Migration

Small: one motion (`library-test`) carries DC content, on boards 1 and 3.
Library load strips DC tracks from all motions and logs what it dropped —
motion name, board, keyframe count — so the envelopes can be re-authored in the
lane. Nothing else in `library.json` is touched.

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

The underlying DC-in-Motion bug is designed around, not fixed. If DC ever
returns to Motions, it returns unexplained.
