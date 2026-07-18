# Sequence Transition Bridges — Design

**Date:** 2026-07-18
**Status:** Design agreed, not yet implemented

## Problem

When motions are chained in a sequence, the transition between two blocks is a
hard cut. Motion A ends with a servo at 50%; Motion B's first keyframe is 0%.
When the sequence fires `MOTION B`, the firmware applies B's first keyframe
*instantly* (`motionApplyServo` → `pwm.setPWM`, no slew — see
`adafruit_16_servo/motion_engine.h`). The servo snaps across the gap with no
time to travel.

Physically this slams whatever hangs off the servo. In one incident a servo was
driven from a raised position further up with no ramp, and the object hanging
from it was yanked into the motor.

The single-motion live player already solves this for a *manual* Play:
`_planMotionPreRoll` (`servo_controller.html`) keeps a running per-servo
position (`motionLastDispatched`) and glides each servo from where it is to the
motion's first keyframe with a timed `DMOVE`, stretched longer for bigger jumps.
That protection does **not** extend to sequences, and does **not** exist at all
when a board plays a baked sequence standalone (no browser).

## Goal

At every transition in a sequence, if a servo isn't already where the next block
needs it, glide it into position *before* the next block plays — instead of
snapping. Reuse must stay correct: the same motion placed after different
neighbors gets different transitions, and the motion itself is never modified.

## Chosen approach: bake the bridges in

Bridge insertion is a **bake-time transform**. It runs on a *copy* of the
sequence's `steps` array, just before serialization into the payload. The
authored sequence you saved is never mutated — bridges live only in the baked
artifact.

This was chosen over a firmware-runtime bridge and a browser-only bridge
because:

- **It protects standalone runs and live runs identically.** The firmware needs
  zero new logic — it just plays the steps it is handed. The original incident
  might have been a standalone run; only a baked-in fix is guaranteed to cover
  it.
- **Cluster lock-step is preserved for free.** Bridges are ordinary steps with
  durations; every board's sequence clock advances through them identically.
  Pausing a whole cluster mid-step in a firmware-runtime design would be much
  harder.
- **Reuse just works.** Bridges are per-transition in the baked output, computed
  from actual neighbors, not baked into the motion.

### Two layers

1. **Helpful default (nice-to-have, optional).** When creating a new motion,
   pre-fill its first keyframe from a sensible resting pose so authoring starts
   in the right place. This only reduces how often bridges are needed; because
   motions are reused, no creation-time default is ever universally correct.
   Not required for the safety net.

2. **Bridge transform (the safety net).** Described below.

## The transform

A pure function: `insertBridges(steps, motionLibrary) → steps'`. Making it pure
is deliberate — it is the whole thing that needs unit tests, and it needs no
hardware.

Walk the sequence's blocks in order, keeping a **virtual position** per servo
channel — a static simulation of where every servo will be, derived purely from
the deterministic motion definitions (each motion's first-keyframe entry pose
and last-keyframe exit pose per track).

For each `MOTION <id>` block:

1. Look up the motion's **entry pose** (first keyframe value per servo track)
   and **exit pose** (last keyframe value per servo track).
2. For each channel the block drives, compare the servo's current virtual
   position to the block's entry pose. If any differ by more than the threshold,
   **insert a bridge** before the block.
3. Advance each driven channel's virtual position to the block's exit pose.

### Bridge encoding

A step is a single command string (≤ `SEQ_CMD_MAX_LEN`, 96 chars) with a
duration and a target board. `DMOVE <ch> <pct> <ms>` moves one channel on one
board. A bridge usually moves several servos, possibly across several boards,
simultaneously. The sequencer already supports this: **zero-duration steps fire
as a chord** (same instant); a following step with a duration holds the clock.

So one bridge is:

```
DMOVE 3 50 900   target 1   durationMs 0    ┐ chord — fire together
DMOVE 5 0  900   target 1   durationMs 0    │
DMOVE 2 80 900   target 2   durationMs 0    ┘
(bridge dwell)   target all durationMs 900   ← holds clock during the glide
```

- Every `DMOVE` in a bridge carries the **same** glide duration so all servos
  arrive together.
- The trailing dwell step (empty `cmd` — firmware already skips dispatch when
  `cmd[0] == '\0'`) holds the sequence clock for that duration so the next real
  block doesn't fire early.
- Targets come from each track's `boardId`, preserving cluster lock-step.

### Threshold and duration (fully automatic)

Reuse the constants the motion editor already trusts:

- **Insert threshold:** skip if the max per-channel delta at the transition is
  `< 5%` (matches `_planMotionPreRoll`'s `maxDelta < 5` slop gate).
- **Duration:** `max(800, ceil(maxDelta × SERVO_FEASIBILITY_MS_PER_PERCENT))`
  (`SERVO_FEASIBILITY_MS_PER_PERCENT = 77`). Floor of 800ms, stretched
  proportionally by the farthest-traveling servo, at the servo's measured safe
  rate. A small nudge is quick; a full 0→100% move becomes a slow glide instead
  of a snap. No user-facing "gentleness" multiplier — automatic only.

This is also the resolution of the original incident: even when a bridge moves a
servo to a high position, it does so at the rated safe speed over a real
duration, so the mechanism can follow instead of being whipped.

## Edge cases

- **Cold start (first block).** At bake time the physical servo positions are
  unknown. Prepend a bridge that `DMOVE`s each channel the first block drives to
  its entry pose. `DMOVE` ramps from the servo's *actual* current position
  regardless of where it is, so this glides rather than snaps — **the direct fix
  for the "already up, told to go up more" incident.** Duration sized
  conservatively (assume a large delta) since it can't be measured.
- **Per-channel scope.** Only bridge channels the *incoming* block drives.
  Untouched servos are left alone.
- **DC tracks excluded.** DC is a speed, not a position — nothing to glide
  (same call `_planMotionPreRoll` makes).
- **Non-MOTION steps** (raw `STOP`, `ROTATE`, `S3`, manual commands) change
  servo positions in ways the walker can't statically model. On hitting one,
  mark the affected channels' virtual position **unknown** so the next motion
  gets a conservative (cold-start-style) bridge rather than a wrong one.
- **Loop wrap.** For a looping sequence, the last block's exit → first block's
  entry is a real transition; bridge it too, sized to the known delta.
  (Reconcile with the cold-start leading bridge during implementation so a
  looping sequence doesn't double-glide on wrap.)
- **Step budget.** Bridges cost steps against `SEQ_MAX_STEPS`. The transform
  totals them and **fails the bake with a clear message** rather than silently
  truncating.

## Visibility (not magic)

Bridges must be visible, and that visibility doubles as the "warning" originally
asked for — a fat bridge before every block means the motions don't flow, seen
before ever baking to hardware.

- **Arrange timeline:** render each inserted bridge as a distinct **dashed**
  block between the two motion blocks, labeled e.g. `↳ bridge 900ms · S3 50→0`.
  Clearly not an authored block, but visible with its cost.
- **Bake log:** one line per bridge (servos, from→to, duration); plus a summary
  (`3 bridges inserted, +2.4s`). Keeps a headless bake auditable.

## Firmware impact

None. The firmware plays the baked steps unchanged. All new logic is the
JS bake-time transform plus Arrange/bake-log rendering.

## Testing

- Unit-test `insertBridges(steps, motionLibrary)` as a pure function: bridges
  appear at the right transitions, with correct targets, durations, chord +
  dwell structure, threshold skipping, cold-start prepend, loop-wrap, non-MOTION
  reset, and the `SEQ_MAX_STEPS` budget error.
- Existing C++ sequence/`DMOVE` tests continue to cover playback, since the
  firmware is unchanged.

## Open implementation details

- Exact cold-start bridge duration policy (worst-case constant vs. a chosen
  home-travel time).
- Cold-start vs. loop-wrap reconciliation for looping sequences.
- Whether the dwell step is a separate empty-cmd step (preferred, for timeline
  visibility) or the duration folded onto the last `DMOVE`.
