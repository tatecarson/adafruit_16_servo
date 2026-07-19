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

### Bridge encoding — synthesized bridge motions

**Constraint that drives this:** `SEQ_MAX_STEPS` is **16**
(`adafruit_16_servo/servo_runtime.h`). A DMOVE-per-servo encoding costs one step
per moving servo plus a dwell, so a couple of multi-servo bridges exhaust the
budget. That does not scale to real sequences.

Instead, an **interior bridge is a synthesized 2-keyframe motion**, inserted as a
**single** `MOTION __bridge_<seqId>_<n>` step. Cost is always one step per
bridge, regardless of how many servos move. It reuses the firmware's existing
motion playback and cluster sync verbatim — still zero new firmware logic.

For a transition with previous exit pose `from` and next entry pose `to`, the
synthesized motion has one servo track per channel that actually moves:

```
{ id: "__bridge_<seqId>_2", name: "↳ bridge", scope: "cluster",
  durationMs: 900,
  tracks: [
    { kind:"servo", boardId:1, channel:0,
      keyframes:[ {atMs:0, value: from.b1s0}, {atMs:900, value: to.b1s0} ] },
    ... one per moving servo ...
  ] }
```

Why the first keyframe is the **from** value: firmware `motionApplyServo`
asserts keyframe-0 instantly at t=0. For an interior transition the servo is
already sitting at `from` (the previous motion held its exit pose), so that
assertion is a no-op, and the track then ramps `from → to`. Servos not in the
bridge are simply omitted, so they hold. After the bridge the servos sit exactly
at the next motion's entry pose, so the real motion's own keyframe-0 assertion is
also a no-op. Continuity end to end.

**Cold start is the exception** and uses DMOVE instead (see Edge cases): at the
very start of a sequence the servos' physical position is unknown, so no baked
keyframe-0 value is correct — only `DMOVE`, which ramps from the servo's *actual*
hardware position, is safe there.

Synthesized bridge motions are added to the bake's `motions` array, sliced
per-board like any motion (only tracks matching a board are kept), and hidden
from the motion-library UI by their `__bridge_` id prefix.

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

- **Cold start (first motion) — DMOVE, not a synth motion.** At bake time the
  servos' physical position is unknown, so no keyframe-0 value is correct and a
  synth bridge motion would snap. Prepend a bridge built from `DMOVE <ch> <pct>
  <ms>` steps (chord of zero-duration DMOVEs targeted per board + a dwell step),
  one per channel the first motion drives. `DMOVE` ramps from the servo's
  *actual* current position regardless of where it is — **the direct fix for the
  "already up, told to go up more" incident.** Duration is sized conservatively
  (`unknownDeltaPercent`, default 100%) since the delta can't be measured. This
  is the only place DMOVE is used, and it happens at most once per sequence.
- **Per-channel scope.** Only bridge channels the *incoming* motion drives.
  Because every motion carries a track for every servo (untouched ones bake as
  constant holds), in practice the incoming motion asserts all servos — so a
  bridge covers every servo whose value actually changes past the threshold.
- **DC tracks excluded.** DC is a speed, not a position — nothing to glide
  (same call `_planMotionPreRoll` makes).
- **Non-MOTION steps** (raw `STOP`, `ROTATE`, `S3`, manual commands) change
  servo positions in ways the walker can't statically model. On hitting one,
  mark virtual position **unknown** so the next motion gets a conservative
  (DMOVE cold-start-style) bridge rather than a wrong one.
- **Loop wrap — v1 limitation.** `LOOP` is a *runtime* flag (`RUN <seq> LOOP`),
  not stored in the sequence, so at bake time we cannot know a sequence will
  loop. Adding a trailing last→first bridge unconditionally would wrongly drag
  servos back toward the start pose at the end of a one-shot run. v1 therefore
  does **not** bridge the loop seam; a looped sequence still snaps once per wrap.
  Documented as a follow-up (options: a per-sequence "loops" hint in the schema,
  or a firmware loop-aware bridge). Interior + cold-start bridging is unaffected.
- **Step budget.** Interior bridges cost 1 step each; the cold-start bridge costs
  (moving servos + 1). The walker totals the produced steps and **fails the bake
  with a clear message** naming the sequence when it would exceed `SEQ_MAX_STEPS`
  (16), rather than silently truncating.

## Visibility (not magic)

Bridges must be visible, and that visibility doubles as the "warning" originally
asked for — a fat bridge before every block means the motions don't flow, seen
before ever baking to hardware.

- **Arrange timeline:** render each inserted bridge as a distinct **dashed**
  block between the two motion blocks, labeled e.g. `↳ bridge 900ms · S3 50→0`.
  Clearly not an authored block, but visible with its cost.
- **Bake log:** one line per bridge (servos, from→to, duration); plus a summary
  (`3 bridges inserted, +2.4s`). Keeps a headless bake auditable.

## Layer 1 — helpful authoring default

When a new motion is created (`defaultMotion`), seed each servo track's first
keyframe from the **exit pose of the currently-open motion** instead of a flat 0,
so authoring literally "starts where the last one ended." Reuses the same
`motionExitPose` helper the bridge transform uses. Purely a convenience that
reduces how many bridges get generated; because motions are reused, it is never a
substitute for Layer 2.

## Firmware impact

None. The firmware plays the baked steps/motions unchanged. All new logic is the
JS bake-time transform plus Arrange/bake-log rendering and the `defaultMotion`
seed.

## Testing

Everything new is a pure function extracted from a marked HTML region and tested
under Node (matching `test/verify_seq_arrangement.mjs`):

- `bridgeServoPose(motion, edge)` — entry/exit pose extraction.
- `planColdStartBridge(toPose, opts)` — DMOVE chord + dwell, threshold, duration.
- `insertSequenceBridges(steps, motions, opts)` → `{ steps, bridgeMotions }` —
  interior synth-motion bridges at the right transitions, virtual-position
  tracking, non-MOTION reset, cold-start prepend, and the `SEQ_MAX_STEPS` budget
  error.
- `seedFirstKeyframesFromExit(prevMotion, newMotion)` — Layer 1 seeding.

Existing C++ sequence / `MOTION` / `DMOVE` tests continue to cover playback,
since the firmware is unchanged.

## Open implementation details

- Exact `unknownDeltaPercent` for the cold-start DMOVE duration (default 100%).
- Loop-seam bridging (deferred — needs a bake-time signal that a sequence loops).
- Whether synth bridge motions are also surfaced (dimmed) in the sequencer table
  or only in the Arrange timeline.
