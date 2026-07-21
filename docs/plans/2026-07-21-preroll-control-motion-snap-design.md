# Pre-roll opt-out and 5-grid value snapping

servo-cjv (pre-roll opt-out) · servo-3o6 (snap to 5) · 2026-07-21

Two changes that attack the same complaint from opposite ends. Pre-rolls are
inserted automatically and the operator cannot refuse one. Snapping keyframe
values to a coarse grid lets one Motion's exit land exactly on the next
Motion's entry, which makes the pre-roll unnecessary in the first place.

## 1. Per-step pre-roll opt-out

Pre-rolls are derived, never authored. `rewriteSequencePreRolls` plans a PREP
glide for every Motion step at display and bake time, so "delete this pre-roll"
has to be recorded as a refusal on the authored step the pre-roll precedes.

### Data model

`normalizeStep` gains an optional field:

```js
...(step?.noPrep ? { noPrep: true } : {}),
```

Absent means enabled, so every saved sequence keeps today's behavior and no
migration is needed.

`rewriteSequencePreRolls` gains one branch. A step carrying `noPrep` is pushed
untouched — no PREP command, no added duration — but **`virt` still advances to
that motion's exit pose**. Skipping the pose update would plan the *next*
motion's pre-roll from a stale position and size it wrong.

The planner still computes what the pre-roll would have been and records it:

```js
summary.push({ kind, durationMs: prepareMs, channels, stepIndex, disabled: true });
```

A disabled entry contributes nothing to the emitted steps but carries the
would-be duration, which is what lets Arrange draw a restorable ghost and
report the glide that was given up.

Bake inherits this for free — `bakeSequences` routes through the same function.
`flattenDcLanes` runs on the rewritten steps, so a DC chord that used to fire
during a pre-roll now fires at the Motion's true start. More accurate, not less.

### Arrange control

`expandPreRollStepsForDisplay` takes the summary as a second argument so it can
place ghosts for disabled entries.

- **Enabled**: today's amber PREP block, now clickable.
- **Disabled**: a zero-duration ghost at the same position, dashed border,
  dimmed to `--ink-mute`, reading `↳ pre-roll off`. `minBlockPx = 30` keeps it
  30px wide and clickable, so nothing switched off becomes unrecoverable.

Both carry `data-arrange-prep="<authoredIndex>"`. A click handler flips `noPrep`
on that step, saves, and re-renders. Disabled during playback, matching the
existing `+ After` / `Use Here` buttons.

Safety lives in the tooltip rather than a confirm dialog. `prepareMs` is
`maxDelta × msPerPercent` floored at 800, so the duration *is* the jump size:

> `Pre-roll off · would have glided 1.4s · servos jump to the start pose`

Past roughly 1.5s — about a 20% traverse — the ghost also takes an amber
warning tint, so a 60% slam does not look identical to a 5% one. The operator
asked to delete unnecessary pre-rolls; the rig is theirs.

The meta line grows a count: `3 steps · 2 pre-rolls · 1 off · …`.

## 2. Value snapping to 5

`MOTION_VALUE_SNAP = 5`, and `snapMotionValue` becomes:

```js
return shiftKey ? Math.round(value * 10) / 10
                : Math.round(value / MOTION_VALUE_SNAP) * MOTION_VALUE_SNAP;
```

Snapping applies everywhere a value is set, which needs a second helper for the
clamp paths. A feasibility clamp returns the furthest a servo can actually
reach in the time available; rounding 42.3 up to 45 hands back a move the
hardware cannot make. So `snapMotionValueToward(value, anchor, spec)` rounds to
the grid but only ever toward the anchor. Feasibility outranks tidiness.

`_clampSlew` and `fitKeyframesToNeighbors` use the toward variant. Direct edits
and `generateShapeKeyframes` use plain `snapMotionValue`, with shape curves
snapping after sampling and before the slew clamp so the clamp keeps the last
word.

The snap helpers currently sit outside every extraction marker and are
therefore untested. They move into a new `SNAP-CORE` region that both
`verify_shape_curves.mjs` and `verify_editor_keyframes.mjs` prepend to their
extracted module, since SHAPE-CORE now calls into it.

## Testing

All pure and host-runnable through `make test`:

- **verify_seq_bridges** — `noPrep` suppresses PREP and adds no duration; the
  following motion's pre-roll is still planned from the right exit pose; the
  summary reports `disabled` with the would-be ms; a ghost lands at the correct
  display index.
- **verify_editor_keyframes** — 47 → 45, 42.5 → 45, Shift → 42.5;
  `snapMotionValueToward` never rounds away from its anchor.
- **verify_shape_curves** — every generated value is a multiple of 5 and slew
  feasibility still holds after snapping.
- **Round trip** — two motions whose exit and entry both snap to 40 produce
  zero pre-rolls, since a delta of 0 falls under the 5% interior-bridge
  threshold. That is the point of the feature, so it gets an explicit test.
