# Motion editor: preset shape keyframes (servo-rig)

Date: 2026-06-02
Issue: servo-rig — DAW-style automation curves in the Motion editor (// 06)

## Goal

Authoring 12 tracks one keyframe at a time is slow. Let the operator paint a
region on a track and drop in a preset curve (ramp/sine/triangle/square/ease/
jitter), materialized as normal, editable keyframes. No firmware change — once
inserted they're ordinary keyframes.

## Scope (v1)

- Single track at a time (satisfies "works across all 12 tracks" individually).
- Multi-track apply ("same shape across all 3 servos / all 9") → follow-up bead.
- Sine = one cycle (multi-cycle param deferred).

## Layering note (keeps servo-rig distinct from servo-4v1 snippets)

Curves operate at the **keyframe level inside a Motion** (produce `keyframe[]`).
Snippets (servo-4v1) operate at the **step level inside a Sequence** (produce
`step[]` that reference Motions by id). The Motion is the single source of truth
for "shape of movement"; curves build Motions, snippets arrange them. A snippet
never embeds a curve.

## Section 1 — Interaction

- **Alt/Option-drag** on a `.motion-track` rubber-bands a selection rectangle
  (live positioned overlay). Plain click still adds a single keyframe; key drag
  still moves a keyframe — Alt distinguishes the shape gesture.
- Rectangle horizontal extent → time span (`t0→t1`, snapped to 100 ms grid);
  vertical extent → value range, mapped via existing `pctToMotionValue` so servo
  inversion + `spec.min/max` are respected.
- On mouseup (box ≥ ~150 ms wide, a few % tall) show a **shape-picker popover**
  anchored at the box: Ramp ↑, Ramp ↓, Sine, Triangle, Square, Ease in, Ease
  out, Jitter. Pick → generate keyframes. Esc / click-away cancels.
- A toolbar hint: "⌥-drag a track to insert a shape".

## Section 2 — Shapes & value math

Sampling: `n = clamp(round((t1−t0)/100)+1, 3, 24)`; phase `u = i/(n−1)`;
`atMs = snap(t0 + u·(t1−t0))`.

Value: `low/high` from box (track-value space); `value =
snapMotionValue(clampMotionValue(low + v·(high−low), spec))`.

| Shape | v(u) |
|---|---|
| Ramp ↑ | u |
| Ramp ↓ | 1 − u |
| Sine | 0.5 − 0.5·cos(2π·u) |
| Triangle | 1 − \|2u − 1\| |
| Square | u < 0.5 ? 0 : 1 |
| Ease in | u² |
| Ease out | 1 − (1−u)² |
| Jitter | random() per point |

Insertion replaces existing keyframes inside `[t0,t1]`, keeps those outside, then
`normalizeKeyframes`.

## Section 3 — Feasibility (servo only; DC is step, unconstrained)

Servo slew limit = `SERVO_FEASIBILITY_MS_PER_PERCENT` (77 ms/%). No red segments
may bake.

- **Amplitude auto-reduction** (ramp/sine/triangle/ease/jitter): find largest
  `s ∈ (0,1]` with every adjacent segment feasible: `s ≤ Δt / (|Δv|·77)`,
  `s_max = min(1, min over pairs)`. Scale values toward box midpoint by `s_max`.
  If `s_max < 1`, insert gentler shape + amber notice "reduced amplitude to fit".
- **Square → trapezoid on servo**: hold low, ramp to high at steepest feasible
  slope (`Δt = |Δv|·77`), hold high, ramp down. If box too narrow for ramps,
  fall back to amplitude-reduction. DC keeps clean step.
- **Backstop**: after insertion, run existing
  `constrainServoKeyframePoint`/`analyzeMotionServoFeasibility`; if anything is
  still infeasible, reject + notify rather than bake red.

## Section 4 — Code structure & testing

Pure logic in a `// === SHAPE-CORE START/END ===` block (DOM-free, no `export`):
`shapeValue`, `generateShapeKeyframes`, `fitServoAmplitude`, `squareTrapezoidServo`.

UI glue (outside the block): Alt-drag handlers on `motionGridEl`, selection-rect
overlay, shape popover → `generateShapeKeyframes` → `mutateCurrentMotion` splice
+ `normalizeKeyframes`.

Tests: `test/verify_shape_curves.mjs` extracts SHAPE-CORE, asserts shape values
at known u; point-count clamp + grid snap; steep servo sine amplitude-reduced and
result passes the 77 ms/% check; servo square → feasible trapezoid; DC square →
step; region-replace keeps out-of-box keyframes. Wired into the node test run in
`make`. Drag/popover glue: stub-DOM smoke + manual (checklist / servo-cdz).

## Follow-ups
- Multi-track apply (same shape across N tracks).
- Sine multi-cycle parameter.
- servo-e3t (automation lane: lines + per-segment curve) builds on this.
