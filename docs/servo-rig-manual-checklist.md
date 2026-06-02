# Manual checklist — Motion editor shape curves (servo-rig)

No hardware needed; pure browser feature.

## Launch
```bash
python3 servo_library_server.py
```
Open the printed URL → section **// 06 Motion**. Create/select a Motion so the
12 tracks are visible. (The `library-test` motion in `library.json` works.)

> The section hint should read "… ⌥-drag a track to insert a shape …".

## A. Gesture basics
- [ ] Plain click on a track still adds a single keyframe (unchanged).
- [ ] **Alt/Option-drag** on a track draws an amber selection rectangle that
      follows the cursor.
- [ ] Releasing a reasonable-sized box opens a **shape popover** near the cursor
      with: Ramp ↑, Ramp ↓, Sine, Triangle, Square, Ease in, Ease out, Jitter.
- [ ] A tiny box (very short/!thin) shows "drag a wider box to insert a shape"
      and opens no popover.
- [ ] **Esc** or clicking elsewhere closes the popover without inserting.

## B. Shape generation (DC track — bottom of each board, -100..100)
- [ ] Alt-drag a wide box on a **DC** track, pick **Sine** → keyframes trace a
      sine across the box; values span the box's vertical range.
- [ ] **Square** on DC → clean two-level step (values only at the box's low/high).
- [ ] **Ramp ↑ / Ramp ↓** → straight rise / fall.
- [ ] **Jitter** → random points within the box.
- [ ] Generated keyframes are **normal** — you can drag/double-click-delete them.
- [ ] Re-painting an overlapping box replaces the keyframes inside it, keeps the
      rest.

## C. Servo feasibility (servo tracks — 0..100%)
- [ ] Alt-drag a **short, tall** box on a servo track, pick **Sine** → it still
      inserts, and the status shows **"reduced amplitude to fit (N%)"** in amber
      (servos are slow: full range needs ~7.7s).
- [ ] Alt-drag a **wide, short** box (gentle slope), pick **Ramp ↑** → inserts
      with **no** amplitude-reduced notice.
- [ ] **Square** on a servo, wide box → a trapezoid (ramped edges), not a flat
      line and not a vertical jump.
- [ ] Bake/feasibility: no track shows red/infeasible segments after inserting
      any shape (the editor's feasibility markers stay clean).

## D. Cross-checks
- [ ] Works on any of the 12 tracks (servo and DC, all 3 boards).
- [ ] Inserted shapes survive save and re-open (library persistence).
- [ ] No console errors during any of the above (open DevTools).

### If something's off
Note the track kind, box size, and shape. The curve math + feasibility are
unit-verified (`make -C test sim-verify`), so issues are most likely in the
drag/popover glue (rectangle geometry, popover placement) or value mapping.
