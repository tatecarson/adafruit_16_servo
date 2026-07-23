# Dashboard reorganization: show control on top, build pipeline below

**Date:** 2026-07-23
**Status:** approved by Tate

## Problem

The dashboard grew section by section and its order no longer matches how
anyone uses it. The creation hierarchy is Motion → Sequence → Setlist
(smallest unit to biggest), but the page shows Bake first and Motion nearly
last. Board Telemetry burns a third of the viewport on per-channel instrument
graphics when the operator question is just "is the board on, and what is it
playing?"

## Decisions

1. **Body follows the build pipeline** top-to-bottom: Motion → Sequence →
   Setlist → Bake → OTA. Smallest thing you author comes first; deploy-ish
   sections sink.
2. **A Show Control band sits at the top** so running a show never requires
   scrolling: the setlist chooser + transport, a slim per-board status strip,
   and the master STOP/STATUS/Motor Test + free terminal.
3. **The setlist chooser and transport MOVE (not duplicate)** from the
   Setlist editor into the show bar — same DOM elements, same ids
   (`slSelect`, `slPlay`, `slPause`, `slStop`, `slLoop`, `slStatus`,
   `slLog`), so the servo-17r device-walker wiring is untouched and there is
   one source of truth for "current setlist". The Setlist editor below
   becomes pure authoring (name/mode/entries/shuffle rules/save/duplicate/
   delete/active-on-boot/Dry Run/Simulate) and always edits the setlist
   chosen at the top.
4. **Board Telemetry becomes a slim strip** — one row per board:
   `id · ip · online dot · uptime · status pills`. Offline rows show
   `signal lost · Ns` in place of uptime. The pills (existing
   `renderPills()`: RUN/MOTION/GALLERY…) are the retained "little bit of
   information" tier.
5. **Per-channel instrument graphics are deleted outright** —
   `renderInstruments()` and its channel meter renderers plus their CSS. The
   Motor Test drawer remains the place to watch/poke individual channels.
   The poll loop, `lastStatus`/`lastSeen`, pills, and gallery summary all
   survive; only presentation dies.

## New page order

```
MASTHEAD          brand · Gallery toggle · online/poll/uplink   (unchanged)
// 01 SHOW CONTROL   show bar (setlist ▾, Play/Pause/Stop/Loop, status, log)
                     board strip (slim rows)
                     master (STOP · STATUS · Motor Test · terminal)
// 02 MOTION EDITOR
// 03 SEQUENCE EDITOR
// 04 SETLIST EDITOR (authoring only)
// 05 BAKE           (content unchanged)
// 06 OTA FIRMWARE
FOOTER / drawers     (unchanged)
```

## Execution & verification

- Pure DOM reshuffle + CSS; no behavior change. All element ids stable, so
  the only JS edits are deleting the instrument renderers and slimming
  `renderBoardCard` into a row renderer.
- Full `make -C test test` must stay green (guards the marker-extracted core
  blocks didn't move).
- Browser verification: fresh page and bench library; Play guards, Dry Run,
  Motor Test drawer, all sections render, no console errors.
- Two commits: (1) section reorder + show control band, (2) board strip
  simplification.
