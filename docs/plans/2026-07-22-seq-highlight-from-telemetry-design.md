# Sequencer highlight from telemetry

2026-07-22

## Problem

Clicking Play in the sequencer highlights the current step in yellow. Playing
a **setlist** (or gallery mode) does not — the same highlight stays dark, so
during a show it is hard to tell where you are.

The highlight is wired only to the browser's `seqPlayer`, the JS playback loop
that streams a single sequence step-by-step. A setlist runs the other way:
`slPlay` dispatches `RUN <seq>` commands the firmware plays on-device, and
`seqPlayer` sits idle the whole time, so nothing drives the highlight.

The data to fix it is already flowing. Every ~800ms poll stores each board's
parsed `/status.json` in `lastStatus`, and that payload already carries:

```json
"runseq": { "active": true, "id": "sq-warm01", "step": 3, "steps": 6,
            "loop": false, "stepMs": 1200 }
```

Nothing reads `runseq` yet. The whole feature is wiring that field to the
highlight the browser already knows how to draw.

## Section 1 — Data source and step mapping

**Source.** `readRunseqTelemetry(lastStatus, schedulerConfig, now)` picks the
authoritative board: the leader (`schedulerConfig.leaderBoardId`) if it reports
`runseq.active`, else the first active board. Returns
`{ id, firmwareStep, firmwareStepCount, loop, stepMs }`, or `null` when nothing
runs or the chosen board's last poll is stale (older than ~2.5s) — a
disconnected board must not freeze a stale highlight.

**Mapping firmware step to authored row.** `runseq.step` counts the flattened
steps we bake — DC chords, PREP glides, dwells. The browser reproduces that
exact layout (`rewriteSequencePreRolls` then `flattenDcLanes` on the running
sequence) and reads `authoredIndex` at that flattened index — the same
`authoredIndex` plumbing `seqPlayer` uses, so the telemetry highlight lands on
exactly the row browser-driven playback would.

**Trust guard.** Before mapping, compare the browser's flattened step count with
`runseq.steps`. A mismatch means the library was edited since the bake actually
on the device, so the per-step mapping would lie — fall back to sequence-level
highlight only (title glow, no row). A match maps to the authored step.

Pure function: `mapFirmwareStepToAuthored(seq, library, firmwareStep,
firmwareStepCount)` → `{ authoredIndex | null, reliable }`, no DOM.

## Section 2 — Highlight-source decision

Two sources can drive the highlight: `seqPlayer` (immediate, exact) and
telemetry (up to ~800ms behind). One pure reducer resolves them:

```
decideSeqHighlight({
  seqPlayerActive, seqPlayerSeqId, seqPlayerStep,
  runseq,          // telemetry, or null
  viewSeqId,       // sequence currently shown
  isEditing,       // cursor in a step field / unsaved edits
}) -> { highlightSeqId, highlightStep, source, autoSwitch, jumpHintSeqId }
```

**Precedence.** An active `seqPlayer` wins outright — exact and lag-free, and
already driving the highlight today. Telemetry fills the gap only when
`seqPlayer` is idle, so browser-triggered playback is unchanged and nothing
regresses.

**Telemetry path** (seqPlayer idle, runseq active):
- Running sequence is the one on screen: highlight its step,
  `source: "telemetry"`, no switch.
- Running sequence differs:
  - not editing: `autoSwitch` to it, plus a "following playback" indicator.
  - editing: do not switch; return `jumpHintSeqId` so a clickable
    "<id> playing -> follow" badge appears. Auto-follow resumes on save/blur.

**Clearing.** runseq null/stale and seqPlayer idle: `highlightSeqId: null`,
back to rest.

Every branch — precedence, same-vs-different sequence, editing guard, clear —
is a table-driven unit test.

## Section 3 — Rendering, integration, testing

**One computed state.** Each poll, after `fetchStatus`, the DOM layer calls
`decideSeqHighlight(...)` and stores `currentSeqHighlight`. The step table
(today `seqPlayer.active && activeAuthoredStepIndex() === index`) and the
Arrange timeline both read `currentSeqHighlight` instead of `seqPlayer`
directly. Same yellow highlight, either source.

**Scope guard.** The `seqPlayer.active` checks that *disable edit buttons* stay
tied to `seqPlayer` only. A setlist running on-device is not browser-streamed
and must not lock the editor. Only the highlight reads the combined state; the
edit-lock is unchanged.

**isEditing.** True when `document.activeElement` is inside a step row/field, or
`seqDirty` is set. Gates auto-switch vs. the jump badge.

**Auto-switch** sets the shown sequence to the running one and re-renders, fired
only on an actual change so the view does not thrash every 800ms. The "following
playback" indicator lives in the sequencer header; the jump badge replaces it
when an edit blocks the switch.

**Testing.**
- Host-tested pure logic: `readRunseqTelemetry` (leader vs first-active,
  staleness), `mapFirmwareStepToAuthored` (chord/PREP/dwell folding, the
  count-mismatch fallback), `decideSeqHighlight` (every branch).
- Verified live: highlight follows a real setlist run; auto-switch respects an
  active edit; highlight clears when playback ends.

**Latency.** The telemetry highlight lags up to ~800ms and moves in whole-poll
jumps — fine for multi-second motions, approximate for fast chords. Browser-
driven playback stays crisp because `seqPlayer` still wins when active.

## Out of scope

- Sub-poll precision for fast chords (accept ~800ms granularity).
- Highlighting on the board instrument panels (this is about the sequencer
  section only).
