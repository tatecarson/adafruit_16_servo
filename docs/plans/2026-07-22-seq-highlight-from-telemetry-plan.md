# Sequencer Highlight From Telemetry — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Drive the sequencer's yellow current-step highlight from `/status.json` `runseq` telemetry, so it shows during firmware-driven setlist/gallery playback, not just browser-driven `seqPlayer` playback.

**Architecture:** Three pure functions in a new `SEQ-HIGHLIGHT-CORE` marker block (host-testable with node, no DOM): `readRunseqTelemetry` selects the authoritative board's runseq, `mapFirmwareStepToAuthored` folds a flattened firmware step index back to an authored row, `decideSeqHighlight` resolves seqPlayer-vs-telemetry precedence and the auto-switch/jump-badge edit guard. A thin DOM layer calls them each poll, stores `currentSeqHighlight`, and the two existing render sites read that instead of `seqPlayer` directly.

**Tech Stack:** Single-file vanilla-JS app (`servo_controller.html`), node ESM host tests that extract marker blocks (`test/verify_*.mjs`), `make test`.

**Design doc:** `docs/plans/2026-07-22-seq-highlight-from-telemetry-design.md` — read it first. Beads: servo-4rq.

---

## Background the executor needs

**How the highlight works today.** Two render sites read `seqPlayer`:
- Step table row: `servo_controller.html:6698` — `const active = seqPlayer.active && activeAuthoredStepIndex() === index;`
- Arrange timeline selected block: `servo_controller.html:6447` — `const selected = seqPlayer.active ? ... activeAuthoredStepIndex() ... : seqArrangeSelectedIndex ...`

`activeAuthoredStepIndex()` (`:6754`) reads `seqPlayer.runSteps[seqPlayer.index].authoredIndex`.

**Telemetry is already polled and stored.** `fetchStatus(ip)` (`:2746`) does `lastStatus.set(ip, j)` and `lastSeen.set(ip, Date.now())`. The parsed `/status.json` includes:
```json
"runseq": { "active": true, "id": "sq-warm01", "step": 3, "steps": 6, "loop": false, "stepMs": 1200 }
```
Nothing reads `runseq` yet. Maps available at module scope: `lastStatus` (`:2712`), `lastSeen` (`:2711`), `boardIdByIp` (`:2713`), `boards` (array of ips).

**Two different flattenings — critical.** Browser live-play (`playSequenceFrom:6791`) flattens authored steps WITHOUT pre-rolls. The DEVICE bake runs `rewriteSequencePreRolls` (`:7180`) then `flattenDcLanes` (`:7448`), and `runseq.step` indexes into THAT baked list. So the telemetry mapping must reproduce the bake flattening. The pure mapping function takes an already-flattened list so it stays trivial to test; a thin DOM wrapper reproduces the bake layout and uses a count-guard (`flatCount === runseq.steps`) as the safety net — on mismatch it degrades to sequence-level highlight (no row).

**Canonical bake opts** (from `renderSequenceArrangement`): `{ seqId: seq.id, msPerPercent: SERVO_FEASIBILITY_MS_PER_PERCENT, minDeltaPercent: 5, floorMs: 800, restPercent: MOTION_SERVO_REST_PERCENT, maxSteps: SEQ_MAX_STEPS }`.

**`authoredIndex` survives flattening.** `rewriteSequencePreRolls` spreads `...step` on emitted steps; `flattenDcLanes` copies `authoredIndex` onto chords (`:7466`) and dwells (`:7496`). Tag authored steps with `authoredIndex` before reproducing, exactly as `playSequenceFrom:6791` does.

**Leader board:** `schedulerConfig.leaderBoardId` (1–3), default 1 (`:3912`).

---

## Task 1: `SEQ-HIGHLIGHT-CORE` block + `readRunseqTelemetry`

**Files:**
- Modify: `servo_controller.html` — add a new marker block. Put it right AFTER `// === SEQ-ARRANGE-CORE END ===` (`:6386`).
- Create: `test/verify_seq_highlight.mjs`
- Modify: `test/Makefile` — add `node verify_seq_highlight.mjs` to the `sim-verify` chain (next to `verify_seq_arrangement.mjs`).

**Step 1: Write the failing test.** Create `test/verify_seq_highlight.mjs`:
```javascript
// Host tests for the sequencer telemetry-highlight core (servo-4rq).
import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === SEQ-HIGHLIGHT-CORE START ===";
const END = "// === SEQ-HIGHLIGHT-CORE END ===";

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}

const s = html.indexOf(START), e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("SEQ-HIGHLIGHT-CORE markers not found"); process.exit(1); }
const dir = mkdtempSync(join(tmpdir(), "seq-highlight-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, html.slice(s + START.length, e) +
  "\nexport { readRunseqTelemetry, mapFirmwareStepToAuthored, decideSeqHighlight };\n", "utf8");
const { readRunseqTelemetry, mapFirmwareStepToAuthored, decideSeqHighlight } = await import(pathToFileURL(modPath).href);

console.log("=== Sequencer telemetry highlight ===");

// readRunseqTelemetry(byBoard, leaderBoardId, nowMs, freshMs)
// byBoard: [{ boardId, runseq, lastSeenMs }]
const run = (id, step, steps, active = true) => ({ active, id, step, steps, loop: false, stepMs: 0 });

eq("nothing active yields null",
   readRunseqTelemetry([{ boardId: 1, runseq: run("a", 0, 3, false), lastSeenMs: 100 }], 1, 100, 2500),
   null);
eq("prefers the leader board when it is active",
   readRunseqTelemetry([
     { boardId: 1, runseq: run("a", 1, 3), lastSeenMs: 100 },
     { boardId: 2, runseq: run("b", 2, 5), lastSeenMs: 100 },
   ], 2, 100, 2500),
   { id: "b", firmwareStep: 2, firmwareStepCount: 5, loop: false, stepMs: 0 });
eq("falls back to the lowest active board when the leader is idle",
   readRunseqTelemetry([
     { boardId: 3, runseq: run("c", 0, 2), lastSeenMs: 100 },
     { boardId: 2, runseq: run("b", 1, 4), lastSeenMs: 100 },
   ], 1, 100, 2500),
   { id: "b", firmwareStep: 1, firmwareStepCount: 4, loop: false, stepMs: 0 });
eq("a stale board is ignored even if active",
   readRunseqTelemetry([{ boardId: 1, runseq: run("a", 1, 3), lastSeenMs: 100 }], 1, 5000, 2500),
   null);
```

**Step 2: Run it, verify it fails.** `cd test && node verify_seq_highlight.mjs` → FAIL "markers not found".

**Step 3: Add the marker block + `readRunseqTelemetry`** after `:6386`:
```javascript

// === SEQ-HIGHLIGHT-CORE START ===
// Pure helpers that drive the sequencer's current-step highlight from device
// telemetry (servo-4rq), so it lights up during firmware-driven setlist/gallery
// playback and not only during browser-driven seqPlayer playback. No DOM here.

// Pick the authoritative board's runseq from a per-board snapshot. Prefer the
// setlist leader; otherwise the lowest-numbered active board (cluster members
// mirror the leader, so any active board reports the same run). A board whose
// last poll is older than freshMs is treated as not running, so a disconnected
// board cannot freeze a stale highlight. Returns a normalized run or null.
function readRunseqTelemetry(byBoard, leaderBoardId, nowMs, freshMs = 2500) {
  const live = (byBoard || [])
    .filter(b => b && b.runseq && b.runseq.active === true &&
                 Number.isFinite(b.lastSeenMs) && (nowMs - b.lastSeenMs) <= freshMs)
    .sort((a, b) => a.boardId - b.boardId);
  if (!live.length) return null;
  const chosen = live.find(b => b.boardId === leaderBoardId) || live[0];
  const r = chosen.runseq;
  return {
    id: String(r.id || ""),
    firmwareStep: Math.max(0, parseInt(r.step, 10) || 0),
    firmwareStepCount: Math.max(0, parseInt(r.steps, 10) || 0),
    loop: !!r.loop,
    stepMs: Math.max(0, parseInt(r.stepMs, 10) || 0),
  };
}
// === SEQ-HIGHLIGHT-CORE END ===
```

**Step 4: Run tests, verify pass.** `cd test && node verify_seq_highlight.mjs` → all PASS.

**Step 5: Wire into Makefile + run full suite.** Add `node verify_seq_highlight.mjs && \` after the `verify_seq_arrangement.mjs` line in `test/Makefile`. Run `cd test && make test` → green.

**Step 6: Commit.**
```bash
git add servo_controller.html test/verify_seq_highlight.mjs test/Makefile
git commit -m "feat: readRunseqTelemetry picks the authoritative board's run (servo-4rq)"
```

---

## Task 2: `mapFirmwareStepToAuthored`

Pure and simple by design: it takes an ALREADY-flattened device step list (each step optionally carrying `authoredIndex`) plus the firmware's reported step + count. The DOM layer (Task 4) reproduces the flattened list; keeping that out of this function is what makes it testable.

**Files:** Modify `servo_controller.html` (inside SEQ-HIGHLIGHT-CORE). Modify `test/verify_seq_highlight.mjs`.

**Step 1: Failing test.** Append to `verify_seq_highlight.mjs` before the summary:
```javascript
// mapFirmwareStepToAuthored(flatSteps, firmwareStep, firmwareStepCount)
// A baked list where authored step 0 owns 2 firmware steps (chord+motion),
// authored step 1 owns 1.
const flat = [
  { authoredIndex: 0 }, { authoredIndex: 0 }, { authoredIndex: 1 },
];
eq("maps a firmware step to its authored owner",
   mapFirmwareStepToAuthored(flat, 1, 3), { authoredIndex: 0, reliable: true });
eq("maps the last firmware step",
   mapFirmwareStepToAuthored(flat, 2, 3), { authoredIndex: 1, reliable: true });
eq("count mismatch (edited since bake) is unreliable, no row",
   mapFirmwareStepToAuthored(flat, 1, 6), { authoredIndex: null, reliable: false });
eq("out-of-range firmware step is unreliable",
   mapFirmwareStepToAuthored(flat, 9, 3), { authoredIndex: null, reliable: false });
eq("a step with no authoredIndex yields null but stays reliable",
   mapFirmwareStepToAuthored([{}, { authoredIndex: 0 }], 0, 2), { authoredIndex: null, reliable: true });
```

**Step 2: Run, verify fail** (`mapFirmwareStepToAuthored is not a function`).

**Step 3: Implement** inside SEQ-HIGHLIGHT-CORE (before END):
```javascript
// Fold a flattened firmware step index back to the authored row it belongs to.
// flatSteps is the device's baked step list (rewriteSequencePreRolls then
// flattenDcLanes), each step optionally carrying authoredIndex. If the browser's
// reproduced count disagrees with what the device reports, the library was
// edited since that bake and the per-step mapping would lie — report unreliable
// so the caller degrades to a sequence-level highlight.
function mapFirmwareStepToAuthored(flatSteps, firmwareStep, firmwareStepCount) {
  const list = Array.isArray(flatSteps) ? flatSteps : [];
  if (list.length !== firmwareStepCount) return { authoredIndex: null, reliable: false };
  if (firmwareStep < 0 || firmwareStep >= list.length) return { authoredIndex: null, reliable: false };
  const ai = list[firmwareStep] && list[firmwareStep].authoredIndex;
  return { authoredIndex: Number.isInteger(ai) ? ai : null, reliable: true };
}
```

**Step 4: Run, verify pass.**

**Step 5: Commit.**
```bash
git add servo_controller.html test/verify_seq_highlight.mjs
git commit -m "feat: mapFirmwareStepToAuthored folds a baked step to its authored row (servo-4rq)"
```

---

## Task 3: `decideSeqHighlight`

The reducer that resolves precedence and the edit guard.

**Files:** Modify `servo_controller.html` (SEQ-HIGHLIGHT-CORE). Modify `test/verify_seq_highlight.mjs`.

**Step 1: Failing test.** Append:
```javascript
// decideSeqHighlight({ seqPlayerActive, seqPlayerSeqId, seqPlayerStep,
//   runseqSeqId, runseqStep, viewSeqId, isEditing }) -> highlight decision
const D = decideSeqHighlight;

eq("seqPlayer wins outright when active",
   D({ seqPlayerActive: true, seqPlayerSeqId: "a", seqPlayerStep: 2,
       runseqSeqId: "b", runseqStep: 5, viewSeqId: "a", isEditing: false }),
   { highlightSeqId: "a", highlightStep: 2, source: "player", autoSwitch: false, jumpHintSeqId: null });

eq("telemetry highlights the step when viewing the running sequence",
   D({ seqPlayerActive: false, runseqSeqId: "a", runseqStep: 1, viewSeqId: "a", isEditing: false }),
   { highlightSeqId: "a", highlightStep: 1, source: "telemetry", autoSwitch: false, jumpHintSeqId: null });

eq("telemetry auto-switches to a different running sequence when idle",
   D({ seqPlayerActive: false, runseqSeqId: "b", runseqStep: 0, viewSeqId: "a", isEditing: false }),
   { highlightSeqId: "b", highlightStep: 0, source: "telemetry", autoSwitch: true, jumpHintSeqId: null });

eq("editing blocks the switch and offers a jump badge instead",
   D({ seqPlayerActive: false, runseqSeqId: "b", runseqStep: 0, viewSeqId: "a", isEditing: true }),
   { highlightSeqId: null, highlightStep: null, source: "telemetry", autoSwitch: false, jumpHintSeqId: "b" });

eq("a null step (unreliable mapping) still highlights the sequence title",
   D({ seqPlayerActive: false, runseqSeqId: "a", runseqStep: null, viewSeqId: "a", isEditing: false }),
   { highlightSeqId: "a", highlightStep: null, source: "telemetry", autoSwitch: false, jumpHintSeqId: null });

eq("nothing running clears everything",
   D({ seqPlayerActive: false, runseqSeqId: null, runseqStep: null, viewSeqId: "a", isEditing: false }),
   { highlightSeqId: null, highlightStep: null, source: "none", autoSwitch: false, jumpHintSeqId: null });
```

**Step 2: Run, verify fail.**

**Step 3: Implement** inside SEQ-HIGHLIGHT-CORE:
```javascript
// Resolve which source drives the highlight and whether the view should follow
// playback. An active seqPlayer wins — it is exact and lag-free. Telemetry fills
// the gap only when seqPlayer is idle: highlight in place when the run is the
// sequence on screen; otherwise auto-switch to it unless the operator is editing,
// in which case surface a jump badge and leave the view put. `null` inputs
// (nothing running) clear the highlight.
function decideSeqHighlight(x) {
  if (x.seqPlayerActive) {
    return { highlightSeqId: x.seqPlayerSeqId, highlightStep: x.seqPlayerStep,
             source: "player", autoSwitch: false, jumpHintSeqId: null };
  }
  const runId = x.runseqSeqId || null;
  if (!runId) {
    return { highlightSeqId: null, highlightStep: null, source: "none", autoSwitch: false, jumpHintSeqId: null };
  }
  const step = Number.isInteger(x.runseqStep) ? x.runseqStep : null;
  if (runId === x.viewSeqId) {
    return { highlightSeqId: runId, highlightStep: step, source: "telemetry", autoSwitch: false, jumpHintSeqId: null };
  }
  if (x.isEditing) {
    return { highlightSeqId: null, highlightStep: null, source: "telemetry", autoSwitch: false, jumpHintSeqId: runId };
  }
  return { highlightSeqId: runId, highlightStep: step, source: "telemetry", autoSwitch: true, jumpHintSeqId: null };
}
```

**Step 4: Run, verify pass. Step 5: full `make test`. Step 6: Commit.**
```bash
git add servo_controller.html test/verify_seq_highlight.mjs
git commit -m "feat: decideSeqHighlight resolves player-vs-telemetry precedence + edit guard (servo-4rq)"
```

---

## Task 4: DOM wiring — feed the two render sites

No new pure logic; connect the core to the running app. This task does NOT auto-switch yet (Task 5) — it only makes the highlight light up when you are already viewing the running sequence.

**Files:** Modify `servo_controller.html` only (DOM layer, outside marker blocks).

**Step 1: Add a module-scope state + a reproduction wrapper.** Near the other sequencer state (after `let seqDirty = false;` at `:3775`):
```javascript
// Telemetry-driven highlight (servo-4rq). Recomputed each poll; the step table
// and Arrange timeline read it instead of seqPlayer directly, so the highlight
// follows firmware-driven setlist/gallery playback too.
let currentSeqHighlight = { highlightSeqId: null, highlightStep: null, source: "none", autoSwitch: false, jumpHintSeqId: null };

// Reproduce the DEVICE bake flattening (rewriteSequencePreRolls then
// flattenDcLanes) so a firmware step index maps to the right authored row.
// Live-play flattening is different (no pre-rolls) and must NOT be used here.
function flattenSequenceForDevice(seq, motions) {
  const authored = (seq.steps || []).map((s, i) => ({ ...normalizeStep(s), authoredIndex: i }));
  const bridged = rewriteSequencePreRolls(authored, motions, {
    seqId: seq.id, msPerPercent: SERVO_FEASIBILITY_MS_PER_PERCENT,
    minDeltaPercent: 5, floorMs: 800, restPercent: MOTION_SERVO_REST_PERCENT, maxSteps: SEQ_MAX_STEPS,
  });
  return flattenDcLanes(bridged.steps, { loop: false, maxSteps: SEQ_MAX_STEPS }).steps;
}
```

**Step 2: Compute `currentSeqHighlight` each poll.** In `pollOnce` (`:3265`), after the status fetches settle (end of the function, before it reschedules), add a call to a new `updateSeqHighlight()`. Implement it near the wrapper:
```javascript
function updateSeqHighlight() {
  const lib = sequenceLibrary();
  const byBoard = boards.map(ip => ({
    boardId: boardIdByIp.get(ip),
    runseq: (lastStatus.get(ip) || {}).runseq,
    lastSeenMs: lastSeen.get(ip) || 0,
  })).filter(b => Number.isInteger(b.boardId));
  const leader = Math.max(1, Math.min(3, parseInt(lib.schedulerConfig?.leaderBoardId, 10) || 1));
  const run = readRunseqTelemetry(byBoard, leader, Date.now());

  let runseqStep = null;
  if (run) {
    const runSeq = (lib.sequences || []).find(s => s.id === run.id);
    if (runSeq) {
      const flat = flattenSequenceForDevice(runSeq, lib.motions || []);
      runseqStep = mapFirmwareStepToAuthored(flat, run.firmwareStep, run.firmwareStepCount).authoredIndex;
    }
  }
  const viewSeq = getCurrentSequence(lib);
  const decision = decideSeqHighlight({
    seqPlayerActive: seqPlayer.active,
    seqPlayerSeqId: viewSeq ? viewSeq.id : null,     // seqPlayer always plays the shown sequence
    seqPlayerStep: seqPlayer.active ? activeAuthoredStepIndex() : null,
    runseqSeqId: run ? run.id : null,
    runseqStep,
    viewSeqId: viewSeq ? viewSeq.id : null,
    isEditing: isSeqEditing(),
  });
  const changedSwitchTarget = decision.autoSwitch && decision.highlightSeqId !== (viewSeq && viewSeq.id);
  currentSeqHighlight = decision;
  // Task 5 handles autoSwitch + badge. For now just re-render the highlight.
  // NOTE (post-review): the step-table re-render gate compares ONLY
  // highlightSeqId/highlightStep. The badge fields (source/jumpHintSeqId)
  // must NOT enter that gate — a table rebuild destroys the operator's focus
  // and reverts unsaved cell edits; the badge renders independently each poll.
  if (!changedSwitchTarget) renderSequenceHighlightOnly();
}
```
For this task, define a stub `function isSeqEditing() { return false; }` and `function renderSequenceHighlightOnly() { renderSequenceSteps?.(); if (seqArrangeDialogEl?.open) renderSequenceArrangement(); }` (use the actual step-table render fn name — find it near `:6697`). Task 5 fills in `isSeqEditing` and the badge/switch.

**Step 3: Point the two render sites at `currentSeqHighlight`.** Add a helper in the DOM layer:
```javascript
// The highlighted authored index for `seqId`, or -1. seqPlayer still wins via
// decideSeqHighlight, so browser-driven playback is unchanged.
function highlightedAuthoredIndex(seqId) {
  return currentSeqHighlight.highlightSeqId === seqId ? currentSeqHighlight.highlightStep : null;
}
```
- Step table (`:6698`): change to
  `const active = highlightedAuthoredIndex(seq.id) === index;`
- Arrange (`:6447`): change the `selected` expression to prefer the highlight when present:
  ```javascript
  const hi = highlightedAuthoredIndex(seq.id);
  const selected = hi != null
    ? Math.max(0, Math.min(hi, steps.length - 1))
    : Math.max(0, Math.min(seqArrangeSelectedIndex, steps.length - 1));
  ```
  Keep the existing `seqPlayer.active ? ...` fallback semantics — since seqPlayer wins in `decideSeqHighlight`, `hi` already reflects it.

**IMPORTANT scope guard:** Do NOT change any `seqPlayer.active` check that DISABLES edit buttons (e.g. `:6422`, `:6423`, `:6485`, the Arrange status text `:6536`). Those stay tied to `seqPlayer` — a setlist on-device must not lock the editor. Only the highlight reads `currentSeqHighlight`.

**Step 4: Verify in the browser.** Follow the `<preview_tools>` workflow:
- `preview_start` the file, seed a library + a runseq via `javascript_tool` (simulate `lastStatus`/`lastSeen`), call `updateSeqHighlight()`, confirm the row/block highlights while `seqPlayer` is idle. Confirm `seqPlayer.active` still wins when set.

**Step 5: Commit.**
```bash
git add servo_controller.html
git commit -m "feat: sequencer highlight follows runseq telemetry when seqPlayer is idle (servo-4rq)"
```

---

## Task 5: Auto-switch, following indicator, jump badge, isEditing

**Files:** Modify `servo_controller.html` (DOM + a little CSS).

**Step 1: Real `isSeqEditing()`.** Replace the stub:
```javascript
// True while the operator is mid-edit, so auto-follow must not yank the view.
function isSeqEditing() {
  if (seqDirty) return true;
  const el = document.activeElement;
  return !!(el && seqStepsEl && seqStepsEl.contains(el));
}
```

**Step 2: Auto-switch in `updateSeqHighlight`.** Replace the `changedSwitchTarget` tail:
```javascript
  currentSeqHighlight = decision;
  if (decision.autoSwitch && decision.highlightSeqId && decision.highlightSeqId !== (viewSeq && viewSeq.id)) {
    switchToSequence(decision.highlightSeqId);   // sets currentSeqId + re-renders (find the existing setter near currentSeqId usage)
  } else {
    renderSequenceHighlightOnly();
  }
  renderSeqFollowIndicator(decision);
```
Find how the app already changes the shown sequence (search `currentSeqId =` and the sequence `<select>` change handler) and reuse that path as `switchToSequence(id)` rather than inventing a new one. It must re-render the step table + Arrange.

**Step 3: Following indicator + jump badge.** Add a small element in the sequencer header (near the seq name/id label around `:2294`), e.g. `<span id="seqFollowBadge" class="seq-follow" hidden></span>`, and:
```javascript
function renderSeqFollowIndicator(d) {
  const el = document.getElementById("seqFollowBadge");
  if (!el) return;
  if (d.source === "telemetry" && d.highlightSeqId && !d.jumpHintSeqId) {
    el.hidden = false; el.className = "seq-follow following"; el.textContent = "● following playback";
    el.onclick = null;
  } else if (d.jumpHintSeqId) {
    el.hidden = false; el.className = "seq-follow jump";
    el.textContent = `${d.jumpHintSeqId} playing → follow`;
    el.onclick = () => switchToSequence(d.jumpHintSeqId);
  } else {
    el.hidden = true; el.onclick = null;
  }
}
```
CSS (near other sequencer styles): a muted pill for `.following`, an amber clickable pill for `.jump` (`cursor:pointer`).

**Step 4: Verify in the browser.**
- Seed a runseq for a DIFFERENT sequence than the one shown, `updateSeqHighlight()` → view auto-switches, badge shows "following playback".
- Focus a step field (or set `seqDirty`), re-run → view stays put, badge shows "… → follow"; click it → switches.
- Clear the runseq → highlight and badge clear.
- Screenshot for the record.

**Step 5: Commit.**
```bash
git add servo_controller.html
git commit -m "feat: auto-follow running sequence with edit-safe jump badge (servo-4rq)"
```

---

## Task 6: Final verification + PR

**Step 1:** `cd test && make test` → all green (including `verify_seq_highlight.mjs`).
**Step 2:** Re-read the design doc; confirm every "Testing" bullet is covered (pure fns host-tested; live behaviors screenshotted).
**Step 3:** `git checkout -- .beads issues.jsonl` to drop beads churn, then push branch `claude/seq-highlight-telemetry` and open a PR against main summarizing: problem, the runseq wiring, the three pure functions, the edit-safety guard, and the count-mismatch fallback. Note the ~800ms latency limitation and that the edit-lock intentionally stays on seqPlayer.
**Step 4:** `bd close servo-4rq --reason="..."`.

---

## Notes / gotchas for the executor

- **Reproduce the BAKE flattening, never live-play.** Live-play skips pre-rolls; the device runs them, and `runseq.step` counts them. The count-guard in `mapFirmwareStepToAuthored` is the safety net when reproduction and device disagree — don't remove it.
- **seqPlayer must keep winning.** Never let telemetry override an active browser run; `decideSeqHighlight` enforces this, and the render sites go through it.
- **Don't touch the edit-locks.** Only the highlight reads `currentSeqHighlight`; button-disabling stays on `seqPlayer.active`.
- **No thrash.** Auto-switch only fires when the target actually differs from the shown sequence; otherwise a plain highlight re-render.
- **Find real function names.** The plan cites line numbers for the step-table render and the sequence-switch path — confirm the actual identifiers before wiring (`renderSequenceSteps`/equivalent, the `<select>` change handler).
