// Host tests for which machines the Motion editor draws (servo-bfe).
//
// The editor used to draw a section per machine unconditionally, so a gesture
// authored for the wands showed three winch tracks and a "the field has no
// servos" note underneath it. A Motion now says which machine it is for, and
// the timeline should say the same thing.
//
// The one case that must not be hidden is a motion whose declared machine and
// whose keyframes disagree. That state bakes to a machine the author is not
// looking at, so the offending board has to stay on screen rather than
// disappear behind the tag.
//
// Run: node verify_motion_editor_boards.mjs   (or via test/run-all.mjs)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}

function block(startMarker, endMarker) {
  const s = html.indexOf(startMarker);
  const e = html.indexOf(endMarker, s + startMarker.length);
  if (s < 0 || e <= s) { fail(`${startMarker} not found or out of order`); process.exit(1); }
  return html.slice(s + startMarker.length, e);
}

// MACHINE-CORE holds the machine table, inferredMachineIds, and the editor's
// visibility rule — all machine truth, no DOM.
const core = block("// === MACHINE-CORE START ===", "// === MACHINE-CORE END ===");

const dir = mkdtempSync(join(tmpdir(), "motion-editor-boards-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { motionEditorBoardIds, MACHINE_IDS };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
if (typeof mod.motionEditorBoardIds !== "function") fail("MACHINE-CORE does not export motionEditorBoardIds()");
if (failed) process.exit(1);
const { motionEditorBoardIds, MACHINE_IDS } = mod;

console.log("=== Motion editor board visibility ===");

/* A track that moves. inferredMachineIds counts a board as driven only when
   some track on it holds more than one value, which is what distinguishes
   authored motion from the rest keyframe normalizeMotion synthesizes. */
const moves = (boardId, channel) => ({
  kind: "servo", boardId, channel,
  keyframes: [{ atMs: 0, value: 100 }, { atMs: 800, value: 40 }],
});
const rests = (boardId, channel) => ({
  kind: "servo", boardId, channel, keyframes: [{ atMs: 0, value: 100 }],
});

// --- the ordinary case: the tag and the keyframes agree --------------------
eq("a wands motion draws only the wands",
   motionEditorBoardIds({ machine: 1, tracks: [moves(1, 0), rests(3, 0)] }), [1]);
eq("a curtain motion draws only the curtain",
   motionEditorBoardIds({ machine: 3, tracks: [moves(3, 0), rests(1, 0)] }), [3]);

// A freshly created motion is tagged before anything is authored in it. The
// empty timeline still has to be the one the author asked for.
eq("a tagged motion with nothing authored yet still draws its own machine",
   motionEditorBoardIds({ machine: 1, tracks: [rests(1, 0), rests(3, 0)] }), [1]);

// --- untagged motions: no tag, no claim ------------------------------------
// Hiding a board on the strength of inferredMachineIds would hide whatever an
// older motion happens not to move at that moment.
eq("an untagged motion draws every machine",
   motionEditorBoardIds({ tracks: [moves(3, 0)] }), MACHINE_IDS);
eq("a null tag draws every machine",
   motionEditorBoardIds({ machine: null, tracks: [moves(3, 0)] }), MACHINE_IDS);
eq("a tag naming no real machine draws every machine",
   motionEditorBoardIds({ machine: 7, tracks: [moves(3, 0)] }), MACHINE_IDS);
eq("no motion at all draws every machine", motionEditorBoardIds(null), MACHINE_IDS);

// --- the disagreement that must stay visible -------------------------------
// Duplicate-to-another-machine leaves rest keyframes behind on the source, so
// a real stray is a board that still moves. That bakes; it cannot be hidden.
eq("a stray moving board stays on screen next to the declared one",
   motionEditorBoardIds({ machine: 1, tracks: [moves(1, 0), moves(3, 0)] }), [1, 3]);
eq("a motion tagged for a machine it does not move still shows both",
   motionEditorBoardIds({ machine: 1, tracks: [moves(3, 0)] }), [1, 3]);
// Board order, not declared-first: the sections keep the same top-to-bottom
// order they have with the filter off, so nothing jumps when a tag changes.
eq("the sections stay in board order",
   motionEditorBoardIds({ machine: 3, tracks: [moves(1, 0)] }), [1, 3]);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
