// Host tests for the pure Sequence arrangement geometry/reorder helpers
// embedded in servo_controller.html (servo-uyb).

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === SEQ-ARRANGE-CORE START ===";
const END = "// === SEQ-ARRANGE-CORE END ===";

let passed = 0, failed = 0;
function fail(message) { console.error(`  FAIL: ${message}`); failed++; }
function ok(name) { console.log(`  PASS  ${name}`); passed++; }
function eq(name, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(name);
  else fail(`${name}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}
function matches(name, pattern) {
  if (pattern.test(html)) ok(name);
  else fail(`${name}\n    pattern not found: ${pattern}`);
}

const start = html.indexOf(START);
const end = html.indexOf(END, start + START.length);
if (start < 0 || end <= start) {
  fail("SEQ-ARRANGE-CORE markers not found or out of order");
  process.exit(1);
}
const core = html.slice(start + START.length, end);
const dir = mkdtempSync(join(tmpdir(), "seq-arrangement-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, `${core}\nexport { sequenceArrangementLayout, reorderSequenceSteps, sequenceStepForMotion, chooseSequenceArrangementMotion, toggleSequenceStepPreRoll, carryStepAuthoringFlags };\n`, "utf8");
const { sequenceArrangementLayout, reorderSequenceSteps, sequenceStepForMotion, chooseSequenceArrangementMotion, toggleSequenceStepPreRoll, carryStepAuthoringFlags } = await import(pathToFileURL(modPath).href);

console.log("=== Sequence arrangement timeline ===");

const steps = [
  { cmd: "MOTION rise", durationMs: 1000 },
  { cmd: "WAIT", durationMs: 2500 },
  { cmd: "STOP", durationMs: 500 },
];
const layout = sequenceArrangementLayout(steps, 100, 30);
eq("total duration sums every step", layout.totalMs, 4000);
eq("blocks start at cumulative times", layout.blocks.map(block => block.startMs), [0, 1000, 3500]);
eq("positive durations map proportionally to width", layout.blocks.map(block => block.widthPx), [100, 250, 50]);
eq("canvas width is the sum of blocks", layout.widthPx, 400);

const edge = sequenceArrangementLayout([
  { durationMs: 0 },
  { durationMs: "bad" },
  { durationMs: -20 },
  { durationMs: 1000 },
], 50, 30);
eq("zero/invalid durations remain visible at minimum width", edge.blocks.map(block => block.widthPx), [30, 30, 30, 50]);
eq("zero/invalid durations do not inflate sequence time", edge.totalMs, 1000);
eq("negative duration normalizes to zero", edge.blocks.map(block => block.durationMs), [0, 0, 0, 1000]);

const original = [{ cmd: "A" }, { cmd: "B" }, { cmd: "C" }, { cmd: "D" }];
const forward = reorderSequenceSteps(original, 0, 3);
eq("drag forward reorders at the drop index", forward.map(step => step.cmd), ["B", "C", "D", "A"]);
eq("reorder does not mutate source array", original.map(step => step.cmd), ["A", "B", "C", "D"]);
eq("drag backward reorders at the drop index", reorderSequenceSteps(original, 3, 1).map(step => step.cmd), ["A", "D", "B", "C"]);
eq("out-of-range drag is a no-op", reorderSequenceSteps(original, 9, 0).map(step => step.cmd), ["A", "B", "C", "D"]);

const motion = { id: "tidal-drift", durationMs: 4800 };
eq("motion choice becomes a duration-matched Sequence step", sequenceStepForMotion(motion), {
  cmd: "MOTION tidal-drift", durationMs: 4800, target: "all", label: "", hold: false,
});
const added = chooseSequenceArrangementMotion(original, motion, 1, "add");
eq("add choice inserts Motion after selected step", added.map(step => step.cmd), ["A", "B", "MOTION tidal-drift", "C", "D"]);
eq("add choice carries Motion duration", added[2].durationMs, 4800);
const replaceSource = [{ cmd: "STOP", durationMs: 1000, target: "2", label: "finale", hold: true }];
const replaced = chooseSequenceArrangementMotion(replaceSource, motion, 0, "replace");
eq("replace choice swaps command and duration", [replaced[0].cmd, replaced[0].durationMs], ["MOTION tidal-drift", 4800]);
eq("replace choice preserves step metadata", [replaced[0].target, replaced[0].label, replaced[0].hold], ["2", "finale", true]);
eq("invalid Motion choice is a no-op", chooseSequenceArrangementMotion(original, {}, 0, "add").map(step => step.cmd), ["A", "B", "C", "D"]);

// --- servo-cjv: clicking a pre-roll block switches it off and back on ---
const prSteps = [
  { cmd: "MOTION a", durationMs: 1000, target: "all", label: "", hold: false },
  { cmd: "MOTION b", durationMs: 2000, target: "all", label: "", hold: false },
];
const off = toggleSequenceStepPreRoll(prSteps, 1);
eq("toggling marks the step as refusing its pre-roll", off[1].noPrep, true);
eq("toggling leaves every other step alone", off[0].noPrep, undefined);
eq("toggling does not mutate the source steps", prSteps[1].noPrep, undefined);
// Cleared rather than set false, so a sequence that never opted out serializes
// exactly as it did before this feature existed.
eq("toggling back removes the field instead of storing false",
   Object.prototype.hasOwnProperty.call(toggleSequenceStepPreRoll(off, 1)[1], "noPrep"), false);
eq("out-of-range index is a no-op", toggleSequenceStepPreRoll(prSteps, 7).map(s => s.cmd), ["MOTION a", "MOTION b"]);
eq("negative index is a no-op", toggleSequenceStepPreRoll(prSteps, -1).map(s => s.cmd), ["MOTION a", "MOTION b"]);

// The // 07 step table has inputs for cmd/duration/target/label/hold/dc and
// nothing for noPrep, so rebuilding a step from its row would delete the
// opt-out. Editing a label must not silently resurrect a deleted pre-roll.
eq("a rebuilt row keeps the pre-roll opt-out it came from",
   carryStepAuthoringFlags({ cmd: "MOTION a", label: "edited" }, { cmd: "MOTION a", noPrep: true }).noPrep, true);
eq("a rebuilt row without an opt-out gains no field",
   Object.prototype.hasOwnProperty.call(carryStepAuthoringFlags({ cmd: "MOTION a" }, { cmd: "MOTION a" }), "noPrep"), false);
eq("a missing prior step is harmless",
   carryStepAuthoringFlags({ cmd: "MOTION a" }, undefined).cmd, "MOTION a");

matches("pre-roll blocks carry a toggle target for the click handler", /data-arrange-prep="/);
matches("disabled pre-roll ghost has its own style rule", /\.seq-arrange-block\.bridge\.disabled\s*\{/s);
matches("a long refused glide is flagged as a jump", /\.seq-arrange-block\.bridge\.disabled\.warn\s*\{/s);

matches("servo preview uses the Motion editor's amber color", /\.seq-arrange-preview-row polyline\s*\{[^}]*stroke:\s*var\(--amber\)/s);
matches("DC preview uses the Motion editor's phosphor color", /\.seq-arrange-preview-row\.dc polyline\s*\{[^}]*stroke:\s*var\(--phosphor\)/s);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
