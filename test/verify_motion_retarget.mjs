// Host tests for Duplicate's "copy to" target (servo-bfe follow-up).
//
// Duplicate can move a gesture to another machine, but only when it can tell
// which machine the gesture is currently on — exactly one machine has to
// actually move. When it cannot, the old code fell through to a plain copy and
// said "duplicated X", so the target you picked was ignored without a word.
//
// These check the plan the button acts on: whether the retarget runs, whether
// a copy happens at all, and what the operator is told when it does not.
//
// Run: node verify_motion_retarget.mjs   (or via test/run-all.mjs)

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
function has(n, got, needle) {
  if (typeof got === "string" && got.includes(needle)) ok(n);
  else fail(`${n}\n    expected to contain: ${JSON.stringify(needle)}\n    got: ${JSON.stringify(got)}`);
}

function block(startMarker, endMarker) {
  const s = html.indexOf(startMarker);
  const e = html.indexOf(endMarker, s + startMarker.length);
  if (s < 0 || e <= s) { fail(`${startMarker} not found or out of order`); process.exit(1); }
  return html.slice(s + startMarker.length, e);
}

const core = block("// === MACHINE-CORE START ===", "// === MACHINE-CORE END ===");

const dir = mkdtempSync(join(tmpdir(), "motion-retarget-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { motionRetargetPlan };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
if (typeof mod.motionRetargetPlan !== "function") fail("MACHINE-CORE does not export motionRetargetPlan()");
if (failed) process.exit(1);
const { motionRetargetPlan } = mod;

console.log("=== Duplicate · copy to another machine ===");

const moves = (boardId, channel) => ({
  kind: "servo", boardId, channel,
  keyframes: [{ atMs: 0, value: 100 }, { atMs: 800, value: 40 }],
});
const rests = (boardId, channel) => ({
  kind: "servo", boardId, channel, keyframes: [{ atMs: 0, value: 100 }],
});

// --- the working case ------------------------------------------------------
const curtainMotion = { machine: 3, tracks: [moves(3, 0), moves(3, 1), rests(1, 0)] };
eq("a curtain gesture retargets to the wands",
   motionRetargetPlan(curtainMotion, 1), { retarget: true, from: 3, to: 1, copy: true, message: null });

// --- no target chosen: an ordinary duplicate, and nothing to explain --------
eq("'same machine' is a plain copy",
   motionRetargetPlan(curtainMotion, null), { retarget: false, from: 3, to: null, copy: true, message: null });

// --- the cases that used to fall through silently --------------------------
// Nothing moves: there is no source machine to read, so there is no gesture to
// carry across. The copy still happens; the operator has to be told it did not
// go where they asked.
const stillMotion = { machine: 3, tracks: [rests(3, 0), rests(1, 0)] };
const still = motionRetargetPlan(stillMotion, 1);
eq("a motion that moves nothing cannot be retargeted", still.retarget, false);
eq("...but it is still copied", still.copy, true);
has("...and says nothing moves", still.message, "nothing moves");
has("...and names the machine it did not reach", still.message, "centre wands");

// Two machines moving: which one is the gesture? The operator has to say.
const splitMotion = { machine: null, tracks: [moves(3, 0), moves(1, 0)] };
const split = motionRetargetPlan(splitMotion, 1);
eq("a motion moving two machines cannot be retargeted", split.retarget, false);
eq("...but it is still copied", split.copy, true);
has("...and names both machines", split.message, "centre wands");
has("...and names both machines", split.message, "dowel curtain");

// Already there.
const already = motionRetargetPlan(curtainMotion, 3);
eq("retargeting to the machine it is already on does nothing", already.retarget, false);
eq("...but it is still copied", already.copy, true);
has("...and says so", already.message, "already on the dowel curtain");

// --- the hard refusal ------------------------------------------------------
// The field has no servos. Not reachable from the picker, which lists only
// machines that have them, but the plan must not invent tracks if it ever is.
const field = motionRetargetPlan(curtainMotion, 2);
eq("the field refuses the copy outright", field.retarget, false);
eq("...and no copy is made", field.copy, false);
has("...and says why", field.message, "no servos");

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
