// Host tests for the pre-roll rate per machine (servo-ua4 follow-up).
//
// A pre-roll is the glide that walks a servo to a Motion's first keyframe
// before it starts. Its length is the distance times the servo's ms-per-
// percent — and that rate is a property of the MECHANISM, not the servo. The
// same goBILDA runs every board; the curtain's winch uses all 1800° (77 ms/%)
// while a wand uses 36° (1.54 ms/%).
//
// The bake glue handed the planner the winch constant for every machine, so a
// wand pre-roll was budgeted 7700ms for a sweep that takes 154ms. That is 50x
// of dead air on stage, and it is baked into the payload — which is how a
// board 1 bake reached 5277 bytes and stopped fitting in RAM.
//
// SEQ-BRIDGE-CORE is extracted on its own by verify_seq_bridges, so it cannot
// reach machineOf(). The rate arrives as an injected lookup instead.
//
// Run: node verify_preroll_rate.mjs   (or via test/run-all.mjs)

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
function near(n, got, want, tol) {
  if (Math.abs(got - want) <= tol) ok(n);
  else fail(`${n}\n    expected: ${want} ±${tol}\n    got:      ${got}`);
}

function block(startMarker, endMarker) {
  const s = html.indexOf(startMarker);
  const e = html.indexOf(endMarker, s + startMarker.length);
  if (s < 0 || e <= s) { fail(`${startMarker} not found or out of order`); process.exit(1); }
  return html.slice(s + startMarker.length, e);
}

const core = block("// === MACHINE-CORE START ===", "// === MACHINE-CORE END ===")
  + block("// === MOTION-SPEC-CORE START ===", "// === MOTION-SPEC-CORE END ===")
  + block("// === SEQ-BRIDGE-CORE START ===", "// === SEQ-BRIDGE-CORE END ===")
  + block("// === BAKE-PAYLOAD-CORE START ===", "// === BAKE-PAYLOAD-CORE END ===")
  + block("// === PREROLL-DURATION-CORE START ===", "// === PREROLL-DURATION-CORE END ===");

// Constants that live outside the extracted blocks, mirrored the same way
// verify_bake_payload.mjs mirrors them.
// MOTION-SPEC-CORE already declares the rest percent; only these two are
// defined outside every block this test extracts.
const shims = `
const SERVO_FEASIBILITY_MS_PER_PERCENT = 77;
const SEQ_MAX_STEPS = 16;
`;

const dir = mkdtempSync(join(tmpdir(), "preroll-rate-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, shims + core + "\nexport { seqPrerollOpts, rewriteSequencePreRolls, servoMsPerPercent, machineUnitPlural, preRollDurationMs };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
if (typeof mod.seqPrerollOpts !== "function") fail("no seqPrerollOpts()");
if (failed) process.exit(1);
const { seqPrerollOpts, rewriteSequencePreRolls, servoMsPerPercent, machineUnitPlural, preRollDurationMs } = mod;

console.log("=== Pre-roll rate per machine ===");

// The two rates, from the mechanism rather than the servo.
near("a winch takes 77ms per percent", servoMsPerPercent(3), 77, 0.5);
near("a wand takes about 1.5ms per percent", servoMsPerPercent(1), 1.54, 0.05);

// --- the glue must offer a per-machine lookup ------------------------------
const opts = seqPrerollOpts("sq-test");
eq("the planner is given a rate lookup", typeof opts.msPerPercentFor, "function");
near("...which knows the curtain", opts.msPerPercentFor(3), 77, 0.5);
near("...and the wands", opts.msPerPercentFor(1), 1.54, 0.05);
near("...and falls back for a machine with no servos", opts.msPerPercentFor(2), 77, 0.5);
near("...and for a board that is not a machine at all", opts.msPerPercentFor(99), 77, 0.5);

// --- planning a real pre-roll ---------------------------------------------
// One wand track travelling the full range from rest. On the curtain this is
// 7700ms; on the wands it must be ~154ms.
const wandMotion = {
  id: "mt-w", machine: 1, durationMs: 1000,
  tracks: [{ kind: "servo", boardId: 1, channel: 0, keyframes: [{ atMs: 0, value: 0 }, { atMs: 1000, value: 0 }] }],
};
const curtainMotion = {
  id: "mt-c", machine: 3, durationMs: 1000,
  tracks: [{ kind: "servo", boardId: 3, channel: 0, keyframes: [{ atMs: 0, value: 0 }, { atMs: 1000, value: 0 }] }],
};
const steps = (id, target) => [{ cmd: `MOTION ${id}`, durationMs: 1000, target: String(target) }];

function prepMsFor(motion, target) {
  const r = rewriteSequencePreRolls(steps(motion.id, target), [motion], seqPrerollOpts("sq-test"));
  const m = /PREP\s+(\d+)/.exec(r.steps[0].cmd || "");
  return m ? Number(m[1]) : 0;
}

const curtainPrep = prepMsFor(curtainMotion, 3);
const wandPrep = prepMsFor(wandMotion, 1);

// The curtain keeps the behaviour it has always had.
eq("a full-travel winch pre-roll is still measured in seconds", curtainPrep >= 7000, true);
// The wand no longer pays the winch's rate. The floor (800ms) still applies,
// so the assertion is "far below the winch", not an exact millisecond count.
eq("a wand pre-roll is nothing like the winch's", wandPrep < curtainPrep / 5, true);
eq("...and the two are no longer identical", wandPrep === curtainPrep, false);

// --- and it must say which parts are moving --------------------------------
// "3 winches" on a wand board is what sent a bench session looking for
// hardware faults. The noun comes from the machine, like the rate does.
eq("the curtain moves winches", machineUnitPlural(3), "winches");
eq("the wands move wands", machineUnitPlural(1), "wands");
eq("a machine with no servos has no parts to name", machineUnitPlural(2), "servos");
eq("neither does something that is not a machine", machineUnitPlural(99), "servos");


// --- the LIVE player has to use the same rate as the bake ------------------
// 813cc27 fixed the bake glue and left the live Motion player on the flat
// winch constant. The board logged DMOVE 0 0 7700 for a wand glide the bake
// log had planned at 800ms — the same move, the same rig, 9.6x apart. These
// pin the live planner's duration, which nothing covered before.

const rate = (boardId) => servoMsPerPercent(boardId);

eq("a full wand sweep clamps to the 800ms floor, not 7700",
   preRollDurationMs([{ boardId: 1, delta: 100 }], rate), 800);
eq("a full curtain sweep still gets its real 7700ms",
   preRollDurationMs([{ boardId: 3, delta: 100 }], rate), 7700);
eq("half a curtain sweep is half the time",
   preRollDurationMs([{ boardId: 3, delta: 50 }], rate), 3850);

// A pre-roll spanning machines waits for the slowest, the same way the bake
// path shares one maximum so the synchronized MOTION start stays aligned.
eq("a mixed pre-roll waits for the slowest machine",
   preRollDurationMs([{ boardId: 1, delta: 100 }, { boardId: 3, delta: 100 }], rate), 7700);
eq("...and the wand alone does not drag the curtain's rate in",
   preRollDurationMs([{ boardId: 1, delta: 100 }, { boardId: 1, delta: 40 }], rate), 800);

// The bug in one assertion: charging every machine the winch rate.
eq("the wands are no longer charged the curtain's rate",
   preRollDurationMs([{ boardId: 1, delta: 100 }], rate) === Math.ceil(100 * 77), false);

// A board with no servos, or one that is not a machine, keeps the 77 fallback
// that servoMsPerPercent already documents.
eq("a machine with no servos falls back to the winch rate",
   preRollDurationMs([{ boardId: 2, delta: 100 }], rate), 7700);

eq("no movement means no pre-roll", preRollDurationMs([], rate), 800);

// The live planner must actually call it. A pure function nothing uses would
// pass every test above and change nothing on the rig.
if (!/const durationMs = preRollDurationMs\(/.test(html)) {
  fail("_planMotionPreRoll does not use preRollDurationMs()");
} else {
  passed++; console.log("PASS: the live Motion player uses the shared duration helper");
}
if (/maxDelta \* SERVO_FEASIBILITY_MS_PER_PERCENT/.test(html)) {
  fail("the flat winch constant is still applied to a pre-roll");
} else {
  passed++; console.log("PASS: no pre-roll multiplies a delta by the flat winch constant");
}

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
