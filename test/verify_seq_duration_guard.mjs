// servo-4vl: a Sequence step that fires `MOTION <id>` must last at least the
// motion's own durationMs, or the next step interrupts before the servos finish
// (no queueing — schema §3). Exercises the pure guard extracted from
// servo_controller.html without a browser.

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, "..");
const html = readFileSync(join(root, "servo_controller.html"), "utf8");

function block(start, end) {
  const a = html.indexOf(start);
  const b = html.indexOf(end, a + start.length);
  if (a < 0 || b <= a) throw new Error(`missing block ${start}`);
  return html.slice(a + start.length, b);
}

const core = block("// === MACHINE-CORE START ===", "// === MACHINE-CORE END ===")
  + block("// === SEQ-DURATION-GUARD START ===", "// === SEQ-DURATION-GUARD END ===");
const dir = mkdtempSync(join(tmpdir(), "seq-dur-guard-"));
const modulePath = join(dir, "core.mjs");
writeFileSync(modulePath,
  `${core}\nexport { seqStepMotionId, motionStepDurationShortfall, stepMachineMismatch };\n`, "utf8");
const { seqStepMotionId, motionStepDurationShortfall, stepMachineMismatch } = await import(pathToFileURL(modulePath).href);

let passed = 0;
function check(condition, message) {
  if (!condition) throw new Error(`FAIL: ${message}`);
  passed++;
  console.log(`PASS: ${message}`);
}

const motions = [{ id: "mt-allup", durationMs: 8000 }, { id: "mt-stir", durationMs: 5000 }];
const shortfall = (cmd, durationMs) => motionStepDurationShortfall({ cmd, durationMs }, motions);

// The exact bug this guards: a 4000ms step firing an 8000ms motion.
const cut = shortfall("MOTION mt-allup", 4000);
check(cut && cut.need === 8000 && cut.have === 4000 && cut.shortMs === 4000,
  "a 4000ms step firing an 8000ms motion reports a 4000ms shortfall");

check(shortfall("MOTION mt-allup", 8000) === null,
  "a step exactly as long as its motion is not flagged");
check(shortfall("MOTION mt-allup", 12000) === null,
  "a step longer than its motion is not flagged");

// A missing/zero duration is the most-cut-off case, not a silent pass.
const zero = shortfall("MOTION mt-stir", 0);
check(zero && zero.need === 5000 && zero.have === 0,
  "a zero-duration MOTION step reports the full motion length as shortfall");

// Non-MOTION commands and unknown ids are out of scope (unknown ids surface in
// the preview row instead), so the guard must stay quiet for them.
check(shortfall("ROTATE 12", 100) === null, "a ROTATE step is never flagged");
check(shortfall("STOP", 0) === null, "a STOP step is never flagged");
check(shortfall("MOTION mt-ghost", 100) === null, "a MOTION with an unknown id is not flagged");

// Command parsing is case-insensitive and tolerant of surrounding whitespace.
check(seqStepMotionId("  motion MT-ALLUP ") === "MT-ALLUP",
  "seqStepMotionId is case-insensitive and trims whitespace");
check(shortfall("motion mt-allup", 4000) !== null,
  "a lowercase 'motion' command is still guarded");

// A step aimed at a machine that cannot carry out its command. A MOTION sent
// to a machine with no servos is the worst kind of authoring mistake: the
// board accepts it, runs the clock, and moves nothing.
const mmMotions = [{ id: "mt-curtain", durationMs: 4000, tracks: [
  { boardId: 3, channel: 0, keyframes: [{ atMs: 0, value: 100 }, { atMs: 4000, value: 40 }] },
  { boardId: 1, channel: 0, keyframes: [{ atMs: 0, value: 100 }] },
]}];
const mism = (target, cmd) => stepMachineMismatch({ target, cmd }, mmMotions);

check(mism(2, "MOTION mt-curtain") !== null, "a MOTION at the machine with no servos is flagged");
check(mism(2, "MOTION mt-curtain").fixTo === 3, "and it offers the machine the motion actually moves");
check(mism(3, "MOTION mt-curtain") === null, "the same MOTION at a machine with servos is fine");
check(mism(1, "MOTION mt-curtain") === null, "a machine with servos is never flagged, even if unused");
check(mism("all", "MOTION mt-curtain") === null, "an untargeted step is not a mismatch");
check(mism(2, "ROTATE 30") === null, "ROTATE at the DC-only machine is exactly what it is for");
check(mism(2, "STOP") === null, "STOP is not a mismatch anywhere");

console.log(`\n${passed} sequence duration-guard checks passed`);
