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

const core = block("// === SEQ-DURATION-GUARD START ===", "// === SEQ-DURATION-GUARD END ===");
const dir = mkdtempSync(join(tmpdir(), "seq-dur-guard-"));
const modulePath = join(dir, "core.mjs");
writeFileSync(modulePath,
  `${core}\nexport { seqStepMotionId, motionStepDurationShortfall };\n`, "utf8");
const { seqStepMotionId, motionStepDurationShortfall } = await import(pathToFileURL(modulePath).href);

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

console.log(`\n${passed} sequence duration-guard checks passed`);
