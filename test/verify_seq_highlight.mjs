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
  "\nexport { readRunseqTelemetry };\n", "utf8");
const { readRunseqTelemetry } = await import(pathToFileURL(modPath).href);

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
eq("falls back to the lowest active board when the leader is absent",
   readRunseqTelemetry([
     { boardId: 3, runseq: run("c", 0, 2), lastSeenMs: 100 },
     { boardId: 2, runseq: run("b", 1, 4), lastSeenMs: 100 },
   ], 1, 100, 2500),
   { id: "b", firmwareStep: 1, firmwareStepCount: 4, loop: false, stepMs: 0 });
eq("falls back to an active board when the leader is present but idle",
   readRunseqTelemetry([
     { boardId: 1, runseq: run("a", 0, 3, false), lastSeenMs: 100 },
     { boardId: 2, runseq: run("b", 1, 4), lastSeenMs: 100 },
   ], 1, 100, 2500),
   { id: "b", firmwareStep: 1, firmwareStepCount: 4, loop: false, stepMs: 0 });
eq("a stale board is ignored even if active",
   readRunseqTelemetry([{ boardId: 1, runseq: run("a", 1, 3), lastSeenMs: 100 }], 1, 5000, 2500),
   null);
eq("a null byBoard yields null",
   readRunseqTelemetry(null, 1, 100),
   null);
eq("string-valued step and steps are coerced to numbers",
   readRunseqTelemetry([{ boardId: 1, runseq: { active: true, id: "a", step: "2", steps: "5", loop: false, stepMs: 0 }, lastSeenMs: 100 }], 1, 100, 2500),
   { id: "a", firmwareStep: 2, firmwareStepCount: 5, loop: false, stepMs: 0 });

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
