// Tests for the Sequence step-preview strip builder (servo-seqviz).
// Pure logic lives in the SEQ-PREVIEW-CORE block of servo_controller.html:
//   motionPreviewStrips(motion, specs)
//
// Run: node verify_seq_preview.mjs   (or via the make test node run)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === SEQ-PREVIEW-CORE START ===";
const END = "// === SEQ-PREVIEW-CORE END ===";

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}
function approx(n, got, want, tol = 1e-6) {
  if (Math.abs(got - want) <= tol) ok(n);
  else fail(`${n}\n    expected: ${want} (±${tol})\n    got:      ${got}`);
}

const s = html.indexOf(START);
const e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("SEQ-PREVIEW-CORE markers not found or out of order"); process.exit(1); }
const core = html.slice(s + START.length, e);

const dir = mkdtempSync(join(tmpdir(), "seq-preview-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { motionPreviewStrips };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
if (typeof mod.motionPreviewStrips !== "function") {
  fail("SEQ-PREVIEW-CORE does not export motionPreviewStrips()");
  process.exit(1);
}
const { motionPreviewStrips } = mod;

console.log("=== Sequence step preview strips ===");

const SERVO = { kind: "servo", boardId: 1, channel: 0, label: "B1.S0", min: 0, max: 100, unit: "%" };
const DC = { kind: "dc", boardId: 1, channel: 0, label: "B1.DC", min: -100, max: 100, unit: "spd" };

// ---- only tracks that actually move (>=2 keyframes) produce strips ---------
const motion = {
  durationMs: 8000,
  tracks: [
    { kind: "servo", boardId: 1, channel: 0, keyframes: [
      { atMs: 0, value: 0 }, { atMs: 4000, value: 100 }, { atMs: 8000, value: 0 },
    ] },
    { kind: "dc", boardId: 1, channel: 0, keyframes: [
      { atMs: 0, value: 0 }, { atMs: 8000, value: -100 },
    ] },
    // empty track: no keyframes → skipped
    { kind: "servo", boardId: 1, channel: 1, keyframes: [] },
    // static hold: single default keyframe → skipped (no motion to show)
    { kind: "servo", boardId: 1, channel: 2, keyframes: [{ atMs: 0, value: 60 }] },
  ],
};
const strips = motionPreviewStrips(motion, [
  SERVO, DC,
  { ...SERVO, channel: 1, label: "B1.S1" },
  { ...SERVO, channel: 2, label: "B1.S2" },
]);
eq("skips empty + single-keyframe tracks (2 strips from 4 specs)", strips.length, 2);
eq("strip keys", strips.map(s => s.key), ["1:servo:0", "1:dc:0"]);
eq("servo strip exposes start/end values", [strips[0].startValue, strips[0].endValue], [0, 0]);
eq("servo strip exposes min/max range", [strips[0].minValue, strips[0].maxValue], [0, 100]);
eq("servo strip exposes keyframe count", strips[0].keyframeCount, 3);
eq("DC strip exposes start/end values", [strips[1].startValue, strips[1].endValue], [0, -100]);
eq("DC strip exposes min/max range", [strips[1].minValue, strips[1].maxValue], [-100, 0]);

// ---- servo x/y mapping ----------------------------------------------------
const sv = strips[0].points;
approx("servo t=0 → x 0%", sv[0].xPct, 0);
approx("servo t=4000/8000 → x 50%", sv[1].xPct, 50);
approx("servo t=8000 → x 100%", sv[2].xPct, 100);
approx("servo value=0 (up) → y 0% (top)", sv[0].yPct, 0);
approx("servo value=100 (down) → y 100% (bottom)", sv[1].yPct, 100);

// ---- dc mapping: positive at top, negative at bottom, 0 mid ---------------
const dc = strips[1].points;
approx("dc value=0 → y 50% (mid)", dc[0].yPct, 50);
approx("dc value=-100 → y 100% (bottom)", dc[1].yPct, 100);

// ---- values clamp into 0..100 even if out of nominal range ----------------
const overshoot = motionPreviewStrips(
  { durationMs: 1000, tracks: [{ kind: "servo", boardId: 2, channel: 2, keyframes: [
    { atMs: 0, value: 150 }, { atMs: 2000, value: -20 },
  ] }] },
  [{ kind: "servo", boardId: 2, channel: 2, label: "B2.S2", min: 0, max: 100, unit: "%" }],
);
approx("over-max value clamps y to 100", overshoot[0].points[0].yPct, 100);
approx("late atMs clamps x to 100", overshoot[0].points[1].xPct, 100);
approx("under-min value clamps y to 0", overshoot[0].points[1].yPct, 0);

// ---- defensive: junk inputs never throw, return [] ------------------------
eq("null motion → []", motionPreviewStrips(null, [SERVO]), []);
eq("no specs → []", motionPreviewStrips(motion, []), []);
eq("missing tracks → []", motionPreviewStrips({ durationMs: 1000 }, [SERVO]), []);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
