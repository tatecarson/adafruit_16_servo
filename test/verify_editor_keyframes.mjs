// Tests for the Motion-editor keyframe selection/deletion/move helpers.
// Pure logic lives in the EDITOR-CORE block of servo_controller.html:
//   keyframesInMarquee, deleteKeyframeIndices, moveSelectedKeyframes
//
// Run: node verify_editor_keyframes.mjs   (or via the make test node run)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === EDITOR-CORE START ===";
const END = "// === EDITOR-CORE END ===";

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}

const s = html.indexOf(START);
const e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("EDITOR-CORE markers not found or out of order"); process.exit(1); }
const core = html.slice(s + START.length, e);

const dir = mkdtempSync(join(tmpdir(), "editor-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { keyframesInMarquee, deleteKeyframeIndices, moveSelectedKeyframes };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
for (const fn of ["keyframesInMarquee", "deleteKeyframeIndices", "moveSelectedKeyframes"]) {
  if (typeof mod[fn] !== "function") fail(`EDITOR-CORE does not export ${fn}()`);
}
if (failed) process.exit(1);
const { keyframesInMarquee, deleteKeyframeIndices, moveSelectedKeyframes } = mod;

console.log("=== Motion editor keyframe select/delete ===");

const kfs = [{ atMs: 0, value: 0 }, { atMs: 1000, value: 50 }, { atMs: 2000, value: 100 }, { atMs: 3000, value: 20 }];

// ---- marquee selects by time AND value bounds (inclusive) ------------------
eq("time+value box selects middle two", keyframesInMarquee(kfs, 900, 2100, 40, 100), [1, 2]);
eq("full-time, low-value box selects only the 0", keyframesInMarquee(kfs, 0, 3000, 0, 10), [0]);
eq("box matching nothing returns empty", keyframesInMarquee(kfs, 100, 900, 0, 100), []);
eq("inclusive bounds include endpoints", keyframesInMarquee(kfs, 0, 1000, 0, 50), [0, 1]);

// ---- bounds are order-independent (marquee dragged any direction) ----------
eq("reversed bounds normalize", keyframesInMarquee(kfs, 2100, 900, 100, 40), [1, 2]);

// ---- delete by indices ------------------------------------------------------
eq("delete two indices", deleteKeyframeIndices(kfs, [1, 2]), [{ atMs: 0, value: 0 }, { atMs: 3000, value: 20 }]);
eq("delete is order-insensitive", deleteKeyframeIndices(kfs, [2, 1]), [{ atMs: 0, value: 0 }, { atMs: 3000, value: 20 }]);
eq("delete none returns same", deleteKeyframeIndices(kfs, []), kfs);
eq("original array not mutated", kfs.length, 4);

// ---- move selected keyframes ------------------------------------------------
{
  const moved = moveSelectedKeyframes(kfs, [1, 2], 500, -10, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "dc" });
  eq("move translates selected times", moved.selectedAtMs, [1500, 2500]);
  eq("move translates selected values", moved.keyframes, [
    { atMs: 0, value: 0 },
    { atMs: 1500, value: 40 },
    { atMs: 2500, value: 90 },
    { atMs: 3000, value: 20 },
  ]);
}
{
  // The keyframe at 3000 is unmoved; the slid pair must stop 1ms short of it
  // rather than landing on 3000 (which would dedup in normalizeKeyframes).
  const moved = moveSelectedKeyframes(kfs, [1, 2], 2000, 0, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "dc" });
  eq("move stops 1ms before unmoved neighbor", moved.selectedAtMs, [1999, 2999]);
  eq("move reports clamped time", moved.clamped, true);
  eq("move never collides keyframe count", moved.keyframes.length, kfs.length);
}
{
  const servo = [{ atMs: 0, value: 0 }, { atMs: 1000, value: 10 }, { atMs: 2000, value: 20 }, { atMs: 3000, value: 20 }];
  const moved = moveSelectedKeyframes(servo, [1, 2], 0, 30, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "servo", msPerPct: 77 });
  eq("servo move clamps value to feasible boundary", moved.keyframes, [
    { atMs: 0, value: 0 },
    { atMs: 1000, value: 12 },
    { atMs: 2000, value: 22 },
    { atMs: 3000, value: 20 },
  ]);
  eq("servo move reports clamped value", moved.clamped, true);
}
{
  const moved = moveSelectedKeyframes(kfs, [1, 2], 500, 0, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "dc" });
  eq("move does not mutate source array", kfs, [{ atMs: 0, value: 0 }, { atMs: 1000, value: 50 }, { atMs: 2000, value: 100 }, { atMs: 3000, value: 20 }]);
  eq("move keeps unselected keyframes", moved.keyframes[0], { atMs: 0, value: 0 });
}

// ---- pin boundary keyframes during group move -------------------------------
// The start keyframe (atMs 0) and any keyframe at the motion end (atMs ===
// durationMs) are anchored in time. Selecting them must NOT freeze the whole
// group horizontally — the interior keyframes still slide. (servo: can't-move
// left/right regression.)
{
  const moved = moveSelectedKeyframes(kfs, [0, 1, 2, 3], 500, 0, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "dc" });
  eq("select-all pins boundaries, slides interior", moved.selectedAtMs, [0, 1500, 2500, 3000]);
  eq("select-all keyframes", moved.keyframes, [
    { atMs: 0, value: 0 },
    { atMs: 1500, value: 50 },
    { atMs: 2500, value: 100 },
    { atMs: 3000, value: 20 },
  ]);
}
{
  const moved = moveSelectedKeyframes(kfs, [0, 1], -500, 0, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "dc" });
  eq("start keyframe pinned, neighbor slides left", moved.selectedAtMs, [0, 500]);
}
{
  const moved = moveSelectedKeyframes(kfs, [2, 3], 500, 0, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "dc" });
  eq("end keyframe pinned, neighbor slides right", moved.selectedAtMs, [2500, 3000]);
}
{
  // Per-keyframe metadata (e.g. easing) must survive the move on both the
  // moved keyframe and the untouched ones.
  const eased = [{ atMs: 0, value: 0 }, { atMs: 1000, value: 50, easing: "ease-in" }, { atMs: 2000, value: 100, easing: "s-curve" }];
  const moved = moveSelectedKeyframes(eased, [1], 200, 0, { durationMs: 3000, minValue: 0, maxValue: 100, kind: "dc" });
  eq("move preserves moved keyframe easing", moved.keyframes.find(k => k.atMs === 1200)?.easing, "ease-in");
  eq("move preserves untouched keyframe easing", moved.keyframes.find(k => k.atMs === 2000)?.easing, "s-curve");
}

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
