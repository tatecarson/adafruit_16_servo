// Tests for the Motion-editor keyframe selection/deletion/move helpers.
// Pure logic lives in the EDITOR-CORE block of servo_controller.html:
//   motionConnectionPoints, motionDragReadout, keyframesInMarquee,
//   deleteKeyframeIndices, moveSelectedKeyframes
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

// The value-snapping helpers live in their own marker block because SHAPE-CORE
// calls into them too (servo-3o6); both suites prepend it to what they extract.
const SNAP_START = "// === SNAP-CORE START ===";
const SNAP_END = "// === SNAP-CORE END ===";
const ss = html.indexOf(SNAP_START);
const se = html.indexOf(SNAP_END, ss + SNAP_START.length);
if (ss < 0 || se <= ss) { fail("SNAP-CORE markers not found or out of order"); process.exit(1); }
const snapCore = html.slice(ss + SNAP_START.length, se);

const dir = mkdtempSync(join(tmpdir(), "editor-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, snapCore + core + "\nexport { motionConnectionPoints, motionDragReadout, keyframesInMarquee, deleteKeyframeIndices, moveSelectedKeyframes, snapMotionValue, snapMotionValueToward };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
for (const fn of ["motionConnectionPoints", "motionDragReadout", "keyframesInMarquee", "deleteKeyframeIndices", "moveSelectedKeyframes", "snapMotionValue", "snapMotionValueToward"]) {
  if (typeof mod[fn] !== "function") fail(`core blocks do not export ${fn}()`);
}
if (failed) process.exit(1);
const { motionConnectionPoints, motionDragReadout, keyframesInMarquee, deleteKeyframeIndices, moveSelectedKeyframes, snapMotionValue, snapMotionValueToward } = mod;

console.log("=== Motion editor keyframe select/delete ===");

// ---- authored DC safety range, now on the Sequencer lanes (servo-i0v) -----
eq("the DC safety limit is declared for the Sequencer lane",
  /const DC_LANE_SPEED_LIMIT\s*=\s*50\s*;/.test(html), true);
eq("Motions no longer declare DC tracks",
  /kind:\s*"dc"/.test(html), false);
eq("the lane inputs are bounded by the safety limit",
  /min="\$\{-DC_LANE_SPEED_LIMIT\}"\s+max="\$\{DC_LANE_SPEED_LIMIT\}"/.test(html), true);
eq("the Motion UI points operators at the Sequencer lanes",
  /motor speed moved to Sequencer DC lanes/.test(html), true);

const kfs = [{ atMs: 0, value: 0 }, { atMs: 1000, value: 50 }, { atMs: 2000, value: 100 }, { atMs: 3000, value: 20 }];

// ---- editor connection geometry -------------------------------------------
eq("servo connection is linear and inset", motionConnectionPoints([
  { atMs: 0, value: 0 }, { atMs: 1000, value: 50 }, { atMs: 2000, value: 100 },
], 2000, { kind: "servo", minValue: 0, maxValue: 100, axisInsetPct: 15 }), [
  { xPct: 0, yPct: 15 }, { xPct: 50, yPct: 50 }, { xPct: 100, yPct: 85 },
]);
eq("DC connection renders held-speed steps", motionConnectionPoints([
  { atMs: 0, value: -100 }, { atMs: 1000, value: 0 }, { atMs: 2000, value: 100 },
], 2000, { kind: "dc", minValue: -100, maxValue: 100, axisInsetPct: 15 }), [
  { xPct: 0, yPct: 85 },
  { xPct: 50, yPct: 85 }, { xPct: 50, yPct: 50 },
  { xPct: 100, yPct: 50 }, { xPct: 100, yPct: 15 },
]);
eq("connection clamps out-of-range points", motionConnectionPoints([
  { atMs: -100, value: -20 }, { atMs: 3000, value: 120 },
], 2000, { kind: "servo", minValue: 0, maxValue: 100 }), [
  { xPct: 0, yPct: 15 }, { xPct: 100, yPct: 85 },
]);
eq("servo drag readout reports value and vertical position", motionDragReadout(42.5, {
  kind: "servo", minValue: 0, maxValue: 100, unit: "%", atMs: 1500, durationMs: 6000, axisInsetPct: 15,
}), { value: 42.5, text: "42.5%", leftPct: 25, topPct: 44.75 });
eq("DC drag readout reports signed speed", motionDragReadout(-40, {
  kind: "dc", minValue: -100, maxValue: 100, unit: "spd", atMs: 9000, durationMs: 6000, axisInsetPct: 15,
}), { value: -40, text: "-40 spd", leftPct: 100, topPct: 64 });
eq("drag readout clamps before display", motionDragReadout(140, {
  kind: "servo", minValue: 0, maxValue: 100, unit: "%",
}), { value: 100, text: "100%", leftPct: null, topPct: 85 });

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
  // The feasible boundary here is +12; the 5-grid pulls the shift back to +10
  // (servo-3o6). Snapping only ever rounds toward zero, so the result stays
  // inside what the servo can reach — it just stops short of the limit.
  eq("servo move clamps value to feasible boundary, then to the grid", moved.keyframes, [
    { atMs: 0, value: 0 },
    { atMs: 1000, value: 10 },
    { atMs: 2000, value: 20 },
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

// --- servo-3o6: keyframe values snap to a 5-grid -----------------------------
// The point is alignment: one Motion's exit landing exactly on the next one's
// entry drives the interior-bridge delta to 0, so no pre-roll is planned.
{
  const SERVO = { kind: "servo", min: 0, max: 100 };
  eq("47 rounds down to the grid", snapMotionValue(47, false), 45);
  eq("43 rounds up to the grid", snapMotionValue(43, false), 45);
  eq("42.5 rounds up on the half", snapMotionValue(42.5, false), 45);
  eq("2.4 collapses to 0", snapMotionValue(2.4, false), 0);
  eq("98 rounds to the top of travel", snapMotionValue(98, false), 100);
  eq("a value already on the grid is untouched", snapMotionValue(40, false), 40);
  eq("Shift keeps the old fine mode", snapMotionValue(42.53, true), 42.5);

  // Feasibility clamps report the furthest a servo can actually reach in the
  // time available. Rounding that AWAY from where it started would hand back a
  // move the hardware cannot make, so the grid only ever pulls it back.
  eq("a clamp moving up rounds down toward its start",
     snapMotionValueToward(42.3, 0, SERVO), 40);
  eq("a clamp moving down rounds up toward its start",
     snapMotionValueToward(57.7, 100, SERVO), 60);
  eq("a reachable grid value is left alone",
     snapMotionValueToward(45, 0, SERVO), 45);
  // An off-grid anchor (legacy data) must not be overshot: feasibility wins.
  eq("snapping never crosses the anchor it is pulling toward",
     snapMotionValueToward(1, 4, SERVO), 4);
  eq("no movement stays no movement",
     snapMotionValueToward(30, 30, SERVO), 30);
  eq("the result stays inside the track's travel limits",
     snapMotionValueToward(-3, 0, SERVO), 0);

  // Dragging a selection is the other way a boundary value gets set. The drag
  // delta is already a grid multiple, but the feasibility clamp can cut it to
  // anything — and a group drag that lands the last keyframe on 28 is exactly
  // the off-grid exit that brings a pre-roll back.
  const group = [
    { atMs: 0, value: 0 }, { atMs: 4000, value: 40 }, { atMs: 8000, value: 40 },
  ];
  const dragged = moveSelectedKeyframes(group, [1, 2], 0, 50, {
    durationMs: 8000, minValue: 0, maxValue: 100, kind: "servo", msPerPct: 77,
  });
  eq("a clamped group drag still lands every servo value on the grid",
     dragged.keyframes.every(k => k.value % 5 === 0), true);
}

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
