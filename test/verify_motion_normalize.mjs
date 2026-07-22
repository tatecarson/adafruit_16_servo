// Host tests for Motion normalization (servo-b4r).
//
// normalizeMotion materializes all nine MOTION_TRACK_SPECS tracks for every
// Motion, so what normalizeKeyframes synthesizes for a channel the author
// never touched is a physical command, not a formality. On this rig 0% is
// FULLY UP and the unpowered rest pose is 100% (fully down), so synthesizing
// 0 turns every un-authored channel into a full-travel raise — and the
// cold-start planner then correctly budgets seconds of pre-roll for it.
//
// Run: node verify_motion_normalize.mjs   (or via the make test node run)

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

// SPEC-CORE holds the track list and the rest-pose constant; SNAP-CORE holds
// clampMotionValue, which normalizeKeyframes uses to bound every value.
const core = block("// === MOTION-SPEC-CORE START ===", "// === MOTION-SPEC-CORE END ===")
  + block("// === SNAP-CORE START ===", "// === SNAP-CORE END ===")
  + block("// === MOTION-NORMALIZE-CORE START ===", "// === MOTION-NORMALIZE-CORE END ===");

const dir = mkdtempSync(join(tmpdir(), "motion-normalize-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { normalizeKeyframes, normalizeMotion, defaultTrackValue, MOTION_SERVO_REST_PERCENT, MOTION_TRACK_SPECS };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
for (const fn of ["normalizeKeyframes", "normalizeMotion", "defaultTrackValue"]) {
  if (typeof mod[fn] !== "function") fail(`core blocks do not export ${fn}()`);
}
if (failed) process.exit(1);
const { normalizeKeyframes, normalizeMotion, defaultTrackValue, MOTION_SERVO_REST_PERCENT, MOTION_TRACK_SPECS } = mod;

console.log("=== Motion normalization ===");

const SERVO = { kind: "servo", boardId: 1, channel: 0, label: "B1.S0", min: 0, max: 100, unit: "%" };

// The rest pose is a physical fact about the rig, not a preference: unpowered,
// the ring's weight pulls every winch to the bottom of its travel.
eq("rest pose is fully down", MOTION_SERVO_REST_PERCENT, 100);
eq("an undefined track sits at rest", defaultTrackValue(SERVO), MOTION_SERVO_REST_PERCENT);

// --- servo-b4r: a channel nobody authored must not command a move ----------
eq("an absent track synthesizes a hold at rest, not a full-travel raise",
   normalizeKeyframes(undefined, 6000, SERVO), [{ atMs: 0, value: 100 }]);
eq("a track with an empty keyframe list does the same",
   normalizeKeyframes({ keyframes: [] }, 6000, SERVO), [{ atMs: 0, value: 100 }]);

// A Motion authored on one board only. The other six channels are the ones
// that were silently commanding a 100% raise.
const boardThreeOnly = {
  id: "mt-rise01", name: "Rise", scope: "cluster", tags: [], durationMs: 6000,
  tracks: [
    { kind: "servo", boardId: 3, channel: 0, keyframes: [{ atMs: 0, value: 100 }, { atMs: 6000, value: 40 }] },
    { kind: "servo", boardId: 3, channel: 1, keyframes: [{ atMs: 0, value: 100 }, { atMs: 6000, value: 43 }] },
    { kind: "servo", boardId: 3, channel: 2, keyframes: [{ atMs: 0, value: 100 }, { atMs: 6000, value: 46 }] },
  ],
};
const normalized = normalizeMotion(boardThreeOnly);
eq("every spec track is still materialized", normalized.tracks.length, MOTION_TRACK_SPECS.length);
eq("un-authored channels rest instead of raising",
   normalized.tracks.filter(t => t.boardId !== 3).map(t => t.keyframes[0].value),
   [100, 100, 100, 100, 100, 100]);
eq("the authored board is untouched",
   normalized.tracks.filter(t => t.boardId === 3).map(t => t.keyframes.map(k => [k.atMs, k.value])),
   [[[0,100],[6000,40]], [[0,100],[6000,43]], [[0,100],[6000,46]]]);

// Normalizing twice must not drift — the app re-normalizes on every load.
eq("normalization is idempotent",
   JSON.stringify(normalizeMotion(normalized)), JSON.stringify(normalized));

// --- authored tracks keep their own values ---------------------------------
eq("an authored value at t=0 is never replaced by the rest pose",
   normalizeKeyframes({ keyframes: [{ atMs: 0, value: 0 }] }, 6000, SERVO),
   [{ atMs: 0, value: 0 }]);
eq("values are still clamped to the track range",
   normalizeKeyframes({ keyframes: [{ atMs: 0, value: 250 }] }, 6000, SERVO),
   [{ atMs: 0, value: 100 }]);
eq("keyframes past the motion end are pulled back to it",
   normalizeKeyframes({ keyframes: [{ atMs: 0, value: 20 }, { atMs: 99000, value: 60 }] }, 6000, SERVO)
     .map(k => k.atMs), [0, 6000]);
eq("easing survives normalization",
   normalizeKeyframes({ keyframes: [{ atMs: 0, value: 20 }, { atMs: 500, value: 60, easing: "ease-in" }] }, 6000, SERVO)[1].easing,
   "ease-in");

// A track authored only later in the timeline still needs a t=0 boundary. It
// gets the rest pose, which is where the rig actually is before it moves.
eq("a late first keyframe still gets a start boundary at rest",
   normalizeKeyframes({ keyframes: [{ atMs: 3000, value: 60 }] }, 6000, SERVO),
   [{ atMs: 0, value: 100 }, { atMs: 3000, value: 60 }]);

// Legacy degree-scale motions (0..180) still migrate to percent.
eq("legacy degree values migrate to percent",
   normalizeKeyframes({ keyframes: [{ atMs: 0, value: 180 }] }, 6000, SERVO),
   [{ atMs: 0, value: 100 }]);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
