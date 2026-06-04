// Tests for the Motion-editor preset shape curves (servo-rig). The pure logic
// lives in the SHAPE-CORE block of servo_controller.html:
//   shapeValue, generateShapeKeyframes, mergeShapeIntoKeyframes
// This extracts that block, loads it as a module, and asserts shape math,
// grid/point sampling, servo feasibility (amplitude reduction + square
// trapezoid), and region-merge behavior.
//
// Run: node verify_shape_curves.mjs   (or via the make test node run)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === SHAPE-CORE START ===";
const END = "// === SHAPE-CORE END ===";

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}
function approx(n, got, want, tol = 1e-9) {
  if (Math.abs(got - want) <= tol) ok(n);
  else fail(`${n}: expected ~${want}, got ${got}`);
}
function assert(n, cond) { cond ? ok(n) : fail(n); }

const s = html.indexOf(START);
const e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("SHAPE-CORE markers not found or out of order"); process.exit(1); }
const core = html.slice(s + START.length, e);

const dir = mkdtempSync(join(tmpdir(), "shape-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { shapeValue, generateShapeKeyframes, mergeShapeIntoKeyframes, fitKeyframesToNeighbors };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
for (const fn of ["shapeValue", "generateShapeKeyframes", "mergeShapeIntoKeyframes", "fitKeyframesToNeighbors"]) {
  if (typeof mod[fn] !== "function") fail(`SHAPE-CORE does not export ${fn}()`);
}
if (failed) process.exit(1);
const { shapeValue, generateShapeKeyframes, mergeShapeIntoKeyframes, fitKeyframesToNeighbors } = mod;

const MS_PER_PCT = 77;
function feasible(kfs) {
  for (let i = 1; i < kfs.length; i++) {
    const dt = kfs[i].atMs - kfs[i - 1].atMs;
    const dv = Math.abs(kfs[i].value - kfs[i - 1].value);
    if (dv * MS_PER_PCT > dt + 1e-6) return false;
  }
  return true;
}
const SERVO = { kind: "servo", min: 0, max: 100 };
const DC = { kind: "dc", min: -100, max: 100 };

console.log("=== Motion shape curves ===");

// ---- shapeValue: normalized curve math -------------------------------------
approx("rampUp(0)=0", shapeValue("rampUp", 0), 0);
approx("rampUp(1)=1", shapeValue("rampUp", 1), 1);
approx("rampDown(0)=1", shapeValue("rampDown", 0), 1);
approx("sine(0)=0", shapeValue("sine", 0), 0);
approx("sine(0.5)=1", shapeValue("sine", 0.5), 1, 1e-9);
approx("sine(1)=0", shapeValue("sine", 1), 0, 1e-9);
approx("triangle(0.5)=1", shapeValue("triangle", 0.5), 1);
approx("triangle(0)=0", shapeValue("triangle", 0), 0);
eq("square(0.25)=0", shapeValue("square", 0.25), 0);
eq("square(0.75)=1", shapeValue("square", 0.75), 1);
approx("easeIn(0.5)=0.25", shapeValue("easeIn", 0.5), 0.25);
approx("easeOut(0.5)=0.75", shapeValue("easeOut", 0.5), 0.75);

// ---- generateShapeKeyframes: DC ramp (no feasibility limit) ------------------
{
  const { keyframes, reducedTo } = generateShapeKeyframes(
    { shape: "rampUp", t0: 0, t1: 1000, low: -100, high: 100, spec: DC, msPerPct: MS_PER_PCT });
  eq("dc ramp: 11 points", keyframes.length, 11);
  eq("dc ramp: starts low", keyframes[0], { atMs: 0, value: -100 });
  eq("dc ramp: ends high", keyframes[keyframes.length - 1], { atMs: 1000, value: 100 });
  eq("dc ramp: not reduced", reducedTo, 1);
  assert("dc ramp: atMs on 100ms grid", keyframes.every(k => k.atMs % 100 === 0));
}

// ---- point-count clamp (3..24) ---------------------------------------------
{
  const wide = generateShapeKeyframes({ shape: "sine", t0: 0, t1: 5000, low: 0, high: 5, spec: SERVO, msPerPct: MS_PER_PCT });
  assert("wide box capped at 24 points", wide.keyframes.length <= 24);
  // Smallest box that spans 3 grid slots (0,100,200) → at least 3 points.
  const small = generateShapeKeyframes({ shape: "rampUp", t0: 0, t1: 200, low: 0, high: 2, spec: SERVO, msPerPct: MS_PER_PCT });
  assert("200ms box has >=3 points", small.keyframes.length >= 3);
  // A 100ms box only has two grid slots, so it can't hold 3 grid-aligned points.
  const tiny = generateShapeKeyframes({ shape: "rampUp", t0: 0, t1: 100, low: 0, high: 1, spec: SERVO, msPerPct: MS_PER_PCT });
  assert("100ms box snaps to 2 grid points", tiny.keyframes.length === 2);
}

// ---- servo feasibility: steep shape gets amplitude-reduced ------------------
{
  const steep = generateShapeKeyframes({ shape: "sine", t0: 0, t1: 400, low: 0, high: 100, spec: SERVO, msPerPct: MS_PER_PCT });
  assert("steep servo sine is feasible after fit", feasible(steep.keyframes));
  assert("steep servo sine reports reduced amplitude", steep.reducedTo < 1);
}

// ---- servo feasibility: gentle shape NOT reduced ----------------------------
{
  const gentle = generateShapeKeyframes({ shape: "rampUp", t0: 0, t1: 4000, low: 0, high: 10, spec: SERVO, msPerPct: MS_PER_PCT });
  assert("gentle servo ramp is feasible", feasible(gentle.keyframes));
  eq("gentle servo ramp not reduced", gentle.reducedTo, 1);
}

// ---- square: DC clean step, servo feasible trapezoid ------------------------
{
  const dcSq = generateShapeKeyframes({ shape: "square", t0: 0, t1: 1000, low: -100, high: 100, spec: DC, msPerPct: MS_PER_PCT });
  assert("dc square values are only low/high", dcSq.keyframes.every(k => k.value === -100 || k.value === 100));

  // box wide enough for full up+down ramps at 20% amplitude (20*77=1540ms each).
  const servoSq = generateShapeKeyframes({ shape: "square", t0: 0, t1: 6000, low: 0, high: 20, spec: SERVO, msPerPct: MS_PER_PCT });
  assert("servo square trapezoid is feasible", feasible(servoSq.keyframes));
  assert("servo square reaches near high (not collapsed)", Math.max(...servoSq.keyframes.map(k => k.value)) >= 18);
}

// ---- mergeShapeIntoKeyframes: replace in-range, keep outside ----------------
{
  const existing = [{ atMs: 0, value: 0 }, { atMs: 1500, value: 50 }, { atMs: 5000, value: 0 }];
  const generated = [{ atMs: 1000, value: 10 }, { atMs: 2000, value: 90 }];
  const merged = mergeShapeIntoKeyframes(existing, generated, 1000, 2000);
  const ats = merged.map(k => k.atMs);
  assert("merge keeps out-of-range keyframes", ats.includes(0) && ats.includes(5000));
  assert("merge drops in-range existing keyframe", !ats.includes(1500));
  assert("merge includes generated", ats.includes(1000) && ats.includes(2000));
  assert("merge is sorted by atMs", ats.every((v, i) => i === 0 || v >= ats[i - 1]));
}

function chainFeasible(seq) {
  for (let i = 1; i < seq.length; i++) {
    const dt = seq[i].atMs - seq[i - 1].atMs;
    const dv = Math.abs(seq[i].value - seq[i - 1].value);
    if (dv * MS_PER_PCT > dt + 1e-6) return false;
  }
  return true;
}

// ---- fitKeyframesToNeighbors: boundary feasibility (the reported bug) -------
{
  // The exact bug: a track starts at 0@0; shape jumps to 52 by 1200ms.
  const prev = { atMs: 0, value: 0 };
  const pts = [{ atMs: 1200, value: 52 }, { atMs: 1300, value: 53 }, { atMs: 2000, value: 60 }];
  const fitted = fitKeyframesToNeighbors(pts, prev, null, MS_PER_PCT);
  assert("fit makes prev→shape feasible", chainFeasible([prev, ...fitted]));
  assert("first point reachable from prev (≤15 in 1200ms)", fitted[0].value <= Math.floor(1200 / MS_PER_PCT));
}
{
  // Trailing anchor: shape must be able to reach a kept keyframe after it.
  const next = { atMs: 5000, value: 0 };
  const pts = [{ atMs: 3000, value: 60 }, { atMs: 4900, value: 62 }];
  const fitted = fitKeyframesToNeighbors(pts, null, next, MS_PER_PCT);
  assert("fit makes shape→next feasible", chainFeasible([...fitted, next]));
}
{
  // Both anchors (mutually feasible, as they are in a previously-valid track).
  const prev = { atMs: 0, value: 0 }, next = { atMs: 8000, value: 0 };
  const pts = [{ atMs: 2000, value: 100 }, { atMs: 4000, value: 0 }, { atMs: 6000, value: 100 }];
  const fitted = fitKeyframesToNeighbors(pts, prev, next, MS_PER_PCT);
  assert("fit makes prev→shape→next feasible", chainFeasible([prev, ...fitted, next]));
}
{
  // No neighbors → unchanged (still feasible if input was).
  const pts = [{ atMs: 0, value: 0 }, { atMs: 1000, value: 10 }];
  eq("no neighbors leaves points unchanged", fitKeyframesToNeighbors(pts, null, null, MS_PER_PCT), pts);
}

// ---- end-to-end: generate + fit + merge yields a fully feasible servo track -
{
  const existing = [{ atMs: 0, value: 0 }, { atMs: 8000, value: 0 }];
  const gen = generateShapeKeyframes({ shape: "sine", t0: 1000, t1: 3000, low: 0, high: 100, spec: SERVO, msPerPct: MS_PER_PCT }).keyframes;
  const prev = { atMs: 0, value: 0 };          // nearest kept before box
  const next = { atMs: 8000, value: 0 };       // nearest kept after box
  const fitted = fitKeyframesToNeighbors(gen, prev, next, MS_PER_PCT);
  const merged = mergeShapeIntoKeyframes(existing, fitted, 1000, 3000);
  assert("merged servo track is fully feasible", chainFeasible(merged));
}

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
