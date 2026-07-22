// Regression checks for per-step DC lanes (servo-i0v). DC speed is authored as
// an optional { boardId: speed } map on each Sequence step; flattenDcLanes()
// turns that into the targeted zero-duration ROTATE chord steps the firmware
// actually runs. Extracts the pure core from servo_controller.html so the
// contract is checked without a browser.

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

const dir = mkdtempSync(join(tmpdir(), "dc-lane-core-"));
const modulePath = join(dir, "core.mjs");
writeFileSync(modulePath, `
const SEQ_MAX_STEPS = 16;
${block("// === DC-LANE-CORE START ===", "// === DC-LANE-CORE END ===")}
export { flattenDcLanes, clampDcLaneValue, readStepDcMap, dcStateByDisplayBlock, DC_LANE_SPEED_LIMIT, DC_LANE_BOARD_IDS };
`, "utf8");

const core = await import(pathToFileURL(modulePath).href);
let passed = 0;
function check(condition, message) {
  if (!condition) throw new Error(`FAIL: ${message}`);
  passed++;
  console.log(`PASS: ${message}`);
}

const step = (cmd, durationMs, dc) => ({ cmd, durationMs, target: "all", label: "", hold: false, ...(dc ? { dc } : {}) });
const chords = out => out.steps.filter(s => s.dcChord).map(s => `${s.cmd}@${s.target}`);

// --- sticky carry ------------------------------------------------------
{
  const out = core.flattenDcLanes([
    step("MOTION a", 1000, { 1: 50 }),
    step("MOTION b", 1000),
    step("MOTION c", 1000),
  ], { loop: true });
  check(chords(out).length === 1, "a board absent from later steps holds its speed (one chord, not three)");
  check(chords(out)[0] === "ROTATE 50@1", "the chord is targeted at the authoring board");
}

// --- blank means unchanged, 0 means stop -------------------------------
{
  const out = core.flattenDcLanes([
    step("MOTION a", 1000, { 1: 50 }),
    step("MOTION b", 1000, { 1: null }),
    step("MOTION c", 1000, { 1: 0 }),
  ], { loop: true });
  check(chords(out).join(",") === "ROTATE 50@1,ROTATE 0@1",
    "null is 'unchanged' while 0 is an explicit stop");
}

// --- redundant values are not re-sent ----------------------------------
{
  const out = core.flattenDcLanes([
    step("MOTION a", 1000, { 1: 50 }),
    step("MOTION b", 1000, { 1: 50 }),
  ], { loop: true });
  check(chords(out).length === 1, "re-stating the current speed emits no chord");
}

// --- STOP zeroes the motors, so tracked state must reset ---------------
{
  const out = core.flattenDcLanes([
    step("MOTION a", 1000, { 1: 50 }),
    step("STOP", 500),
    step("MOTION b", 1000, { 1: 50 }),
  ], { loop: true });
  check(chords(out).join(",") === "ROTATE 50@1,ROTATE 50@1",
    "a STOP step resets tracked state so the next identical value is re-sent");
}

// --- end of a non-looping sequence parks running motors ----------------
{
  const out = core.flattenDcLanes([
    step("MOTION a", 1000, { 1: 50, 3: -25 }),
  ], { loop: false });
  check(chords(out).slice(-2).sort().join(",") === "ROTATE 0@1,ROTATE 0@3",
    "non-looping sequence emits a trailing zero for every running board");
  const last = out.steps[out.steps.length - 1];
  check(last.dcChord && last.durationMs === 0, "the trailing zero is a zero-duration chord");
}
{
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: 50 })], { loop: true });
  check(!chords(out).includes("ROTATE 0@1"), "a looping sequence does not park the motor at the wrap");
}
{
  const out = core.flattenDcLanes([
    step("MOTION a", 1000, { 1: 50 }),
    step("STOP", 100),
  ], { loop: false });
  check(chords(out).filter(c => c === "ROTATE 0@1").length === 0,
    "no trailing zero when a STOP already parked the motor");
}

// --- per-board targeting ------------------------------------------------
{
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: -50, 3: 25 })], { loop: true });
  check(chords(out).join(",") === "ROTATE -50@1,ROTATE 25@3",
    "each board gets its own chord targeted at that board only");
}

// --- clamping -----------------------------------------------------------
{
  check(core.clampDcLaneValue(999) === core.DC_LANE_SPEED_LIMIT, "lane clamps above the speed limit");
  check(core.clampDcLaneValue(-999) === -core.DC_LANE_SPEED_LIMIT, "lane clamps below the speed limit");
  check(core.DC_LANE_SPEED_LIMIT === 50, "lane speed limit stays at the servo-1gc safety value");
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: 100 })], { loop: true });
  check(chords(out)[0] === "ROTATE 50@1", "an out-of-range authored value is clamped before it bakes");
}

// --- the authoring field never reaches the device ----------------------
{
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: 50 })], { loop: true });
  check(out.steps.every(s => s.dc === undefined),
    "the authoring-only `dc` field is stripped from every emitted step");
}

// --- chords are free on the clock --------------------------------------
{
  const authored = [step("MOTION a", 1000, { 1: 50 }), step("MOTION b", 2000, { 1: -50 })];
  const out = core.flattenDcLanes(authored, { loop: true });
  const total = out.steps.reduce((n, s) => n + s.durationMs, 0);
  check(total === 3000, "zero-duration chords do not change the sequence's total time");
}

// --- lane timeline for rendering ---------------------------------------
{
  const out = core.flattenDcLanes([
    step("MOTION a", 1000, { 1: 50 }),
    step("MOTION b", 2000, { 1: -50 }),
  ], { loop: true });
  const b1 = out.lane.filter(p => p.boardId === 1);
  check(b1.length === 2 && b1[0].atMs === 0 && b1[1].atMs === 1000,
    "the lane timeline records each change at its authored wall-clock time");
}

// --- budget ------------------------------------------------------------
{
  const many = [];
  for (let i = 0; i < 10; i++) many.push(step(`MOTION m${i}`, 100, { 1: i % 2 ? 50 : -50 }));
  const out = core.flattenDcLanes(many, { loop: true, maxSteps: 16 });
  check(out.steps.length === 20, "ten alternating steps flatten to twenty");
  check(out.error && out.error.code === "dc-step-budget" && out.error.produced === 20,
    "overflowing the firmware step budget is reported, not silently truncated");
}
{
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: 50 })], { loop: true, maxSteps: 16 });
  check(!out.error, "a sequence inside the budget reports no error");
}

// --- readStepDcMap normalization ---------------------------------------
{
  check(JSON.stringify(core.readStepDcMap({ dc: { 1: "50", 2: "", 3: null } })) === JSON.stringify({ 1: 50 }),
    "readStepDcMap drops blank entries and coerces numeric strings");
  check(JSON.stringify(core.readStepDcMap({})) === "{}", "a step with no dc field normalizes to an empty map");
}

// --- device target must be numeric --------------------------------------
// The firmware parses `target` with bakeParseInteger, which fails on a quoted
// "2" and falls back to 255 ("no board"). A string target silently disables
// the step on every board, which is how DC chords first failed to fire.
{
  const compact = /function compactStepForDevice\(step\)[\s\S]*?\n\}/.exec(html)[0];
  check(/out\.target\s*=\s*Number\(target\)/.test(compact),
    "compactStepForDevice bakes target as a number, not the select's string");
}

// --- emitAll: previewing one step out of sequence ----------------------
{
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: 0 })], { loop: true, emitAll: true });
  check(chords(out).join(",") === "ROTATE 0@1",
    "emitAll sends an explicit 0 that the sticky diff would otherwise suppress");
}
{
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: 30, 3: -20 })], { loop: true, emitAll: true });
  check(chords(out).join(",") === "ROTATE 30@1,ROTATE -20@3", "emitAll sends every speed named on the step");
  check(out.steps[out.steps.length - 1].cmd === "MOTION a", "the step's own cmd still fires last");
}
{
  const out = core.flattenDcLanes([step("MOTION a", 1000, { 1: 30 })], { loop: false, emitAll: true });
  check(chords(out).length === 1, "emitAll does not append a trailing park (the step is not a whole sequence)");
}

// --- one flattener, THREE consumers ------------------------------------
// Bake, live playback, and per-step Preview must agree. Divergence between any
// of these is the bug class this feature was built to retire. Preview was
// missed on the first pass and silently ignored the lane.
{
  const bakeCall = /function buildBakeLibrary[\s\S]{0,2000}?flattenDcLanes\(/.test(html);
  const liveCall = /function playSequenceFrom[\s\S]{0,2000}?flattenDcLanes\(/.test(html);
  const previewCall = /dataset\.preview[\s\S]{0,1200}?flattenDcLanes\(/.test(html);
  check(bakeCall, "buildBakeLibrary flattens DC lanes");
  check(liveCall, "the browser live sequence player flattens DC lanes through the same function");
  check(previewCall, "the per-step Preview button flattens DC lanes through the same function");
}

// --- firmware invariants the lanes depend on ----------------------------
// Both of these were guards written for DC-inside-Motions that turn actively
// harmful once DC moves to lanes. Source assertions because the C++ host suite
// does not link command_interface.h.
{
  const cmdIface = readFileSync(join(root, "adafruit_16_servo/command_interface.h"), "utf8");
  const rotate = /startsWith\(cmd, "ROTATE"\)[\s\S]*?\n  \}/.exec(cmdIface)[0];
  check(!/cancelMotionPlayback\(\)/.test(rotate),
    "ROTATE does not cancel Motion playback (a lane chord must not abort its own Motion)");
  check(/cancelSequencePlayback\(\)/.test(rotate),
    "ROTATE still cancels a running Sequence so a manual override takes over");

  const motionEngine = readFileSync(join(root, "adafruit_16_servo/motion_engine.h"), "utf8");
  const prepared = /inline bool startMotionPreparedFromStorage[\s\S]*?\n\}/.exec(motionEngine)[0];
  check(!/setMotorSpeedQuiet\(0\)/.test(prepared),
    "the Motion pre-roll leaves motor state alone (zeroing here wipes the DC chord)");
}

// --- servo-d5y: a chord must land on the Motion, not on the glide ------
// rewriteSequencePreRolls folds the travel glide INSIDE the step
// (MOTION x PREP n, durationMs d+n). A chord can only fire at a step
// boundary, so without splitting it lands at the START of the glide and the
// motor changes speed seconds before the Motion it was authored against.
{
  const out = core.flattenDcLanes([
    step("MOTION rise", 6000, { 3: 20 }),
    { ...step("MOTION ripple PREP 3465", 9465), dc: { 3: -30 } },
  ], { loop: false });
  const timeline = [];
  let at = 0;
  for (const s of out.steps) { timeline.push([at, s.cmd]); at += Math.max(0, s.durationMs | 0); }
  check(JSON.stringify(timeline) === JSON.stringify([
    [0, "ROTATE 20"],
    [0, "MOTION rise"],
    [6000, "MOTION ripple PREP 3465"],   // the glide, on its own clock
    [9465, "ROTATE -30"],                 // fires exactly as Ripple opens
    [9465, ""],                           // dwell holds Ripple's real length
    [15465, "ROTATE 0"],                  // trailing park
  ]), "a chord on a PREP step fires at the Motion start, not at the glide start");

  const dwell = out.steps.find(s => s.cmd === "");
  check(dwell.durationMs === 6000, "the dwell step carries the Motion's real duration");
  check(dwell.target === "all", "the dwell step is cluster-wide so every board holds the clock");
  check(!dwell.dcChord, "the dwell step is not mistaken for a chord");
}

// Splitting costs a firmware step, so it must only happen when it buys
// something: no DC change on the step means the compact form is kept.
{
  const out = core.flattenDcLanes([
    { ...step("MOTION ripple PREP 3465", 9465) },
  ], { loop: false });
  check(out.steps.length === 1, "a PREP step with no DC change is left compact");
  check(out.steps[0].durationMs === 9465, "and keeps its combined duration");
}

// A DC change that restates the current speed emits no chord, so there is
// nothing to re-time and no reason to spend a step splitting.
{
  const out = core.flattenDcLanes([
    step("MOTION rise", 6000, { 3: 20 }),
    { ...step("MOTION ripple PREP 3465", 9465), dc: { 3: 20 } },
  ], { loop: false });
  check(!out.steps.some(s => s.cmd === ""), "a redundant DC value does not split the step");
}

// The split must survive the step-budget check: it produces more steps, so
// the overflow has to be reported rather than silently baked.
{
  const many = [];
  for (let i = 0; i < 6; i++) many.push({ ...step(`MOTION m${i} PREP 800`, 1800), dc: { 3: i % 2 ? 10 : 20 } });
  const out = core.flattenDcLanes(many, { loop: false, maxSteps: 16 });
  check(out.error && out.error.code === "dc-step-budget",
    "splitting counts against the firmware step budget");
}

// The Arrange lane and the bake must agree. This is the assertion that would
// have caught servo-d5y: draw the change on the travel block while the chord
// fires there too and both are "consistent" — but the Motion has not started.
{
  // What Arrange draws, given the display expansion of a glide + Motion.
  const display = [
    step("MOTION rise", 6000, { 3: 20 }),
    { cmd: "", durationMs: 3465, target: "all", bridge: true },   // travel block
    { ...step("MOTION ripple", 6000), dc: { 3: -30 } },
  ];
  const states = core.dcStateByDisplayBlock(display);
  let at = 0; const drawn = [];
  display.forEach((s, i) => { drawn.push([at, states[i][3]]); at += s.durationMs; });
  check(JSON.stringify(drawn) === JSON.stringify([[0, 20], [6000, 20], [9465, -30]]),
    "the lane holds the old speed across the glide and changes at the Motion");

  // What the rig does, from the same authored steps.
  const baked = core.flattenDcLanes([
    step("MOTION rise", 6000, { 3: 20 }),
    { ...step("MOTION ripple PREP 3465", 9465), dc: { 3: -30 } },
  ], { loop: true });
  let bt = 0; const fired = [];
  for (const s of baked.steps) {
    if (/^ROTATE/.test(s.cmd)) fired.push([bt, parseInt(s.cmd.split(" ")[1], 10)]);
    bt += Math.max(0, s.durationMs | 0);
  }
  check(JSON.stringify(fired) === JSON.stringify([[0, 20], [9465, -30]]),
    "and the baked chords fire at exactly those same times");
}

// --- DC is gone from the Motion editor ---------------------------------
{
  check(!/\{\s*kind:\s*"dc"/.test(html), "MOTION_TRACK_SPECS no longer defines DC tracks");
  check(!html.includes("analyzeMotionDcLiveFeasibility"), "the DC live-feasibility analyzer is deleted");
  check(!html.includes("DC_LIVE_MIN_SPACING_MS"), "the DC live throughput throttle is deleted");
}

console.log(`\n${passed} checks passed`);
