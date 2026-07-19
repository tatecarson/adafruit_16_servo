import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const match = html.match(/\/\/ === BAKED-PREROLL-CORE START ===([\s\S]*?)\/\/ === BAKED-PREROLL-CORE END ===/);
if (!match) throw new Error("BAKED-PREROLL-CORE block not found");

const context = {};
vm.createContext(context);
vm.runInContext(`${match[1]}\nthis.plan = planBakedMotionPreRoll;`, context);

const specs = [1, 2, 3].flatMap(boardId => [
  { kind: "servo", boardId, channel: 0 },
  { kind: "servo", boardId, channel: 1 },
  { kind: "servo", boardId, channel: 2 },
  { kind: "dc", boardId, channel: 0 },
]);
const track = (kind, boardId, channel, values) => ({
  kind, boardId, channel,
  keyframes: values.map(([atMs, value]) => ({ atMs, value })),
});
const motion = (id, tracks) => ({ id, tracks });

let passed = 0;
function check(condition, message) {
  if (!condition) throw new Error(`FAIL: ${message}`);
  passed++;
  console.log(`PASS: ${message}`);
}

const firstRun = context.plan(null, motion("next", []), specs);
check(firstRun.durationMs === 0 && firstRun.moves.length === 0,
  "first baked play starts immediately when no prior pose is known");
check(/motionMatch[\s\S]*sendBakedMotionWithPreRoll\(ip, motionMatch\[1\]\)/.test(html),
  "the shared command sender routes every MOTION source through pre-roll");
check(/STOP[\s\S]*bakedMotionPreRollSerial\+\+/.test(html),
  "STOP invalidates a delayed baked Motion start");

const previous = motion("arc", [
  track("servo", 1, 0, [[0, 0], [8000, 100]]),
  track("servo", 2, 1, [[0, 20], [8000, 80]]),
  track("dc", 1, 0, [[0, 0], [8000, 45]]),
]);
const replay = motion("arc", [
  track("servo", 1, 0, [[0, 0], [8000, 100]]),
  track("servo", 2, 1, [[0, 20], [8000, 80]]),
  track("dc", 1, 0, [[0, 0], [8000, 45]]),
]);
const replayPlan = context.plan(previous, replay, specs);
check(replayPlan.durationMs === 7700, "full-range replay gets the measured 7.7s warm-up");
check(replayPlan.moves.length === 2, "all changed servo tracks are prepared");
check(replayPlan.moves.every(move => move.durationMs === 7700),
  "every board shares the cluster-wide maximum warm-up duration");
check(replayPlan.moves.some(move => move.boardId === 1 && move.channel === 0 && move.toValue === 0),
  "board 1 rewinds to its first keyframe");
check(replayPlan.moves.some(move => move.boardId === 2 && move.channel === 1 && move.toValue === 20),
  "board 2 rewinds to its first keyframe");
check(JSON.stringify(replayPlan.dcStopBoardIds) === "[1]",
  "a prior nonzero DC endpoint is stopped during servo warm-up");

const settled = motion("hold", [track("servo", 1, 0, [[0, 50], [8000, 50]])]);
const settledPlan = context.plan(settled, settled, specs);
check(settledPlan.durationMs === 0 && settledPlan.moves.length === 0,
  "a Motion already at its start pose is not delayed");

const tinyPrevious = motion("tiny", [track("servo", 1, 0, [[0, 10], [1000, 13]])]);
const tinyNext = motion("tiny", [track("servo", 1, 0, [[0, 10], [1000, 13]])]);
check(context.plan(tinyPrevious, tinyNext, specs).durationMs === 0,
  "sub-five-percent drift does not add a perceptible warm-up");

console.log(`\n${passed}/11 baked Motion pre-roll checks passed.`);
