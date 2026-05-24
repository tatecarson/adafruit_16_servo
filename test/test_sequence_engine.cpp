#include "mock/Arduino.h"
#include "mock/EEPROM.h"
#include "mock/Adafruit_PWMServoDriver.h"

#include <stdio.h>
#include <stdexcept>
#include <string.h>
#include <vector>

EEPROMClass EEPROM;

#include "../adafruit_16_servo/storage_crc.h"
#include "../adafruit_16_servo/servo_runtime.h"
#include "../adafruit_16_servo/dc_motor.h"
#include "../adafruit_16_servo/storage.h"

Adafruit_PWMServoDriver pwm;
ServoConfig servoConfig[NUM_SERVOS];
ServoState servoState[NUM_SERVOS];
MotorState motorState;

uint16_t timeMultiplier = 1;
bool sequenceActive = false;
bool sequenceLoop = false;
const Keyframe* currentSequence = nullptr;
uint8_t currentSequenceLength = 0;
unsigned long sequenceStartTime = 0;
uint8_t lastTriggeredKeyframe = 0;
bool speedSeqActive = false;
bool speedSeqLoop = false;
const SpeedFrame* currentSpeedSeq = nullptr;
uint8_t currentSpeedSeqLength = 0;
unsigned long speedSeqStartTime = 0;
uint8_t lastTriggeredSpeedFrame = 0;
bool programActive = false;
bool programLoop = false;
const SequenceProgramDefinition* currentProgram = nullptr;
bool programPositionDone = true;
bool programSpeedDone = true;
uint8_t currentProgramPositionStepIndex = 0;
uint16_t currentProgramPositionIteration = 0;
uint8_t currentProgramSpeedStepIndex = 0;
uint16_t currentProgramSpeedIteration = 0;
MotionRuntime motionRuntime;
SequenceRuntime sequenceRunner;

// Test rig: instead of pulling in the full command_interface.h dispatcher,
// stub processCommand() to record each invocation. This lets us verify
// that the runner fires the right cmds at the right times without booting
// the whole sketch (servo writes etc. would otherwise need PCA9685 mocks).
struct DispatchedCmd { unsigned long ts; std::string cmd; };
static std::vector<DispatchedCmd> dispatched;
// Set by tests that need to simulate the cancellation-on-cmd-dispatch
// behavior real commands have (PLAY/SPLAY/MOTION/STOP/Sn all call
// cancelSequencePlayback). When true, every processCommand invocation
// reaches back into the engine to attempt a cancel.
static bool triggerNestedCancel = false;
void processCommand(char* cmd) {
  dispatched.push_back({ _mock_millis, std::string(cmd) });
  if (triggerNestedCancel) cancelSequencePlayback();
}

// motion_engine.h calls cancelSequencePlayback (cross-engine cancel) but
// we don't include motion_engine.h here, so no stub needed in the other
// direction. We DO need motion's cancel hook though, since command_interface
// isn't loaded either — provide one for the runner's own start path.
void cancelMotionPlayback() {}

uint16_t sequenceDegreesToPulse(uint8_t servo, uint16_t degrees) {
  degrees = constrain(degrees, 0, 180);
  return map(degrees, 0, 180, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
}

#include "../adafruit_16_servo/sequence_engine.h"

static int _tests_run = 0, _tests_passed = 0, _tests_failed = 0;
#define ASSERT_EQ(a, b) do { if ((long long)(a) != (long long)(b)) { char _b[256]; snprintf(_b, sizeof(_b), "FAIL at line %d: got %lld, expected %lld", __LINE__, (long long)(a), (long long)(b)); throw std::runtime_error(_b);} } while(0)
#define ASSERT_STREQ(a, b) do { if (std::string(a) != std::string(b)) { char _b[512]; snprintf(_b, sizeof(_b), "FAIL at line %d: got '%s', expected '%s'", __LINE__, std::string(a).c_str(), std::string(b).c_str()); throw std::runtime_error(_b);} } while(0)
#define ASSERT_TRUE(x)  ASSERT_EQ((int)(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ((int)(x), 0)
#define RUN(name) do { _tests_run++; printf("  " #name " ... "); fflush(stdout); try { test_##name(); _tests_passed++; printf("PASS\n"); } catch (const std::exception& e) { _tests_failed++; printf("%s\n", e.what()); } } while(0)

// Worked example: an "evening-arc" sequence with three steps, mix of
// targets and durations. Matches schema doc §3 worked example shape.
static const char* kBlob =
  "{\"schemaVersion\":1,"
  "\"motions\":[],"
  "\"sequences\":["
    "{\"id\":\"evening-arc\",\"name\":\"Evening Arc\",\"steps\":["
      "{\"cmd\":\"MOTION tidal-drift\",\"durationMs\":800,\"target\":\"all\"},"
      "{\"cmd\":\"ROTATE 30\",\"durationMs\":400,\"target\":2},"
      "{\"cmd\":\"STOP\",\"durationMs\":100,\"target\":\"all\"}"
    "]},"
    "{\"id\":\"chord-test\",\"name\":\"Chord\",\"steps\":["
      "{\"cmd\":\"PLAY 1\",\"durationMs\":0,\"target\":\"all\"},"
      "{\"cmd\":\"SPLAY 1\",\"durationMs\":0,\"target\":\"all\"},"
      "{\"cmd\":\"S0 90\",\"durationMs\":500,\"target\":\"all\"}"
    "]}"
  "],"
  "\"setlists\":[],\"activeSetlistId\":null,\"schedulerConfig\":{}}";

static void reset_state() {
  EEPROM_reset();
  storageInit();
  storageSetBoardId(1);
  motionRuntime.active = false;
  memset(&sequenceRunner, 0, sizeof(sequenceRunner));
  dispatched.clear();
  // Cleared here so nested-cancel mode can't leak between tests if an
  // earlier assertion throws before the test's own cleanup runs.
  triggerNestedCancel = false;
  _mock_millis = 0;
}

static void test_parser_loads_steps_in_order() {
  reset_state();
  SequenceRuntime parsed;
  const char* error = nullptr;
  ASSERT_TRUE(sequenceLoadFromBuffer((const uint8_t*)kBlob, strlen(kBlob), "evening-arc", parsed, &error));
  ASSERT_EQ(parsed.stepCount, 3);
  ASSERT_STREQ(parsed.id, "evening-arc");
  ASSERT_STREQ(parsed.steps[0].cmd, "MOTION tidal-drift");
  ASSERT_EQ(parsed.steps[0].durationMs, 800);
  ASSERT_EQ(parsed.steps[0].target, 0);  // "all" → 0
  ASSERT_STREQ(parsed.steps[1].cmd, "ROTATE 30");
  ASSERT_EQ(parsed.steps[1].target, 2);
  ASSERT_STREQ(parsed.steps[2].cmd, "STOP");
}

static void test_missing_sequence_id() {
  reset_state();
  SequenceRuntime parsed;
  const char* error = nullptr;
  ASSERT_FALSE(sequenceLoadFromBuffer((const uint8_t*)kBlob, strlen(kBlob), "nope", parsed, &error));
  ASSERT_STREQ(error, "sequence-not-found");
}

static void test_per_board_target_filtering_at_runtime() {
  // Board 1: should fire step 0 (all) + step 2 (all), skip step 1 (target=2).
  reset_state();
  storageSetBoardId(1);
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSequenceFromStorage("evening-arc", false, false));

  // First step fires immediately on start. The stub records the raw cmd
  // string from the bake (the real processCommand uppercases in-place,
  // but our stub captures the pointer's contents before mutation runs).
  ASSERT_EQ(dispatched.size(), 1);
  ASSERT_STREQ(dispatched[0].cmd, "MOTION tidal-drift");

  _mock_millis = 800;
  updateSequenceRunner();
  // Step 1 was target=2; board 1 should not fire it. The runner still
  // advances past it, then on the same tick step 2 (target=all) fires.
  // But step 1's duration is 400ms, so updateSequenceRunner at t=800
  // only advances to step 1 (not yet expired). It should record nothing
  // new because step 1 isn't for us.
  ASSERT_EQ(dispatched.size(), 1);  // still just the first MOTION call
  ASSERT_EQ(sequenceRunner.currentStep, 1);
  ASSERT_TRUE(sequenceRunner.active);

  _mock_millis = 1200;
  updateSequenceRunner();
  // Step 1 expired (was at t=800 + 400). Advance to step 2 (target=all),
  // fires "STOP".
  ASSERT_EQ(dispatched.size(), 2);
  ASSERT_STREQ(dispatched[1].cmd, "STOP");
  ASSERT_EQ(sequenceRunner.currentStep, 2);
  ASSERT_TRUE(sequenceRunner.active);

  _mock_millis = 1300;
  updateSequenceRunner();
  ASSERT_FALSE(sequenceRunner.active);
  ASSERT_EQ(dispatched.size(), 2);
}

static void test_board_2_fires_targeted_step() {
  reset_state();
  storageSetBoardId(2);
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSequenceFromStorage("evening-arc", false, false));

  // First step (target=all) fires.
  ASSERT_EQ(dispatched.size(), 1);

  _mock_millis = 800;
  updateSequenceRunner();
  // Step 1 (target=2) should fire on this board.
  ASSERT_EQ(dispatched.size(), 2);
  ASSERT_STREQ(dispatched[1].cmd, "ROTATE 30");
}

static void test_loop_restarts_at_step_zero() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSequenceFromStorage("evening-arc", true, false));
  ASSERT_TRUE(sequenceRunner.loop);

  // Run past the natural end (800 + 400 + 100 = 1300ms).
  _mock_millis = 1400;
  updateSequenceRunner();
  // Should have looped — step 0 fires again.
  ASSERT_TRUE(sequenceRunner.active);
  ASSERT_EQ(sequenceRunner.currentStep, 0);
  // dispatched should now include another MOTION call.
  bool sawSecondMotion = false;
  int motionCount = 0;
  for (auto& d : dispatched) {
    if (d.cmd.find("MOTION") == 0) motionCount++;
  }
  ASSERT_TRUE(motionCount >= 2);
}

static void test_zero_duration_chord_resolves_same_tick() {
  // chord-test has two zero-duration steps followed by a 500ms step.
  // The whole chord should fire on the same tick.
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSequenceFromStorage("chord-test", false, false));

  // start fires step 0. Then a single updateSequenceRunner call at t=0
  // should drain through the zero-duration step 1 and land on step 2.
  ASSERT_EQ(dispatched.size(), 1);
  updateSequenceRunner();
  ASSERT_EQ(dispatched.size(), 3);  // PLAY, SPLAY, S0 90 all fired
  ASSERT_EQ(sequenceRunner.currentStep, 2);
  ASSERT_TRUE(sequenceRunner.active);
}

static void test_cancel_stops_runner() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSequenceFromStorage("evening-arc", true, false));
  ASSERT_TRUE(sequenceRunner.active);
  cancelSequencePlayback();
  ASSERT_FALSE(sequenceRunner.active);
}

static void test_re_entry_guard_lets_step_cmds_dispatch_safely() {
  // With triggerNestedCancel set, every processCommand call invokes
  // cancelSequencePlayback from inside the engine's step dispatch.
  // The sequenceFiringStep guard should suppress that mid-dispatch
  // self-cancel so the runner keeps walking through its steps.
  // Without the guard, the very first step's dispatch would abort
  // the sequence and active would flip to false immediately.
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  triggerNestedCancel = true;
  ASSERT_TRUE(startSequenceFromStorage("chord-test", false, false));

  // The first step fired and invoked the nested cancel. Guard should
  // have suppressed it — runner is still active.
  ASSERT_TRUE(sequenceRunner.active);

  // Drain the chord-test (two zero-duration steps + a 500 ms step).
  // The two zero-duration steps cascade in one updateSequenceRunner
  // tick; each dispatch attempts a nested cancel that the guard
  // suppresses. The runner survives all three step dispatches.
  updateSequenceRunner();
  ASSERT_TRUE(sequenceRunner.active);
  ASSERT_EQ(sequenceRunner.currentStep, 2);
  // All three steps got dispatched in spite of the nested cancels.
  ASSERT_EQ(dispatched.size(), 3);

  // Sanity check: with the trigger cleared, cancelSequencePlayback
  // called from top-level (no firing-step flag set) cancels normally.
  // This guards against accidentally making the guard sticky.
  triggerNestedCancel = false;
  cancelSequencePlayback();
  ASSERT_FALSE(sequenceRunner.active);
}

int main() {
  printf("=== Sequence Engine Tests ===\n");
  RUN(parser_loads_steps_in_order);
  RUN(missing_sequence_id);
  RUN(per_board_target_filtering_at_runtime);
  RUN(board_2_fires_targeted_step);
  RUN(loop_restarts_at_step_zero);
  RUN(zero_duration_chord_resolves_same_tick);
  RUN(cancel_stops_runner);
  RUN(re_entry_guard_lets_step_cmds_dispatch_safely);
  printf("\n%d/%d passed, %d failed\n", _tests_passed, _tests_run, _tests_failed);
  return _tests_failed ? 1 : 0;
}
