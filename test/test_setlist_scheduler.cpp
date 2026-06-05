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

// Legacy PLAY/SPLAY/RUN-n globals removed in servo-voc.
MotionRuntime motionRuntime;
SequenceRuntime sequenceRunner;
SetlistRuntime setlistScheduler;

void cancelMotionPlayback() {}

// Forward-declared so the processCommand stub can simulate the real STOP
// handler (which cancels the scheduler) and the real RUN handler (which
// starts a sequence → sequenceRunner.active = true).
inline void cancelSetlistPlayback();

// Test rig. The scheduler fires entries through dispatchCommand (the mirroring
// entry point), so that's what we record + simulate. processCommand is only
// referenced by sequence_engine.h linkage and stays a no-op here.
//   - "RUN <id>" → a sequence starts (sequenceRunner.active = true), except
//     "ghost" which simulates a not-found sequence (start fails, stays false).
//   - "STOP"     → the real STOP handler cancels the setlist scheduler. The
//     scheduler guards its own STOP dispatch, so only a user STOP cancels.
struct DispatchedCmd { unsigned long ts; std::string cmd; };
static std::vector<DispatchedCmd> dispatched;
void processCommand(char* cmd) { (void)cmd; }
void dispatchCommand(const char* cmd, bool fromNetwork) {
  (void)fromNetwork;
  dispatched.push_back({ _mock_millis, std::string(cmd) });
  if (strncmp(cmd, "RUN ", 4) == 0 && strcmp(cmd, "RUN AUTO") != 0) {
    sequenceRunner.active = (strstr(cmd, "ghost") == nullptr);
  } else if (strcmp(cmd, "STOP") == 0) {
    sequenceRunner.active = false;
    cancelSetlistPlayback();
  }
}

#include "../adafruit_16_servo/sequence_engine.h"
#include "../adafruit_16_servo/setlist_scheduler.h"

static int _tests_run = 0, _tests_passed = 0, _tests_failed = 0;
#define ASSERT_EQ(a, b) do { if ((long long)(a) != (long long)(b)) { char _b[256]; snprintf(_b, sizeof(_b), "FAIL at line %d: got %lld, expected %lld", __LINE__, (long long)(a), (long long)(b)); throw std::runtime_error(_b);} } while(0)
#define ASSERT_STREQ(a, b) do { if (std::string(a) != std::string(b)) { char _b[512]; snprintf(_b, sizeof(_b), "FAIL at line %d: got '%s', expected '%s'", __LINE__, std::string(a).c_str(), std::string(b).c_str()); throw std::runtime_error(_b);} } while(0)
#define ASSERT_TRUE(x)  ASSERT_EQ((int)(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ((int)(x), 0)
#define RUN(name) do { _tests_run++; printf("  " #name " ... "); fflush(stdout); try { test_##name(); _tests_passed++; printf("PASS\n"); } catch (const std::exception& e) { _tests_failed++; printf("%s\n", e.what()); } } while(0)

// One ordered + one shuffle setlist, two referenced sequences. Scheduler-
// relevant fields only; sequences kept trivial since the stub fakes playback.
static const char* kBlob =
  "{\"schemaVersion\":1,"
  "\"motions\":[],"
  "\"sequences\":["
    "{\"id\":\"arc\",\"name\":\"Arc\",\"steps\":[{\"cmd\":\"S0 90\",\"durationMs\":100,\"target\":\"all\"}]},"
    "{\"id\":\"pulse\",\"name\":\"Pulse\",\"steps\":[{\"cmd\":\"S1 90\",\"durationMs\":100,\"target\":\"all\"}]},"
    "{\"id\":\"drift\",\"name\":\"Drift\",\"steps\":[{\"cmd\":\"S2 90\",\"durationMs\":100,\"target\":\"all\"}]}"
  "],"
  "\"setlists\":["
    "{\"id\":\"evening\",\"name\":\"Evening\",\"mode\":\"ordered\",\"entries\":["
      "{\"seqId\":\"arc\",\"repeat\":1,\"gapMs\":500},"
      "{\"seqId\":\"pulse\",\"repeat\":2,\"gapMs\":0}"
    "]},"
    "{\"id\":\"gallery\",\"name\":\"Gallery\",\"mode\":\"shuffle\",\"entries\":["
      "{\"seqId\":\"arc\",\"repeat\":1,\"gapMs\":0,\"weight\":1},"
      "{\"seqId\":\"pulse\",\"repeat\":1,\"gapMs\":0,\"weight\":1},"
      "{\"seqId\":\"drift\",\"repeat\":1,\"gapMs\":0,\"weight\":1}"
    "],\"shuffleRules\":{\"minGapEntries\":2,\"seed\":42}}"
  "],"
  "\"activeSetlistId\":\"evening\","
  "\"schedulerConfig\":{\"leaderBoardId\":1,\"graceMs\":10000}}";

static void reset_state() {
  EEPROM_reset();
  storageInit();
  storageSetBoardId(1);
  motionRuntime.active = false;
  memset(&sequenceRunner, 0, sizeof(sequenceRunner));
  memset(&setlistScheduler, 0, sizeof(setlistScheduler));
  dispatched.clear();
  _mock_millis = 0;
}

// ---- Cycle 1: parsing --------------------------------------------

static void test_parse_ordered_setlist() {
  reset_state();
  SetlistRuntime out;
  const char* error = nullptr;
  ASSERT_TRUE(setlistLoadFromBuffer((const uint8_t*)kBlob, strlen(kBlob), "evening", out, &error));
  ASSERT_STREQ(out.id, "evening");
  ASSERT_FALSE(out.shuffle);
  ASSERT_EQ(out.entryCount, 2);
  ASSERT_STREQ(out.entries[0].seqId, "arc");
  ASSERT_EQ(out.entries[0].repeat, 1);
  ASSERT_EQ(out.entries[0].gapMs, 500);
  ASSERT_STREQ(out.entries[1].seqId, "pulse");
  ASSERT_EQ(out.entries[1].repeat, 2);
  ASSERT_EQ(out.entries[1].gapMs, 0);
}

static void test_parse_shuffle_setlist_rules() {
  reset_state();
  SetlistRuntime out;
  const char* error = nullptr;
  ASSERT_TRUE(setlistLoadFromBuffer((const uint8_t*)kBlob, strlen(kBlob), "gallery", out, &error));
  ASSERT_TRUE(out.shuffle);
  ASSERT_EQ(out.entryCount, 3);
  ASSERT_EQ(out.minGapEntries, 2);
  ASSERT_EQ(out.entries[1].weight, 1);
}

static void test_parse_missing_setlist() {
  reset_state();
  SetlistRuntime out;
  const char* error = nullptr;
  ASSERT_FALSE(setlistLoadFromBuffer((const uint8_t*)kBlob, strlen(kBlob), "nope", out, &error));
  ASSERT_STREQ(error, "setlist-not-found");
}

static void test_parse_active_id_and_leader() {
  reset_state();
  char id[SETLIST_ID_MAX_LEN + 1];
  ASSERT_TRUE(activeSetlistId((const uint8_t*)kBlob, strlen(kBlob), id, sizeof(id)));
  ASSERT_STREQ(id, "evening");
  ASSERT_EQ(schedulerLeaderBoardId((const uint8_t*)kBlob, strlen(kBlob)), 1);
}

// Out-of-range numeric fields must fail the load rather than wrap on narrowing
// (servo-dos review): repeat/weight > 65535, minGapEntries > 255.
static void test_parse_rejects_oversized_numeric_fields() {
  reset_state();
  SetlistRuntime out;
  const char* error = nullptr;
  const char* repeatBlob =
    "{\"schemaVersion\":1,\"motions\":[],\"sequences\":[],"
    "\"setlists\":[{\"id\":\"x\",\"name\":\"X\",\"mode\":\"ordered\","
      "\"entries\":[{\"seqId\":\"a\",\"repeat\":70000,\"gapMs\":0}]}],"
    "\"activeSetlistId\":\"x\",\"schedulerConfig\":{}}";
  ASSERT_FALSE(setlistLoadFromBuffer((const uint8_t*)repeatBlob, strlen(repeatBlob), "x", out, &error));

  const char* gapBlob =
    "{\"schemaVersion\":1,\"motions\":[],\"sequences\":[],"
    "\"setlists\":[{\"id\":\"y\",\"name\":\"Y\",\"mode\":\"shuffle\","
      "\"entries\":[{\"seqId\":\"a\",\"repeat\":1,\"gapMs\":0,\"weight\":1}],"
      "\"shuffleRules\":{\"minGapEntries\":256}}],"
    "\"activeSetlistId\":\"y\",\"schedulerConfig\":{}}";
  ASSERT_FALSE(setlistLoadFromBuffer((const uint8_t*)gapBlob, strlen(gapBlob), "y", out, &error));
  ASSERT_STREQ(error, "bad-shuffle-rules");
}

// ---- Cycle 2: playback -------------------------------------------

// Helper: simulate the currently-playing sequence finishing.
static void finishSequence() { sequenceRunner.active = false; }

static void test_ordered_plays_in_order_with_repeat_and_loop() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startActiveSetlist());           // active = "evening" (ordered)
  ASSERT_TRUE(setlistScheduler.active);
  ASSERT_EQ(dispatched.size(), 1);
  ASSERT_STREQ(dispatched[0].cmd, "RUN arc");

  // While the sequence plays, the scheduler waits.
  updateSetlistScheduler();
  ASSERT_EQ(dispatched.size(), 1);

  // arc (repeat 1) finishes → STOP, then GAP of 500ms.
  finishSequence();
  updateSetlistScheduler();
  ASSERT_EQ(dispatched.size(), 2);
  ASSERT_STREQ(dispatched[1].cmd, "STOP");
  ASSERT_TRUE(setlistScheduler.active);        // internal STOP did NOT self-cancel

  // Gap not elapsed yet.
  _mock_millis += 400;
  updateSetlistScheduler();
  ASSERT_EQ(dispatched.size(), 2);

  // Gap elapsed → advance to pulse (entry 1).
  _mock_millis += 100;
  updateSetlistScheduler();
  ASSERT_EQ(dispatched.size(), 3);
  ASSERT_STREQ(dispatched[2].cmd, "RUN pulse");

  // pulse has repeat 2: first finish re-fires the same seqId.
  finishSequence();
  updateSetlistScheduler();
  ASSERT_EQ(dispatched.size(), 4);
  ASSERT_STREQ(dispatched[3].cmd, "RUN pulse");

  // Second finish → STOP, gap 0 → loops back to entry 0 (arc).
  finishSequence();
  updateSetlistScheduler();                    // STOP
  ASSERT_STREQ(dispatched[4].cmd, "STOP");
  updateSetlistScheduler();                    // gap 0 → advance, loops to arc
  ASSERT_STREQ(dispatched[5].cmd, "RUN arc");
  ASSERT_EQ(setlistScheduler.currentEntry, 0);
}

static void test_non_leader_does_not_schedule() {
  reset_state();
  storageSetBoardId(2);                         // blob leaderBoardId = 1
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_FALSE(startActiveSetlist());           // follower: no-op
  ASSERT_FALSE(setlistScheduler.active);
  ASSERT_EQ(dispatched.size(), 0);
}

static void test_user_stop_cancels_scheduler() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startActiveSetlist());
  ASSERT_TRUE(setlistScheduler.active);
  cancelSetlistPlayback();                      // user STOP (outside guard)
  ASSERT_FALSE(setlistScheduler.active);
}

// ---- Cycle 3: shuffle + robustness -------------------------------

// Collect the seqIds of the next `n` shuffle picks by driving the scheduler.
static std::vector<std::string> collectPicks(int n) {
  std::vector<std::string> picks;
  // The first pick already fired at start; capture it from dispatched.
  picks.push_back(dispatched.back().cmd);
  while ((int)picks.size() < n) {
    finishSequence();
    updateSetlistScheduler();        // → STOP, GAP
    _mock_millis += 1;
    updateSetlistScheduler();        // gap 0 → advance → fire next
    picks.push_back(dispatched.back().cmd);
  }
  return picks;
}

static void test_shuffle_is_deterministic_for_seed() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSetlistFromStorage("gallery"));   // seed 42, minGapEntries 2
  std::vector<std::string> first = collectPicks(6);

  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSetlistFromStorage("gallery"));
  std::vector<std::string> second = collectPicks(6);

  ASSERT_EQ(first.size(), 6);
  for (size_t i = 0; i < first.size(); i++) ASSERT_STREQ(first[i], second[i]);
}

static void test_shuffle_honors_min_gap_entries() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startSetlistFromStorage("gallery"));   // 3 entries, minGapEntries 2
  std::vector<std::string> picks = collectPicks(9);
  // With minGapEntries=2 and 3 entries, no seqId may recur within any window
  // of 2 consecutive picks (i.e. no two of any 3 consecutive picks match).
  for (size_t i = 2; i < picks.size(); i++) {
    ASSERT_TRUE(picks[i] != picks[i - 1]);
    ASSERT_TRUE(picks[i] != picks[i - 2]);
  }
}

static void test_bad_seqid_advances_without_hanging() {
  // A 1-entry ordered setlist whose seq doesn't exist: each tick makes exactly
  // one transition, so the scheduler advances forever without hot-spinning.
  static const char* ghostBlob =
    "{\"schemaVersion\":1,\"motions\":[],\"sequences\":[],"
    "\"setlists\":[{\"id\":\"g\",\"name\":\"G\",\"mode\":\"ordered\","
      "\"entries\":[{\"seqId\":\"ghost\",\"repeat\":1,\"gapMs\":0}]}],"
    "\"activeSetlistId\":\"g\",\"schedulerConfig\":{\"leaderBoardId\":1}}";
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)ghostBlob, strlen(ghostBlob)));
  ASSERT_TRUE(startActiveSetlist());
  ASSERT_STREQ(dispatched[0].cmd, "RUN ghost");
  ASSERT_FALSE(sequenceRunner.active);          // start "failed"

  updateSetlistScheduler();                     // PLAYING !active → STOP, GAP
  ASSERT_STREQ(dispatched[1].cmd, "STOP");
  updateSetlistScheduler();                     // GAP gap0 → re-fire ghost
  ASSERT_STREQ(dispatched[2].cmd, "RUN ghost");
  ASSERT_TRUE(setlistScheduler.active);         // still alive, made progress

  cancelSetlistPlayback();
  ASSERT_FALSE(setlistScheduler.active);
}

int main() {
  printf("=== Setlist Scheduler Tests ===\n");
  RUN(parse_ordered_setlist);
  RUN(parse_shuffle_setlist_rules);
  RUN(parse_missing_setlist);
  RUN(parse_active_id_and_leader);
  RUN(parse_rejects_oversized_numeric_fields);
  RUN(ordered_plays_in_order_with_repeat_and_loop);
  RUN(non_leader_does_not_schedule);
  RUN(user_stop_cancels_scheduler);
  RUN(shuffle_is_deterministic_for_seed);
  RUN(shuffle_honors_min_gap_entries);
  RUN(bad_seqid_advances_without_hanging);
  printf("\n%d/%d passed, %d failed\n", _tests_passed, _tests_run, _tests_failed);
  return _tests_failed ? 1 : 0;
}
