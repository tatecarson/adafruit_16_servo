#include "mock/Arduino.h"
#include "mock/EEPROM.h"
#include "mock/Adafruit_PWMServoDriver.h"

#include <stdio.h>
#include <stdexcept>
#include <string.h>

EEPROMClass EEPROM;

#include "../adafruit_16_servo/storage_crc.h"
#include "../adafruit_16_servo/servo_runtime.h"
#include "../adafruit_16_servo/dc_motor.h"
#include "../adafruit_16_servo/storage.h"

Adafruit_PWMServoDriver pwm;
ServoConfig servoConfig[NUM_SERVOS];
ServoState servoState[NUM_SERVOS];
MotorState motorState;

// wave globals removed (servo-dz7)
// Legacy PLAY/SPLAY/RUN-n globals removed in servo-voc.
MotionRuntime motionRuntime;
SequenceRuntime sequenceRunner;

// sequence_engine.h isn't included in this test — provide a stub for the
// cross-engine cancel hook so motion_engine.h compiles standalone.
void cancelSequencePlayback() {}

uint16_t degreesToPulse(uint8_t servo, uint16_t degrees) {
  degrees = constrain(degrees, 0, servoConfig[servo].totalDegrees);
  return map(degrees, 0, servoConfig[servo].totalDegrees, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
}

uint16_t percentToDegrees(uint8_t servo, uint8_t percent) {
  percent = constrain(percent, 0, 100);
  return map(percent, 0, 100, servoConfig[servo].upDegrees, servoConfig[servo].downDegrees);
}

#include "../adafruit_16_servo/motion_engine.h"

static int _tests_run = 0, _tests_passed = 0, _tests_failed = 0;
#define ASSERT_EQ(a, b) do { if ((long long)(a) != (long long)(b)) { char _buf[256]; snprintf(_buf, sizeof(_buf), "FAIL at line %d: got %lld, expected %lld", __LINE__, (long long)(a), (long long)(b)); throw std::runtime_error(_buf);} } while(0)
#define ASSERT_TRUE(x)  ASSERT_EQ((int)(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ((int)(x), 0)
#define RUN(name) do { _tests_run++; printf("  " #name " ... "); fflush(stdout); try { test_##name(); _tests_passed++; printf("PASS\n"); } catch (const std::exception& e) { _tests_failed++; printf("%s\n", e.what()); } } while(0)

static const char* kBlob =
  "{\"schemaVersion\":1,"
  "\"motions\":["
     "{\"id\":\"tidal-drift\",\"name\":\"Tidal Drift\",\"scope\":\"board\",\"durationMs\":1000,"
      "\"tracks\":["
       "{\"kind\":\"servo\",\"boardId\":1,\"channel\":0,\"keyframes\":[{\"atMs\":0,\"value\":0},{\"atMs\":1000,\"value\":100}]},"
       "{\"kind\":\"dc\",\"boardId\":1,\"channel\":0,\"keyframes\":[{\"atMs\":0,\"value\":0},{\"atMs\":500,\"value\":50},{\"atMs\":1000,\"value\":-50}]}"
     "]},"
    "{\"id\":\"other\",\"name\":\"Other\",\"scope\":\"board\",\"durationMs\":100,"
     "\"tracks\":[{\"kind\":\"servo\",\"boardId\":1,\"channel\":1,\"keyframes\":[{\"atMs\":0,\"value\":10},{\"atMs\":100,\"value\":20}]}]}"
  "],"
  "\"sequences\":[],\"setlists\":[],\"activeSetlistId\":null,\"schedulerConfig\":{}}";

static void reset_state() {
  EEPROM_reset();
  storageInit();
  storageSetBoardId(1);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servoConfig[i].minPulse = 150;
    servoConfig[i].maxPulse = 600;
    servoConfig[i].totalDegrees = 1800;
    servoConfig[i].upDegrees = 0;
    servoConfig[i].downDegrees = 1800;
    servoState[i].posPulse = 375;
    servoState[i].targetPulse = 375;
    servoState[i].moving = false;
    servoState[i].stopped = false;
  }
  motorState.currentSpeed = 0;
  motionRuntime.active = false;
  _mock_millis = 0;
}

static void test_loads_motion_case_insensitive() {
  reset_state();
  MotionRuntime parsed;
  const char* error = nullptr;
  ASSERT_TRUE(motionLoadFromBuffer((const uint8_t*)kBlob, strlen(kBlob), "TIDAL-DRIFT", parsed, &error));
  ASSERT_EQ(parsed.durationMs, 1000);
  ASSERT_EQ(parsed.trackCount, 2);
  ASSERT_EQ(parsed.keyframeCount, 5);
  ASSERT_EQ(parsed.tracks[0].kind, MOTION_TRACK_SERVO);
  ASSERT_EQ(parsed.tracks[1].kind, MOTION_TRACK_DC);
}

static void test_skips_tracks_for_other_board() {
  reset_state();
  storageSetBoardId(2);
  MotionRuntime parsed;
  const char* error = nullptr;
  ASSERT_FALSE(motionLoadFromBuffer((const uint8_t*)kBlob, strlen(kBlob), "tidal-drift", parsed, &error));
  ASSERT_TRUE(strcmp(error, "no-local-tracks") == 0);
}

static void test_playback_interpolates_servo_and_dc() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startMotionFromStorage("tidal-drift", false));
  ASSERT_TRUE(motionRuntime.active);
  ASSERT_EQ(servoState[0].posPulse, 150);
  ASSERT_EQ(motorState.currentSpeed, 0);

  _mock_millis = 500;
  updateMotion();
  ASSERT_EQ(servoState[0].posPulse, 375);
  ASSERT_EQ(motorState.currentSpeed, 50);

  _mock_millis = 750;
  updateMotion();
  ASSERT_EQ(servoState[0].posPulse, 487);
  ASSERT_EQ(motorState.currentSpeed, 0);

  _mock_millis = 1000;
  updateMotion();
  ASSERT_FALSE(motionRuntime.active);
  ASSERT_EQ(servoState[0].posPulse, 600);
  ASSERT_EQ(motorState.currentSpeed, -50);
}

static void test_motion_loads_from_large_storage_lease() {
  reset_state();
  static uint8_t largeBlob[5000];
  size_t baseLen = strlen(kBlob);
  memcpy(largeBlob, kBlob, baseLen - 1); // keep the outer closing brace for last
  memset(largeBlob + baseLen - 1, ' ', sizeof(largeBlob) - baseLen);
  largeBlob[sizeof(largeBlob) - 1] = '}';

  ASSERT_TRUE(storageWriteSlot(largeBlob, sizeof(largeBlob)));
  ASSERT_TRUE(storageActiveIsLarge());
  ASSERT_TRUE(startMotionFromStorage("tidal-drift", false));
  ASSERT_TRUE(motionRuntime.active);
  ASSERT_EQ(motionRuntime.trackCount, 2);
}

static void test_completed_motion_can_replay() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));

  ASSERT_TRUE(startMotionFromStorage("tidal-drift", false));
  _mock_millis = 1000;
  updateMotion();
  ASSERT_FALSE(motionRuntime.active);
  ASSERT_EQ(servoState[0].posPulse, 600);

  ASSERT_TRUE(startMotionFromStorage("tidal-drift", false));
  ASSERT_TRUE(motionRuntime.active);
  ASSERT_EQ(servoState[0].posPulse, 150);
  ASSERT_EQ(motorState.currentSpeed, 0);

  _mock_millis = 1500;
  updateMotion();
  ASSERT_TRUE(motionRuntime.active);
  ASSERT_EQ(servoState[0].posPulse, 375);
  ASSERT_EQ(motorState.currentSpeed, 50);
}

// A legacy or hand-authored baked library may still contain the historical
// -100..100 DC range. Loading and playback must clamp it to the Motion-only
// safe range even when the browser editor was bypassed.
static void test_dc_motion_speed_is_safety_clamped() {
  reset_state();
  const char* unsafeDcBlob =
    "{\"schemaVersion\":1,\"motions\":["
      "{\"id\":\"unsafe-dc\",\"name\":\"Unsafe DC\",\"scope\":\"board\",\"durationMs\":1000,"
       "\"tracks\":[{\"kind\":\"dc\",\"boardId\":1,\"channel\":0,"
                   "\"keyframes\":[{\"atMs\":0,\"value\":100},{\"atMs\":1000,\"value\":-100}]}]}"
    "],\"sequences\":[],\"setlists\":[],\"activeSetlistId\":null,\"schedulerConfig\":{}}";

  ASSERT_TRUE(storageWriteSlot((const uint8_t*)unsafeDcBlob, strlen(unsafeDcBlob)));
  ASSERT_TRUE(startMotionFromStorage("unsafe-dc", false));
  ASSERT_EQ(motionRuntime.keyframes[0].value, MOTION_DC_SPEED_MAX);
  ASSERT_EQ(motionRuntime.keyframes[1].value, MOTION_DC_SPEED_MIN);
  ASSERT_EQ(motorState.currentSpeed, MOTION_DC_SPEED_MAX);

  _mock_millis = 1000;
  updateMotion();
  ASSERT_EQ(motorState.currentSpeed, MOTION_DC_SPEED_MIN);
}

static void test_cancel_stops_dc_track() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startMotionFromStorage("tidal-drift", false));

  _mock_millis = 500;
  updateMotion();
  ASSERT_EQ(motorState.currentSpeed, 50);

  cancelMotionPlayback();
  ASSERT_FALSE(motionRuntime.active);
  ASSERT_EQ(motorState.currentSpeed, 0);
}

// Regression for CodeRabbit finding: switching to a Motion with no DC track
// must stop any DC motor speed inherited from the previous Motion. Prior
// behavior let motionLoadFromBuffer zero motionRuntime.active before
// startMotionFromStorage could cancel, leaving the motor spinning forever.
static void test_switching_to_servo_only_motion_stops_dc() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  ASSERT_TRUE(startMotionFromStorage("tidal-drift", false));
  _mock_millis = 500;
  updateMotion();
  ASSERT_EQ(motorState.currentSpeed, 50);

  // "other" has only a servo track on channel 1 — no DC track.
  ASSERT_TRUE(startMotionFromStorage("other", false));
  ASSERT_EQ(motorState.currentSpeed, 0);
}

// Regression for CodeRabbit finding: present-but-malformed boardId used to
// silently include the track on every board because the && short-circuit
// treated a parse failure the same as a missing field.
static void test_malformed_boardid_excludes_track() {
  reset_state();
  const char* bad =
    "{\"schemaVersion\":1,\"motions\":["
      "{\"id\":\"x\",\"name\":\"X\",\"scope\":\"board\",\"durationMs\":100,"
       "\"tracks\":[{\"kind\":\"servo\",\"boardId\":\"oops\",\"channel\":0,"
                   "\"keyframes\":[{\"atMs\":0,\"value\":0},{\"atMs\":100,\"value\":10}]}]}"
    "],\"sequences\":[],\"setlists\":[],\"activeSetlistId\":null,\"schedulerConfig\":{}}";
  MotionRuntime parsed;
  const char* error = nullptr;
  ASSERT_FALSE(motionLoadFromBuffer((const uint8_t*)bad, strlen(bad), "x", parsed, &error));
  ASSERT_TRUE(strcmp(error, "no-local-tracks") == 0);
}

// servo-vna: synchronized Motion start. startMotionFromStorageAt() anchors the
// Motion's startMs to an explicit (cluster-shared) local time. A start in the
// future must hold the current pose until the instant arrives, then begin at
// t=0 — boards arm early and fire together.
static void test_future_start_holds_pose_until_instant() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  // _mock_millis is 0; arm to begin 200ms in the future.
  ASSERT_TRUE(startMotionFromStorageAt("tidal-drift", 200, false));
  ASSERT_TRUE(motionRuntime.active);
  // Held: servo stays at the reset pose (375), NOT slammed to the t=0 value (150).
  ASSERT_EQ(servoState[0].posPulse, 375);
  ASSERT_EQ(motorState.currentSpeed, 0);

  _mock_millis = 199;            // one tick before the start instant
  updateMotion();
  ASSERT_EQ(servoState[0].posPulse, 375);
  ASSERT_TRUE(motionRuntime.active);

  _mock_millis = 200;            // start instant — t=0 keyframe applies
  updateMotion();
  ASSERT_EQ(servoState[0].posPulse, 150);
  ASSERT_EQ(motorState.currentSpeed, 0);
}

// A board that receives the start packet late (or reboots mid-Motion) must
// join at the CORRECT phase, not restart at t=0 — that's what keeps a late
// follower in lockstep.
static void test_past_start_catches_up_to_phase() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  _mock_millis = 700;            // "now" is 500ms past the intended start
  ASSERT_TRUE(startMotionFromStorageAt("tidal-drift", 200, false));
  ASSERT_TRUE(motionRuntime.active);
  // Caught up to elapsed=500: servo at 50% (375), dc at 50 — not t=0.
  ASSERT_EQ(servoState[0].posPulse, 375);
  ASSERT_EQ(motorState.currentSpeed, 50);
}

static void test_implicit_final_hold_and_single_keyframe_track() {
  reset_state();
  const char* compactBlob =
    "{\"schemaVersion\":1,\"motions\":["
      "{\"id\":\"compact\",\"durationMs\":1000,\"tracks\":["
        "{\"kind\":\"servo\",\"boardId\":1,\"channel\":0,"
          "\"keyframes\":[{\"atMs\":0,\"value\":25}]},"
        "{\"kind\":\"dc\",\"boardId\":1,\"channel\":0,"
          "\"keyframes\":[{\"atMs\":0,\"value\":10},{\"atMs\":500,\"value\":20}]}"
      "]}],\"sequences\":[],\"setlists\":[],\"activeSetlistId\":null,\"schedulerConfig\":{}}";

  ASSERT_TRUE(storageWriteSlot((const uint8_t*)compactBlob, strlen(compactBlob)));
  ASSERT_TRUE(startMotionFromStorage("compact", false));
  ASSERT_EQ(motionRuntime.keyframeCount, 3);
  ASSERT_EQ(servoState[0].posPulse, 262);  // 25% of 150..600
  ASSERT_EQ(motorState.currentSpeed, 10);

  _mock_millis = 750;
  updateMotion();
  ASSERT_TRUE(motionRuntime.active);
  ASSERT_EQ(servoState[0].posPulse, 262);  // single keyframe holds
  ASSERT_EQ(motorState.currentSpeed, 20); // last DC keyframe holds to duration

  _mock_millis = 1000;
  updateMotion();
  ASSERT_FALSE(motionRuntime.active);
  ASSERT_EQ(motorState.currentSpeed, 20);
}

static void test_prepared_start_arms_motion_and_servo_glide() {
  reset_state();
  ASSERT_TRUE(storageWriteSlot((const uint8_t*)kBlob, strlen(kBlob)));
  motorState.currentSpeed = 40;

  ASSERT_TRUE(startMotionPreparedFromStorage("tidal-drift", 7700, false));
  ASSERT_TRUE(motionRuntime.active);
  ASSERT_EQ(motionRuntime.startMs, 7700);
  // A pre-roll must NOT touch motor state (servo-i0v). Sequencer DC lanes bake
  // to `ROTATE` chord steps that fire immediately before this command, so
  // zeroing here would silently wipe the operator's authored speed and leave
  // the motor dead for the whole Motion. That was the original bug.
  ASSERT_EQ(motorState.currentSpeed, 40);
  ASSERT_TRUE(servoState[0].moving);
  ASSERT_TRUE(servoState[0].linearMove);
  ASSERT_EQ(servoState[0].startPulse, 375);
  ASSERT_EQ(servoState[0].targetPulse, 150);
  ASSERT_EQ(servoState[0].moveDurationMs, 7700);

  _mock_millis = 7699;
  updateMotion();
  ASSERT_TRUE(motionRuntime.active);
  ASSERT_EQ(servoState[0].posPulse, 375); // Motion itself is still gated
}

int main() {
  printf("=== Motion Engine Tests ===\n");
  RUN(loads_motion_case_insensitive);
  RUN(skips_tracks_for_other_board);
  RUN(playback_interpolates_servo_and_dc);
  RUN(motion_loads_from_large_storage_lease);
  RUN(completed_motion_can_replay);
  RUN(dc_motion_speed_is_safety_clamped);
  RUN(cancel_stops_dc_track);
  RUN(switching_to_servo_only_motion_stops_dc);
  RUN(malformed_boardid_excludes_track);
  RUN(future_start_holds_pose_until_instant);
  RUN(past_start_catches_up_to_phase);
  RUN(implicit_final_hold_and_single_keyframe_track);
  RUN(prepared_start_arms_motion_and_servo_glide);
  printf("\n%d/%d passed, %d failed\n", _tests_passed, _tests_run, _tests_failed);
  return _tests_failed ? 1 : 0;
}
